/*
 Makibox A6 firmware, based on Sprinter (master branch, 1 Sep 2012).
 Designed for Printrboard (Rev B).
 
 ---

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#include <avr/interrupt.h>
#include "pgmspace.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <util/crc16.h>

#include "config.h"
#include "board_io.h"
#include "makibox.h"
#include "heater.h"
#include "serial.h"
#include "command.h"
#include "planner.h"

#ifdef USE_ARC_FUNCTION
  #include "arc_func.h"
#endif

#ifdef USE_EEPROM_SETTINGS
  #include "store_eeprom.h"
#endif

extern "C" void __cxa_pure_virtual();

#define  FORCE_INLINE __attribute__((always_inline)) inline


void analogWrite_check(uint8_t check_pin, int val);
void cmdbuf_read_serial();
void cmdbuf_process();
void execute_command();
void prepare_move();
void prepare_arc_move(char isclockwise);
#ifdef USE_ARC_FUNCTION
  void get_arc_coordinates();
#endif
void kill();

#if (MINIMUM_FAN_START_SPEED > 0)
void manage_fan_start_speed(void);
#endif

extern "C" {
  void setup();
  void loop();
}

void read_command();
void process_command(const char *cmdstr);
void execute_gcode(struct command *cmd);
void execute_mcode(struct command *cmd);
void get_coordinates(struct command *cmd);
void get_arc_coordinates(struct command *cmd);
void execute_m92(struct command *cmd);
void execute_m201(struct command *cmd);

#ifndef CRITICAL_SECTION_START
#define CRITICAL_SECTION_START  unsigned char _sreg = SREG; cli()
#define CRITICAL_SECTION_END    SREG = _sreg
#endif //CRITICAL_SECTION_START

// look here for descriptions of gcodes: http://linuxcnc.org/handbook/gcode/g-code.html
// http://objects.reprap.org/wiki/Mendel_User_Manual:_RepRapGCodes

//Implemented Codes
//-------------------
// G0  -> G1
// G1  - Coordinated Movement X Y Z E
// G2  - CW ARC
// G3  - CCW ARC
// G4  - Dwell S<seconds> or P<milliseconds>
// G28 - Home all Axis
// G90 - Use Absolute Coordinates
// G91 - Use Relative Coordinates
// G92 - Set current position to cordinates given

//RepRap M Codes
// M104 - Set extruder target temp
// M105 - Read current temp
// M106 - Fan on
// M107 - Fan off
// M109 - Wait for extruder current temp to reach target temp.
// M114 - Display current position

//Custom M Codes
// M80  - Turn on Power Supply
// M81  - Turn off Power Supply
// M82  - Set E codes absolute (default)
// M83  - Set E codes relative while in Absolute Coordinates (G90) mode
// M84  - Disable steppers until next move, 
//        or use S<seconds> to specify an inactivity timeout, after which the steppers will be disabled.  S0 to disable the timeout.
// M85  - Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
// M92  - Set axis_steps_per_unit - same syntax as G92
// M93  - Send axis_steps_per_unit
// M115	- Capabilities string
// M119 - Show Endstopper State 
// M140 - Set bed target temp
// M190 - Wait for bed current temp to reach target temp.
// M201 - Set maximum acceleration in units/s^2 for print moves (M201 X1000 Y1000)
// M202 - Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in mm/sec
// M203 - Set temperture monitor to Sx
// M204 - Set default acceleration: S normal moves T filament only moves (M204 S3000 T7000) in mm/sec^2
// M205 - advanced settings:  minimum travel speed S=while printing T=travel only,  X=maximum xy jerk, Z=maximum Z jerk
// M206 - set additional homing offset

// M220 - set speed factor override percentage S=factor in percent 
// M221 - set extruder multiply factor S100 --> original Extrude Speed 

// M301 - Set PID parameters P I and D
// M303 - PID relay autotune S<temperature> sets the target temperature. (default target temperature = 150C)

// M400 - Finish all moves

// M500 - stores paramters in EEPROM
// M501 - reads parameters from EEPROM (if you need to reset them after you changed them temporarily).
// M502 - reverts to the default "factory settings". You still need to store them in EEPROM afterwards if you want to.
// M503 - Print settings

// Debug feature / Testing the PID for Hotend
// M601 - Show Temp jitter from Extruder (min / max value from Hotend Temperature while printing)
// M602 - Reset Temp jitter from Extruder (min / max val) --> Don't use it while Printing
// M603 - Show Free Ram


static const char VERSION_TEXT[] = "1.3.22T / 20.08.2012";

#ifdef PIDTEMP
 unsigned int PID_Kp = PID_PGAIN, PID_Ki = PID_IGAIN, PID_Kd = PID_DGAIN;
#endif

// X, Y, Z and E max acceleration in mm/s^2 for printing moves or retracts
long  max_acceleration_units_per_sq_second[4] = _MAX_ACCELERATION_UNITS_PER_SQ_SECOND;

//adjustable feed factor for online tuning printer speed
volatile int feedmultiply=100; //100->original / 200 -> Factor 2 / 50 -> Factor 0.5
int saved_feedmultiply;
volatile bool feedmultiplychanged=false;

float destination[NUM_AXIS] = {0.0, 0.0, 0.0, 0.0};
float current_position[NUM_AXIS] = {0.0, 0.0, 0.0, 0.0};
float add_homing[3]={0,0,0};

bool home_all_axis = true;
//unsigned ?? ToDo: Check
int feedrate = 1500, next_feedrate, saved_feedrate;

bool relative_mode = false;  //Determines Absolute or Relative Coordinates

#ifdef USE_ARC_FUNCTION
//For arc center point coordinates, sent by commands G2/G3
float offset[3] = {0.0, 0.0, 0.0};
#endif

#if (MINIMUM_FAN_START_SPEED > 0)
  unsigned char fan_last_speed = 0;
  unsigned char fan_org_start_speed = 0;
  unsigned long previous_millis_fan_start = 0;
#endif

// comm variables and Commandbuffer
// MAX_CMD_SIZE does not include the trailing \0 that terminates the string.
#define MAX_CMD_SIZE 95
char cmdbuf[MAX_CMD_SIZE+1];
unsigned char bufpos = 0;
uint32_t cmdseqnbr = 0;
uint8_t cmdready = 0;

//Send Temperature in °C to Host
int hotendtC = 0, bedtempC = 0;
       
//Inactivity shutdown variables
unsigned long previous_millis_cmd = 0;
unsigned long max_inactive_time = 0;
unsigned long stepper_inactive_time = 0;

//Temp Monitor for repetier
unsigned char manage_monitor = 255;


void enable_x()
{
#if X_ENABLE_PIN > -1
  WRITE(X_ENABLE_PIN, X_ENABLE_ON);
#endif
}
void disable_x()
{
#if X_ENABLE_PIN > -1
  WRITE(X_ENABLE_PIN,!X_ENABLE_ON);
#endif
}
void enable_y()
{
#if Y_ENABLE_PIN > -1
  WRITE(Y_ENABLE_PIN, Y_ENABLE_ON);
#endif
}
void disable_y()
{
#if Y_ENABLE_PIN > -1
  WRITE(Y_ENABLE_PIN,!Y_ENABLE_ON);
#endif
}
void enable_z()
{
#if Z_ENABLE_PIN > -1
  WRITE(Z_ENABLE_PIN, Z_ENABLE_ON);
#endif
}
void disable_z()
{
#if Z_ENABLE_PIN > -1
  WRITE(Z_ENABLE_PIN,!Z_ENABLE_ON);
#endif
}
void enable_e()
{
#if E_ENABLE_PIN > -1
  WRITE(E_ENABLE_PIN, E_ENABLE_ON);
#endif
}
void disable_e()
{
#if E_ENABLE_PIN > -1
  WRITE(E_ENABLE_PIN,!E_ENABLE_ON);
#endif
}


#define MIN(a,b) ((a)<(b)?(a):(b))                                               
#define MAX(a,b) ((a)>(b)?(a):(b))                                               
#define CONSTRAIN(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt))) 


int FreeRam1(void)
{
  extern int  __bss_end;
  extern int* __brkval;
  int free_memory;

  if (reinterpret_cast<int>(__brkval) == 0)
  {
    // if no heap use from end of bss section
    free_memory = reinterpret_cast<int>(&free_memory) - reinterpret_cast<int>(&__bss_end);
  }
  else
  {
    // use from top of stack to heap
    free_memory = reinterpret_cast<int>(&free_memory) - reinterpret_cast<int>(__brkval);
  }
  
  return free_memory;
}

//------------------------------------------------
//Function the check the Analog OUT pin for not using the Timer1
//------------------------------------------------
void analogWrite_check(uint8_t check_pin, int val)
{
  #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) 
  //Atmega168/328 can't use OCR1A and OCR1B
  //These are pins PB1/PB2 or on Arduino D9/D10
    if((check_pin != 9) && (check_pin != 10))
    {
        analogWrite(check_pin, val);
    }
  #endif
  
  #if defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__) 
  //Atmega664P/1284P can't use OCR1A and OCR1B
  //These are pins PD4/PD5 or on Arduino D12/D13
    if((check_pin != 12) && (check_pin != 13))
    {
        analogWrite(check_pin, val);
    }
  #endif

  #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) 
  //Atmega1280/2560 can't use OCR1A, OCR1B and OCR1C
  //These are pins PB5,PB6,PB7 or on Arduino D11,D12 and D13
    if((check_pin != 11) && (check_pin != 12) && (check_pin != 13))
    {
        analogWrite(check_pin, val);
    }
  #endif  
}


//------------------------------------------------
// Init 
//------------------------------------------------
void setup()
{ 
  serial_init(); 
  serial_send("// Makibox %s started.\r\n", VERSION_TEXT);
  
  //Initialize Dir Pins
  #if X_DIR_PIN > -1
    SET_OUTPUT(X_DIR_PIN);
  #endif
  #if Y_DIR_PIN > -1 
    SET_OUTPUT(Y_DIR_PIN);
  #endif
  #if Z_DIR_PIN > -1 
    SET_OUTPUT(Z_DIR_PIN);
  #endif
  #if E_DIR_PIN > -1 
    SET_OUTPUT(E_DIR_PIN);
  #endif
  
  //Initialize Enable Pins - steppers default to disabled.
  
  #if (X_ENABLE_PIN > -1)
    SET_OUTPUT(X_ENABLE_PIN);
  if(!X_ENABLE_ON) WRITE(X_ENABLE_PIN,HIGH);
  #endif
  #if (Y_ENABLE_PIN > -1)
    SET_OUTPUT(Y_ENABLE_PIN);
  if(!Y_ENABLE_ON) WRITE(Y_ENABLE_PIN,HIGH);
  #endif
  #if (Z_ENABLE_PIN > -1)
    SET_OUTPUT(Z_ENABLE_PIN);
  if(!Z_ENABLE_ON) WRITE(Z_ENABLE_PIN,HIGH);
  #endif
  #if (E_ENABLE_PIN > -1)
    SET_OUTPUT(E_ENABLE_PIN);
  if(!E_ENABLE_ON) WRITE(E_ENABLE_PIN,HIGH);
  #endif

  #ifdef CONTROLLERFAN_PIN
    SET_OUTPUT(CONTROLLERFAN_PIN); //Set pin used for driver cooling fan
  #endif
  
  #ifdef EXTRUDERFAN_PIN
    SET_OUTPUT(EXTRUDERFAN_PIN); //Set pin used for extruder cooling fan
  #endif
  
  //endstops and pullups
  #ifdef ENDSTOPPULLUPS
  #if X_MIN_PIN > -1
    SET_INPUT(X_MIN_PIN); 
    WRITE(X_MIN_PIN,HIGH);
  #endif
  #if X_MAX_PIN > -1
    SET_INPUT(X_MAX_PIN); 
    WRITE(X_MAX_PIN,HIGH);
  #endif
  #if Y_MIN_PIN > -1
    SET_INPUT(Y_MIN_PIN); 
    WRITE(Y_MIN_PIN,HIGH);
  #endif
  #if Y_MAX_PIN > -1
    SET_INPUT(Y_MAX_PIN); 
    WRITE(Y_MAX_PIN,HIGH);
  #endif
  #if Z_MIN_PIN > -1
    SET_INPUT(Z_MIN_PIN); 
    WRITE(Z_MIN_PIN,HIGH);
  #endif
  #if Z_MAX_PIN > -1
    SET_INPUT(Z_MAX_PIN); 
    WRITE(Z_MAX_PIN,HIGH);
  #endif
  #else
  #if X_MIN_PIN > -1
    SET_INPUT(X_MIN_PIN); 
  #endif
  #if X_MAX_PIN > -1
    SET_INPUT(X_MAX_PIN); 
  #endif
  #if Y_MIN_PIN > -1
    SET_INPUT(Y_MIN_PIN); 
  #endif
  #if Y_MAX_PIN > -1
    SET_INPUT(Y_MAX_PIN); 
  #endif
  #if Z_MIN_PIN > -1
    SET_INPUT(Z_MIN_PIN); 
  #endif
  #if Z_MAX_PIN > -1
    SET_INPUT(Z_MAX_PIN); 
  #endif
  #endif
  
  #if (HEATER_0_PIN > -1) 
    SET_OUTPUT(HEATER_0_PIN);
    WRITE(HEATER_0_PIN,LOW);
  #endif  
  #if (HEATER_1_PIN > -1) 
    SET_OUTPUT(HEATER_1_PIN);
    WRITE(HEATER_1_PIN,LOW);
  #endif  
  
  //Initialize Fan Pin
  #if (FAN_PIN > -1) 
    SET_OUTPUT(FAN_PIN);
  #endif
  
  //Initialize Alarm Pin
  #if (ALARM_PIN > -1) 
    SET_OUTPUT(ALARM_PIN);
    WRITE(ALARM_PIN,LOW);
  #endif

  //Initialize LED Pin
  #if (LED_PIN > -1) 
    SET_OUTPUT(LED_PIN);
    WRITE(LED_PIN,LOW);
  #endif 
  
//Initialize Step Pins
  #if (X_STEP_PIN > -1) 
    SET_OUTPUT(X_STEP_PIN);
  #endif  
  #if (Y_STEP_PIN > -1) 
    SET_OUTPUT(Y_STEP_PIN);
  #endif  
  #if (Z_STEP_PIN > -1) 
    SET_OUTPUT(Z_STEP_PIN);
  #endif  
  #if (E_STEP_PIN > -1) 
    SET_OUTPUT(E_STEP_PIN);
  #endif  

#ifdef HEATER_USES_MAX6675
  SET_OUTPUT(SCK_PIN);
  WRITE(SCK_PIN,0);
  
  SET_OUTPUT(MOSI_PIN);
  WRITE(MOSI_PIN,1);
  
  SET_INPUT(MISO_PIN);
  WRITE(MISO_PIN,1);
  
  SET_OUTPUT(MAX6675_SS);
  WRITE(MAX6675_SS,1);
#endif  
 
  #if defined(PID_SOFT_PWM) || (defined(FAN_SOFT_PWM) && (FAN_PIN > -1))
  serial_send("// Soft PWM Init\r\n");
  init_Timer2_softpwm();
  #endif
  
  serial_send("// Planner Init\r\n");
  plan_init();  // Initialize planner;

  serial_send("// Stepper Timer init\r\n");
  st_init();    // Initialize stepper

  #ifdef USE_EEPROM_SETTINGS
  //first Value --> Init with default
  //second value --> Print settings to UART
  EEPROM_RetrieveSettings(false,false);
  #endif
  
  #ifdef PIDTEMP
  updatePID();
  #endif

  //Planner Buffer Size
  //serial_send("// Plan Buffer Size: %d / %d\r\n", (int)sizeof(block_t)*BLOCK_BUFFER_SIZE, BLOCK_BUFFER_SIZE);
  
  //Free Ram
  serial_send("// Free Ram: %d\r\n", FreeRam1());
  
  for(int8_t i=0; i < NUM_AXIS; i++)
  {
    axis_steps_per_sqr_second[i] = max_acceleration_units_per_sq_second[i] * axis_steps_per_unit[i];
  }

  // ensure all our init stuff gets sent
  serial_flush();
}



//------------------------------------------------
//MAIN LOOP
//------------------------------------------------
void loop()
{
  // Read a command from the UART and process it.
  read_command();

  // Manage the heater and fan.
  manage_inactivity(1);
  #if (MINIMUM_FAN_START_SPEED > 0)
    manage_fan_start_speed();
  #endif
}


//------------------------------------------------
//READ COMMAND
//
//  This function reads characters from the UART and
//  places them in the command buffer.  The first    
//  newline (either \r or \n) terminates the command.
//
//  If a command exceeds MAX_CMD_SIZE, it is simply
//  truncated.
//
//  TODO:  recognize ';' comments and ignore them?
//
void read_command() 
{ 
  while (serial_can_read())
  {
    char ch = serial_read();
    if (ch == '\n' || ch == '\r')
    {
      // Newline marks end of this command;  terminate
      // string and process it.
      cmdbuf[bufpos] = '\0';
      process_command(cmdbuf);
      bufpos = 0;
      // Flush any output which may not have been sent 
      // at the end of command execution.
      serial_flush();
      break;
    }
    cmdbuf[bufpos++] = ch;
    if (bufpos > MAX_CMD_SIZE)
    {
        // TODO:  can we do something more intelligent than
        // just silently truncating the command?
        bufpos--;
    }
  }
}

//------------------------------------------------
//PARSE COMMAND
//
//  These functions parse "words" (parameter name and
//  int/float value) from the command string.
//------------------------------------------------
int16_t find_word(const char *cmd, char word)
{
  char ch;
  int16_t pos = -1;
  int16_t first = -1;
  int16_t last = -1;

  while ((ch = cmd[++pos]) != '\0')
  {
    if (ch == word)
    {
      if (first == -1) first = pos;
      last = pos;
    }
  }
  if (first == -1) return -1;
  if (first != last) return -1;
  if (first >= 0 && cmd[pos+1] == ' ') return -1;
  return first;
}

int parse_int(const char *cmd, char word, int32_t *value)
{
  // Parse the value.
  // TODO:  check errno, check for no whitespace
  int16_t pos = find_word(cmd, word);
  if (pos < 0)
    return 0;
  *value = strtol(&cmd[pos+1], NULL, 10);
  return 1;
}

int parse_uint(const char *cmd, char word, uint32_t *value)
{
  // Parse the value.
  // TODO:  check errno, check for no whitespace
  int16_t pos = find_word(cmd, word);
  if (pos < 0)
    return 0;
  *value = strtoul(&cmd[pos+1], NULL, 10);
  return 1;
}

int parse_hex(const char *cmd, char word, uint32_t *value)
{
  // Parse the value.
  // TODO:  check errno, check for no whitespace
  int16_t pos = find_word(cmd, word);
  if (pos < 0)
    return 0;
  *value = strtoul(&cmd[pos+1], NULL, 16);
  return 1;
}

int parse_float(const char *cmd, char word, float *value)
{
  // Parse the value.
  // TODO:  check errno, check for no whitespace
  int16_t pos = find_word(cmd, word);
  if (pos < 0)
    return 0;
  *value = strtod(&cmd[pos+1], NULL);
  return 1;
}

//------------------------------------------------
//PROCESS COMMAND
//
//  This function is responsible for validating the
//  command checksum, ensuring that the sequence
//  number is correct, and dispatching the command.
//
//  The protocol is currently fairly tolerant about
//  the order of words, but the recommended format is:
//
//      [G|M]<code> [params...] N<seqnbr>*123
//      [G|M]<code> [params...] N<seqnbr>;1A04
//
//  The '*ddd' checksum is a simple 8-bit XOR of the
//  message characters (not including the '*'), while
//  the ';xxxx' checksum is the far more robust 16-bit
//  XMODEM CRC.
//------------------------------------------------
void process_command(const char *cmdstr)
{
  // Validate the command's checksum, if provided.
  uint32_t checksum = -1;
  if (parse_hex(cmdstr, ';', &checksum)) {
    // 16-bit XMODEM cyclic redundancy check.
    uint16_t calculated_checksum = 0;
    if (checksum < 0x0000 || checksum > 0xFFFF)
    {
      serial_send("rs %ld (checksum out of range)\r\n", cmdseqnbr);
      return;
    }
    for (int i = 0; i < MAX_CMD_SIZE && cmdstr[i] != ';'; i++)
    {
      calculated_checksum = _crc_xmodem_update(calculated_checksum, cmdstr[i]);
    }
    if (calculated_checksum != (uint16_t)checksum)
    {
      serial_send("rs %ld (incorrect checksum - should be %u)\r\n", cmdseqnbr, calculated_checksum);
      return;
    }
  } else if (parse_uint(cmdstr, '*', &checksum)) {
    // 8-bit XOR checksum.
    uint8_t calculated_checksum = 0;
    if (checksum < 0 || checksum > 255)
    {
      serial_send("rs %ld (checksum out of range)\r\n", cmdseqnbr);
      return;
    }
    for (int i = 0; i < MAX_CMD_SIZE && cmdstr[i] != '*'; i++)
    {
      calculated_checksum ^= cmdstr[i];
    }
    if (calculated_checksum != (uint8_t)checksum)
    {
      serial_send("rs %ld (incorrect checksum - should be %u)\r\n", cmdseqnbr, calculated_checksum);
      return;
    }
  }

  // Validate the command's sequence number, if provided.
  uint32_t seqnbr;
  if (parse_uint(cmdstr, 'N', &seqnbr))
  {
    if (seqnbr == 0) cmdseqnbr = 0;
    if (seqnbr != cmdseqnbr) {
      serial_send("rs %ld (incorrect seqnbr)\r\n", cmdseqnbr);
      return;
    }
  }

  // Validate that the command has a single G or M code.
  int has_gcode;
  int has_mcode;
  uint32_t code = -1;
  has_gcode = parse_uint(cmdstr, 'G', &code);
  has_mcode = parse_uint(cmdstr, 'M', &code);
  if (has_gcode && has_mcode)
  {
    serial_send("rs %ld (multiple command codes)\r\n", cmdseqnbr);
    return;
  }
  if (!has_gcode && !has_mcode)
  {
    serial_send("rs %ld (command code missing)\r\n", cmdseqnbr);
    return;
  }
  if (code < 1 || code > 999)
  {
    serial_send("rs %ld (command code out of range)\r\n", cmdseqnbr);
    return;
  }

  // Create the 'struct command'.
  struct command cmd;
  cmd.seqnbr = seqnbr;
  cmd.type = has_gcode ? 'G' : 'M';
  cmd.code = code;
  cmd.has_X = parse_float(cmdstr, 'X', &cmd.X);
  cmd.has_Y = parse_float(cmdstr, 'Y', &cmd.Y);
  cmd.has_Z = parse_float(cmdstr, 'Z', &cmd.Z);
  cmd.has_E = parse_float(cmdstr, 'E', &cmd.E);
  cmd.has_F = parse_float(cmdstr, 'F', &cmd.F);
  cmd.has_I = parse_float(cmdstr, 'I', &cmd.I);
  cmd.has_J = parse_float(cmdstr, 'J', &cmd.J);
  cmd.has_P = parse_int(cmdstr, 'P', &cmd.P);
  cmd.has_S = parse_int(cmdstr, 'S', &cmd.S);
  cmd.has_T = parse_int(cmdstr, 'T', &cmd.T);

  // Dispatch command.
  unsigned long start_tm, end_tm;
  serial_send("go %ld (executing %c%d)\r\n", cmdseqnbr, cmd.type, cmd.code);
  start_tm = millis();
  switch (cmd.type) {
  case 'G':  execute_gcode(&cmd);  break;
  case 'M':  execute_mcode(&cmd);  break;
  }
  end_tm = millis();
  uint8_t qfree = blocks_available();
  serial_send("ok %ld Q%d (%lums execute)\r\n", 
    cmdseqnbr, qfree, end_tm - start_tm);
  cmdseqnbr++;
  previous_millis_cmd = end_tm;
}


FORCE_INLINE void homing_routine(unsigned char axis)
{
  int min_pin, max_pin, home_dir, max_length, home_bounce;

  switch(axis){
    case X_AXIS:
      min_pin = X_MIN_PIN;
      max_pin = X_MAX_PIN;
      home_dir = X_HOME_DIR;
      max_length = X_MAX_LENGTH;
      home_bounce = 10;
      break;
    case Y_AXIS:
      min_pin = Y_MIN_PIN;
      max_pin = Y_MAX_PIN;
      home_dir = Y_HOME_DIR;
      max_length = Y_MAX_LENGTH;
      home_bounce = 10;
      break;
    case Z_AXIS:
      min_pin = Z_MIN_PIN;
      max_pin = Z_MAX_PIN;
      home_dir = Z_HOME_DIR;
      max_length = Z_MAX_LENGTH;
      home_bounce = 4;
      break;
    default:
      //never reached
      break;
  }

  if ((min_pin > -1 && home_dir==-1) || (max_pin > -1 && home_dir==1))
  {
    current_position[axis] = -1.5 * max_length * home_dir;
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
    destination[axis] = 0;
    feedrate = homing_feedrate[axis];
    prepare_move();
    st_synchronize();

    current_position[axis] = home_bounce/2 * home_dir;
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
    destination[axis] = 0;
    prepare_move();
    st_synchronize();

    current_position[axis] = -home_bounce * home_dir;
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
    destination[axis] = 0;
    feedrate = homing_feedrate[axis]/2;
    prepare_move();
    st_synchronize();

    current_position[axis] = (home_dir == -1) ? 0 : max_length;
    current_position[axis] += add_homing[axis];
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
    destination[axis] = current_position[axis];
    feedrate = 0;
  }
}

//------------------------------------------------
// CHECK COMMAND AND CONVERT VALUES
//------------------------------------------------
void execute_gcode(struct command *cmd)
{
  unsigned long codenum; //throw away variable
  switch(cmd->code) {
      case 0: // G0 -> G1
      case 1: // G1
        #if (defined DISABLE_CHECK_DURING_ACC) || (defined DISABLE_CHECK_DURING_MOVE) || (defined DISABLE_CHECK_DURING_TRAVEL)
          manage_heater();
        #endif
        get_coordinates(cmd); // For X Y Z E F
        prepare_move();
        previous_millis_cmd = millis();
        return;
        //break;
      #ifdef USE_ARC_FUNCTION
      case 2: // G2  - CW ARC
        get_arc_coordinates(cmd);
        prepare_arc_move(true);
        previous_millis_cmd = millis();
        //break;
        return;
      case 3: // G3  - CCW ARC
        get_arc_coordinates(cmd);
        prepare_arc_move(false);
        previous_millis_cmd = millis();
        //break;
        return;  
      #endif  
      case 4: // G4 dwell
        codenum = 0;
        if(cmd->has_P) codenum += cmd->P; // milliseconds to wait
        if(cmd->has_S) codenum += cmd->S * 1000; // seconds to wait
        serial_send("-- delaying %lums\r\n", codenum);
        codenum += millis();  // keep track of when we started waiting
        st_synchronize();  // wait for all movements to finish
        while(millis()  < codenum ){
          manage_heater();
        }
        break;
      case 28: //G28 Home all Axis one at a time
        saved_feedrate = feedrate;
        saved_feedmultiply = feedmultiply;
        previous_millis_cmd = millis();
        
        feedmultiply = 100;    
      
        enable_endstops(true);
      
        for(int i=0; i < NUM_AXIS; i++) 
        {
          destination[i] = current_position[i];
        }
        feedrate = 0;
        is_homing = true;

        home_all_axis = !(cmd->has_X || cmd->has_Y || cmd->has_Z);

        if((home_all_axis) || cmd->has_X)
          homing_routine(X_AXIS);

        if((home_all_axis) || cmd->has_Y)
          homing_routine(Y_AXIS);

        if((home_all_axis) || cmd->has_Z)
          homing_routine(Z_AXIS);
        
        #ifdef ENDSTOPS_ONLY_FOR_HOMING
            enable_endstops(false);
      	#endif
      
        is_homing = false;
        feedrate = saved_feedrate;
        feedmultiply = saved_feedmultiply;
      
        previous_millis_cmd = millis();
        break;
      case 90: // G90
        relative_mode = false;
        break;
      case 91: // G91
        relative_mode = true;
        break;
      case 92: // G92
        if(!cmd->has_E)
          st_synchronize();
          
        if (cmd->has_X) current_position[X_AXIS] = cmd->X;
        if (cmd->has_Y) current_position[Y_AXIS] = cmd->Y;
        if (cmd->has_Z) current_position[Z_AXIS] = cmd->Z;
        if (cmd->has_E) current_position[E_AXIS] = cmd->E;
        plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
        break;
      default:
        serial_send("-- Unknown code G%d.\r\n", cmd->code);
        break;
  }
}


void execute_mcode(struct command *cmd) {
    unsigned long codenum; //throw away variable
    switch(cmd->code) {
      case 104: // M104
#ifdef CHAIN_OF_COMMAND
          st_synchronize(); // wait for all movements to finish
#endif
        if (cmd->has_S) target_raw = temp2analogh(target_temp = cmd->S);
        #ifdef WATCHPERIOD
            if(target_raw > current_raw)
            {
                watchmillis = MAX(1,millis());
                watch_raw = current_raw;
            }
            else
            {
                watchmillis = 0;
            }
        #endif
        break;
      case 140: // M140 set bed temp
#ifdef CHAIN_OF_COMMAND
          st_synchronize(); // wait for all movements to finish
#endif
        #if TEMP_1_PIN > -1 || defined BED_USES_AD595
            if (cmd->has_S) target_bed_raw = temp2analogBed(cmd->S);
        #endif
        break;
      case 105: // M105
        #if (TEMP_0_PIN > -1) || defined (HEATER_USES_MAX6675)|| defined HEATER_USES_AD595
          hotendtC = analog2temp(current_raw);
        #endif
        #if TEMP_1_PIN > -1 || defined BED_USES_AD595
          bedtempC = analog2tempBed(current_bed_raw);
        #endif
        #if (TEMP_0_PIN > -1) || defined (HEATER_USES_MAX6675) || defined HEATER_USES_AD595
          serial_send("ok T%d", hotendtC);
          #ifdef PIDTEMP
            serial_send(" D%d", heater_duty);
            /*
            serial_send(",P:%d", pTerm);
            serial_send(",I:%d", iTerm);
            serial_send(",D:%d", dTerm);
            */
            #ifdef AUTOTEMP
              serial_send(" A%d", autotemp_setpoint);
            #endif
          #endif
          #if TEMP_1_PIN > -1 || defined BED_USES_AD595
            serial_send(" B%d", bedtempC);
          #endif
          serial_send("\r\n");
        #else
          #error No temperature source available
        #endif
        return;
        //break;
      case 109: { // M109 - Wait for extruder heater to reach target.
#ifdef CHAIN_OF_COMMAND
          st_synchronize(); // wait for all movements to finish
#endif
        if (cmd->has_S) target_raw = temp2analogh(target_temp = cmd->S);
        #ifdef WATCHPERIOD
            if(target_raw>current_raw)
            {
                watchmillis = MAX(1,millis());
                watch_raw = current_raw;
            }
            else
            {
                watchmillis = 0;
            }
        #endif
        codenum = millis(); 
        
        /* See if we are heating up or cooling down */
        bool target_direction = (current_raw < target_raw);  // true if heating, false if cooling
        
      #ifdef TEMP_RESIDENCY_TIME
        long residencyStart;
        residencyStart = -1;
        /* continue to loop until we have reached the target temp   
           _and_ until TEMP_RESIDENCY_TIME hasn't passed since we reached it */
        while( (target_direction ? (current_raw < target_raw) : (current_raw > target_raw))
            || (residencyStart > -1 && (millis() - residencyStart) < TEMP_RESIDENCY_TIME*1000) ) {
      #else
        while ( target_direction ? (current_raw < target_raw) : (current_raw > target_raw) ) {
      #endif
          if( (millis() - codenum) > 1000 ) //Print Temp Reading every 1 second while heating up/cooling down
          {
            serial_send("T%d\r\n", analog2temp(current_raw));
            codenum = millis();
          }
          manage_heater();
          #if (MINIMUM_FAN_START_SPEED > 0)
            manage_fan_start_speed();
          #endif
          #ifdef TEMP_RESIDENCY_TIME
            /* start/restart the TEMP_RESIDENCY_TIME timer whenever we reach target temp for the first time
               or when current temp falls outside the hysteresis after target temp was reached */
            if (   (residencyStart == -1 &&  target_direction && current_raw >= target_raw)
                || (residencyStart == -1 && !target_direction && current_raw <= target_raw)
                || (residencyStart > -1 && labs(analog2temp(current_raw) - analog2temp(target_raw)) > TEMP_HYSTERESIS) ) {
              residencyStart = millis();
            }
          #endif
	    }
      }
      break;
      case 190: // M190 - Wait for bed heater to reach target temperature.
#ifdef CHAIN_OF_COMMAND
          st_synchronize(); // wait for all movements to finish
#endif
      #if TEMP_1_PIN > -1
        if (cmd->has_S) target_bed_raw = temp2analogBed(cmd->S);
        codenum = millis(); 
        while(current_bed_raw < target_bed_raw) 
        {
          if( (millis()-codenum) > 1000 ) //Print Temp Reading every 1 second while heating up.
          {
            hotendtC=analog2temp(current_raw);
            serial_send("T%d B%d\r\n", hotendtC, analog2tempBed(current_bed_raw));
            codenum = millis(); 
          }
          manage_heater();
          #if (MINIMUM_FAN_START_SPEED > 0)
            manage_fan_start_speed();
          #endif
        }
      #endif
      break;
      #if FAN_PIN > -1
      case 106: //M106 Fan On
#ifdef CHAIN_OF_COMMAND
          st_synchronize(); // wait for all movements to finish
#endif
        if (cmd->has_S)
        {
            unsigned char l_fan_code_val = CONSTRAIN(cmd->S,0,255);
            
            #if (MINIMUM_FAN_START_SPEED > 0)
              if(l_fan_code_val > 0 && fan_last_speed == 0)
              {
                 if(l_fan_code_val < MINIMUM_FAN_START_SPEED)
                 {
                   fan_org_start_speed = l_fan_code_val;
                   l_fan_code_val = MINIMUM_FAN_START_SPEED;
                   previous_millis_fan_start = millis();
                 }
                 fan_last_speed = l_fan_code_val;  
              }  
              else
              {
                fan_last_speed = l_fan_code_val;
                fan_org_start_speed = 0;
              }  
            #endif
          
            #if defined(FAN_SOFT_PWM) && (FAN_PIN > -1)
              g_fan_pwm_val = l_fan_code_val;
            #else
              WRITE(FAN_PIN, HIGH);
              analogWrite_check(FAN_PIN, l_fan_code_val;
            #endif
            
        }
        else 
        {
            #if defined(FAN_SOFT_PWM) && (FAN_PIN > -1)
              g_fan_pwm_val = 255;
            #else
              WRITE(FAN_PIN, HIGH);
              analogWrite_check(FAN_PIN, 255 );
            #endif
        }
        break;
      case 107: //M107 Fan Off
          #if defined(FAN_SOFT_PWM) && (FAN_PIN > -1)
            g_fan_pwm_val = 0;
          #else
            analogWrite_check(FAN_PIN, 0);
            WRITE(FAN_PIN, LOW);
          #endif
        break;
      #endif
      #if (PS_ON_PIN > -1)
      case 80: // M81 - ATX Power On
        SET_OUTPUT(PS_ON_PIN); //GND
        break;
      case 81: // M81 - ATX Power Off
#ifdef CHAIN_OF_COMMAND
          st_synchronize(); // wait for all movements to finish
#endif
        SET_INPUT(PS_ON_PIN); //Floating
        break;
      #endif
      case 82:
        axis_relative_modes[3] = false;
        break;
      case 83:
        axis_relative_modes[3] = true;
        break;
      case 84:
        st_synchronize(); // wait for all movements to finish
        if(cmd->has_S)
        {
          stepper_inactive_time = cmd->S * 1000; 
        }
        else if(cmd->has_T)
        {
          enable_x(); 
          enable_y(); 
          enable_z(); 
          enable_e(); 
        }
        else
        { 
          disable_x(); 
          disable_y(); 
          disable_z(); 
          disable_e(); 
        }
        break;
      case 85: // M85
        if (cmd->has_S)
	{
          max_inactive_time = cmd->S * 1000; 
        } else 
        {
          serial_send("!! S param required.\r\n");
        }
        break;
      case 92: // M92
        execute_m92(cmd);
        break;
      case 93: // M93 show current axis steps.
        serial_send("ok X%f Y%f Z%f E%f\r\n",
          axis_steps_per_unit[0],
          axis_steps_per_unit[1],
          axis_steps_per_unit[2],
          axis_steps_per_unit[3]
        );
        break;
      case 115: // M115
        serial_send("FIRMWARE_NAME: Makibox PROTOCOL_VERSION:1.0 MACHINE_TYPE:Mendel EXTRUDER_COUNT:1\r\n");
        serial_send("00000000-0000-0000-0000-000000000000\r\n");
        break;
      case 114: // M114
        serial_send("ok X%f Y%f Z%f E%f\r\n",
          current_position[0],
          current_position[1],
          current_position[2],
          current_position[3]
        );
        break;
      case 119: // M119
        serial_send("// ");
      	#if (X_MIN_PIN > -1)
      	  serial_send("x_min:%s", (READ(X_MIN_PIN)^X_ENDSTOP_INVERT)?"H ":"L ");
      	#endif
      	#if (X_MAX_PIN > -1)
      	  serial_send("x_max:%s", (READ(X_MAX_PIN)^X_ENDSTOP_INVERT)?"H ":"L ");
      	#endif
      	#if (Y_MIN_PIN > -1)
      	  serial_send("y_min:%s", (READ(Y_MIN_PIN)^Y_ENDSTOP_INVERT)?"H ":"L ");
      	#endif
      	#if (Y_MAX_PIN > -1)
      	  serial_send("y_max:%s", (READ(Y_MAX_PIN)^Y_ENDSTOP_INVERT)?"H ":"L ");
      	#endif
      	#if (Z_MIN_PIN > -1)
      	  serial_send("z_min:%s", (READ(Z_MIN_PIN)^Z_ENDSTOP_INVERT)?"H ":"L ");
      	#endif
      	#if (Z_MAX_PIN > -1)
      	  serial_send("z_max:%s", (READ(Z_MAX_PIN)^Z_ENDSTOP_INVERT)?"H ":"L ");
      	#endif
        serial_send("\r\n"); 
      	break;
      case 201: // M201  Set maximum acceleration in units/s^2 for print moves (M201 X1000 Y1000)
        execute_m201(cmd);
        break;
      case 202: // M202 max feedrate mm/sec
        if (cmd->has_X)
          max_feedrate[X_AXIS] = cmd->X;
        if (cmd->has_Y)
          max_feedrate[Y_AXIS] = cmd->Y;
        if (cmd->has_Z)
          max_feedrate[Z_AXIS] = cmd->Z;
        if (cmd->has_E)
          max_feedrate[E_AXIS] = cmd->E;
      break;
      case 203: // M203 Temperature monitor
          if (cmd->has_S)
            manage_monitor = cmd->S;
          if(manage_monitor==100) manage_monitor=1; // Set 100 to heated bed
      break;
      case 204: // M204 acceleration S normal moves T filmanent only moves
          if (cmd->has_S)
            move_acceleration = cmd->S;
          if (cmd->has_T)
            retract_acceleration = cmd->T;
      break;
      case 205: //M205 advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk, E= max E jerk
        if (cmd->has_S) minimumfeedrate = cmd->S;
        if (cmd->has_T) mintravelfeedrate = cmd->T;
      //if (cmd->has_B) minsegmenttime = cmd->B;
        if (cmd->has_X) max_xy_jerk = cmd->X;
        if (cmd->has_Z) max_z_jerk = cmd->Z;
        if (cmd->has_E) max_e_jerk = cmd->E;
      break;
      case 206: // M206 additional homing offset
        serial_send("// M206 Addhome X:%f Y:%f Z:%f\r\n", add_homing[0], add_homing[1], add_homing[2]);
        if (cmd->has_X) add_homing[X_AXIS] = cmd->X;
        if (cmd->has_Y) add_homing[Y_AXIS] = cmd->Y;
        if (cmd->has_Z) add_homing[Z_AXIS] = cmd->Z;
      break;  
      case 220: // M220 S<factor in percent>- set speed factor override percentage
      {
        if(cmd->has_S) 
        {
          feedmultiply = CONSTRAIN(cmd->S, 20, 200);
          feedmultiplychanged=true;
        }
      }
      break;
      case 221: // M221 S<factor in percent>- set extrude factor override percentage
      {
        if(cmd->has_S) 
          extrudemultiply = CONSTRAIN(cmd->S, 40, 200);
      }
      break;
#ifdef PIDTEMP
      case 301: // M301
      {
        //if(code_seen('P')) PID_Kp = code_value();
        //if(code_seen('I')) PID_Ki = code_value();
        //if(code_seen('D')) PID_Kd = code_value();
        //updatePID();
      }
      break;
#endif //PIDTEMP      
#ifdef PID_AUTOTUNE
      case 303: // M303 PID autotune
      {
        float help_temp = 150.0;
        if (cmd->has_S) help_temp=cmd->S;
        PID_autotune(help_temp);
      }
      break;
#endif
      case 400: // M400 finish all moves
      {
      	st_synchronize();	
      }
      break;
#ifdef USE_EEPROM_SETTINGS
      case 500: // Store settings in EEPROM
      {
        EEPROM_StoreSettings();
      }
      break;
      case 501: // Read settings from EEPROM
      {
        EEPROM_RetrieveSettings(false,true);
        for(int8_t i=0; i < NUM_AXIS; i++)
        {
          axis_steps_per_sqr_second[i] = max_acceleration_units_per_sq_second[i] * axis_steps_per_unit[i];
        }
      }
      break;
      case 502: // Revert to default settings
      {
        EEPROM_RetrieveSettings(true,true);
        for(int8_t i=0; i < NUM_AXIS; i++)
        {
          axis_steps_per_sqr_second[i] = max_acceleration_units_per_sq_second[i] * axis_steps_per_unit[i];
        }
      }
      break;
      case 503: // print settings currently in memory
      {
        EEPROM_printSettings();
      }
      break;  
#endif      
      case 603: // M603  Free RAM
            serial_send("// Free Ram: %d\r\n", FreeRam1());
      break;
      default:
            serial_send("-- Unknown code M%d.\r\n", cmd->code);
      break;

    }
}

void update_axis_steps(int axis, float steps_per_unit)
{
  axis_steps_per_unit[axis] = steps_per_unit;
  axis_steps_per_sqr_second[axis] = 
    max_acceleration_units_per_sq_second[axis] * steps_per_unit;
}

void update_max_acceleration(int axis, float units_per_sq_second)
{
  max_acceleration_units_per_sq_second[axis] = units_per_sq_second;
  axis_steps_per_sqr_second[axis] = 
    units_per_sq_second * axis_steps_per_unit[axis];
}

void execute_m92(struct command *cmd)
{
  if (cmd->has_X)
    update_axis_steps(X_AXIS, cmd->X);
  if (cmd->has_Y)
    update_axis_steps(Y_AXIS, cmd->Y);
  if (cmd->has_Z)
    update_axis_steps(Z_AXIS, cmd->Z);
  if (cmd->has_E)
    update_axis_steps(E_AXIS, cmd->E);
}

void execute_m201(struct command *cmd)
{
  if (cmd->has_X)
    update_max_acceleration(X_AXIS, cmd->X);
  if (cmd->has_Y)
    update_max_acceleration(Y_AXIS, cmd->Y);
  if (cmd->has_Z)
    update_max_acceleration(Z_AXIS, cmd->Z);
  if (cmd->has_E)
    update_max_acceleration(E_AXIS, cmd->E);
}

void update_axis_pos(int axis, float pos)
{
  destination[axis] = pos + (axis_relative_modes[axis] || relative_mode)*current_position[axis];
}

void get_coordinates(struct command *cmd)
{
  if (cmd->has_X)
    update_axis_pos(X_AXIS, cmd->has_X ? cmd->X : current_position[X_AXIS]);
  if (cmd->has_Y)
    update_axis_pos(Y_AXIS, cmd->has_Y ? cmd->Y : current_position[Y_AXIS]);
  if (cmd->has_Z)
    update_axis_pos(Z_AXIS, cmd->has_Z ? cmd->Z : current_position[Z_AXIS]);
  if (cmd->has_F && cmd->F > 0.0)
    feedrate = cmd->F;
}

#ifdef USE_ARC_FUNCTION
void get_arc_coordinates(struct command *cmd)
{
   get_coordinates(cmd);
   offset[0] = cmd->has_I ? cmd->I : 0.0;
   offset[1] = cmd->has_J ? cmd->J : 0.0;
}
#endif



void prepare_move()
{
  long help_feedrate = 0;

  if(!is_homing){
    #if MIN_SOFTWARE_ENDSTOPS
    if (destination[X_AXIS] < 0) destination[X_AXIS] = 0.0;
    if (destination[Y_AXIS] < 0) destination[Y_AXIS] = 0.0;
    if (destination[Z_AXIS] < 0) destination[Z_AXIS] = 0.0;
    #endif

    #if MAX_SOFTWARE_ENDSTOPS
    if (destination[X_AXIS] > X_MAX_LENGTH) destination[X_AXIS] = X_MAX_LENGTH;
    if (destination[Y_AXIS] > Y_MAX_LENGTH) destination[Y_AXIS] = Y_MAX_LENGTH;
    if (destination[Z_AXIS] > Z_MAX_LENGTH) destination[Z_AXIS] = Z_MAX_LENGTH;
    #endif
  }

  if(destination[E_AXIS] > current_position[E_AXIS])
  {
    help_feedrate = ((long)feedrate*(long)feedmultiply);
  }
  else
  {
    help_feedrate = ((long)feedrate*(long)100);
  }
  
  plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], help_feedrate/6000.0);
  
  for(int i=0; i < NUM_AXIS; i++)
  {
    current_position[i] = destination[i];
  } 
}


#ifdef USE_ARC_FUNCTION
void prepare_arc_move(char isclockwise) 
{

  float r = hypot(offset[X_AXIS], offset[Y_AXIS]); // Compute arc radius for mc_arc
  long help_feedrate = 0;

  if(destination[E_AXIS] > current_position[E_AXIS])
  {
    help_feedrate = ((long)feedrate*(long)feedmultiply);
  }
  else
  {
    help_feedrate = ((long)feedrate*(long)100);
  }

  // Trace the arc
  mc_arc(current_position, destination, offset, X_AXIS, Y_AXIS, Z_AXIS, help_feedrate/6000.0, r, isclockwise);
  
  // As far as the parser is concerned, the position is now == target. In reality the
  // motion control system might still be processing the action and the real tool position
  // in any intermediate location.
  for(int8_t i=0; i < NUM_AXIS; i++) 
  {
    current_position[i] = destination[i];
  }
}
#endif

FORCE_INLINE void kill()
{
  #if TEMP_0_PIN > -1
    target_raw=0;
    WRITE(HEATER_0_PIN,LOW);
  #endif
  
  #if TEMP_1_PIN > -1
    target_bed_raw=0;
    if(HEATER_1_PIN > -1) WRITE(HEATER_1_PIN,LOW);
  #endif

  disable_x();
  disable_y();
  disable_z();
  disable_e();
}


void manage_inactivity(unsigned char debug) 
{ 
  manage_heater();
  if( (millis()-previous_millis_cmd) >  max_inactive_time ) if(max_inactive_time) kill(); 
  
  if( (millis()-previous_millis_cmd) >  stepper_inactive_time ) if(stepper_inactive_time) 
  { 
    disable_x(); 
    disable_y(); 
    disable_z(); 
    disable_e(); 
  }
  check_axes_activity();
}

#if (MINIMUM_FAN_START_SPEED > 0)
void manage_fan_start_speed(void)
{
  if(fan_org_start_speed > 0)
  {
     if((millis() - previous_millis_fan_start) > MINIMUM_FAN_START_TIME )
     { 
       #if FAN_PIN > -1
         #if defined(FAN_SOFT_PWM)
           g_fan_pwm_val = fan_org_start_speed;
         #else
           WRITE(FAN_PIN, HIGH);
           analogWrite_check(FAN_PIN, fan_org_start_speed;
         #endif  
       #endif
       
       fan_org_start_speed = 0;
     }  
  }
}
#endif

// Block until all buffered steps are executed
void st_synchronize()
{
  while(blocks_queued()) {
    manage_inactivity(1);
    #if (MINIMUM_FAN_START_SPEED > 0)
      manage_fan_start_speed();
    #endif
  }
}

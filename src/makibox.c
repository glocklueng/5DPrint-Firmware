/*
  Makibox A6 Firmware
  Based on Sprinter (master branch, 1 Sep 2012).
  Designed for Printrboard (Rev B).
  ---
  Copyright (c) 2012-2013 by Makible Limited.
 
  This file is part of the Makibox A6 Firmware.
 
  Makibox A6 Firmware is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  The Makibox A6 Firmware is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with the Makibox A6 Firmware.  If not, see <http://www.gnu.org/licenses/>.
*/
/**
   \file makibox.c
   \brief Handles all the commands, setup routine and main loop
*/

// look here for descriptions of gcodes: http://linuxcnc.org/handbook/gcode/g-code.html
//   http://objects.reprap.org/wiki/Mendel_User_Manual:_RepRapGCodes

/**
   \fn void execute_gcode(struct command *cmd)
   \brief Executes the gcode command
   
   - G0  -> G1
   - G1  - Coordinated Movement X Y Z E
   - G2  - CW ARC
   - G3  - CCW ARC
   - G4  - Dwell S<seconds> or P<milliseconds>
   - G28 - Home all Axis
   - G90 - Use Absolute Coordinates
   - G91 - Use Relative Coordinates
   - G92 - Set current position to cordinates given
*/
/**
   \fn void execute_mcode(struct command *cmd)
   \brief Executes the mcode command

   RepRap M Codes
   - M104 - Set extruder target temp
   - M140 - Set bed target temp
   - M105 - Read current temp
   - M106 - Fan on
   - M107 - Fan off
   - M109 - Wait for extruder current temp to reach target temp.
   - M190 - Wait for bed current temp to reach target temp.
   - M112 - Emergency Stop
   - M114 - Display current position

   Custom M Codes
   - M18	- Disable steppers until next move. Same as M84.
   - M20  - List SD card - F value passed via the F are the flags for displaying 
   directories and file sizes.	Bit 0x01 = display file size;
   Bit 0x02 = display directories;
   - M21  - Init SD card
   - M22  - Release SD card
   - M23  - Select SD file (M23 filename.g)
   - M24  - Start/resume SD print
   - M25  - Pause SD print
   - M27  - Report SD print status
   - M28  - Start SD write (M28 filename.g)
   - M29  - Stop SD write- 	
   - M30  - Delete file from SD (M30 filename.g)
   - M31  - Enable (M31 E1) / Disable (M31 E0) autoprint from SD card
   - M80  - Turn on Power Supply
   - M81  - Turn off Power Supply
   - M82  - Set E codes absolute (default)
   - M83  - Set E codes relative while in Absolute Coordinates (G90) mode
   - M84  - Disable steppers until next move,-  
   or use S<seconds> to specify an inactivity timeout, after which the steppers will be disabled.  S0 to disable the timeout.
   - M85  - Use S<seconds> to specify an inactivity timeout, after which the steppers and heaters will be disabled.  S0 to disable the timeout.
   - M92  - Set axis_steps_per_unit - same syntax as G92
   - M93  - Show axis_steps_per_unit
   - M115	- Capabilities string
   - M119 - Show Endstopper State 
   - M201 - Set maximum acceleration in units/s^2 for print moves (M201 X1000 Y1000)
   - M202 - Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in mm/sec
   - M203 - Set temperture monitor to Sx; Px sets regular temperature reporting to every x seconds.
   - M204 - Set default acceleration: S normal moves T filament only moves (M204 S3000 T7000) in mm/sec^2
   - M205 - advanced settings:  minimum travel speed S=while printing T=travel only,  X=maximum xy jerk, Z=maximum Z jerk
   - M206 - set additional homing offset

   - M220 - set speed factor override percentage S=factor in percent 
   - M221 - set extruder multiply factor S100 --> original Extrude Speed 

   - M226 - M226 / M226 P1 = Pause print  <br>
   M226 P0 = resume print <br>
   M226 P-255 = discard plan buffer contents and resume normal operation

   - M300 - Starts the buzzer with freqeuncy f and period p
   - M301 - Set PID parameters P I and D
   - M302 - Enable / Disable Cold Extrudes P1 = allow cold extrudes; P0 = Do not allow cold extrudes
   - M303 - PID relay autotune S<temperature> sets the target temperature. (default target temperature = 150C)
   - M305 - set hot bed max duty cycle. M305 S<before or after 70C> D<duty cycle> <br>
   before 70C: 0, after 70C: 1 <br>
   duty cycle is from 0% to 100%   
   - M400 - Finish all moves

   - M500 - stores paramters in EEPROM
   - M501 - reads parameters from EEPROM (if you need to reset them after you changed them temporarily).
   - M502 - reverts to the default "factory settings".
   - M503 - Print settings

   Debug feature
   - M603 - Show Free Ram
   - M604 - Show Timer 1 COMPA ISR Execution Time Debug Info
   - M605 - Reset Timer 1 COMPA ISR Execution Time Min / Max Values
   - M606 - Show CPU loading information
   - M607 - Reset Peak and Average CPU load values
   - M608 - Show Firmware Version Info
   - M609 - Show last reset flags
   - M610 - Set Extruder Heater Max Current P = 0 - 100%.

   - M852 - Enter Boot Loader Command (Requires correct F pass code)
   - M906 - Set current limits for stepper motors e.g. M906 X1700 Y1700 Z1700 E1700 S100
   S is the current sense resistance
   - M907 - Set microstep settings for stepper motors. e.g. M906 X16 Y16 Z16 E16
*/

#include <avr/interrupt.h>
#include "pgmspace.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <util/crc16.h>

#include "config.h"
#include "board_io.h"
#include "pins.h"
#include "pins_teensy.h"
#include "makibox.h"
#include "heater.h"
#include "usb.h"
#include "command.h"
#include "planner.h"
#include "stepper.h"
#include "sdcard/makibox_sdcard.h"
#include "language.h"
#include "tone.h"
#include "autoprint.h"

#if DIGIPOTS > 0
#include "i2c/Master_I2C_Comms.h"
#endif


#ifdef USE_ARC_FUNCTION
#include "arc_func.h"
#endif

#ifdef USE_EEPROM_SETTINGS
#include "store_eeprom.h"
#endif


#define  FORCE_INLINE __attribute__((always_inline)) inline

#define BOOTLOADER_PASSCODE		3464

// Minimum change in target temperature for 'WATCHPERIOD' to be activated
// for hotend heater.
#define MIN_TARGET_TEMP_CHANGE	10

//void cmdbuf_read_serial();
//void cmdbuf_process();
void execute_command();
void prepare_move();
void prepare_arc_move(char isclockwise);
#ifdef USE_ARC_FUNCTION
void get_arc_coordinates();
#endif
void kill();
void JumpToBootloader(void) __attribute__((noreturn));

#if (MINIMUM_FAN_START_SPEED > 0)
void manage_fan_start_speed(void);
#endif

void read_command();
void process_command(const char *cmdstr);
void execute_gcode(struct command *cmd);
void execute_mcode(struct command *cmd);
void get_coordinates(struct command *cmd);
void get_arc_coordinates(struct command *cmd);
void execute_m92(struct command *cmd);
void execute_m201(struct command *cmd);
void execute_m226(struct command *cmd);

void do_position_report(void);
void wait_extruder_target_temp(void);
void wait_bed_target_temp(void);
void set_extruder_heater_max_current(struct command *cmd);

#if DIGIPOTS > 0
void execute_m906(struct command *cmd);
#endif
#if SET_MICROSTEP > 0
void execute_m907(struct command *cmd);
#endif

#ifndef CRITICAL_SECTION_START
#define CRITICAL_SECTION_START  unsigned char _sreg = SREG; cli()
#define CRITICAL_SECTION_END    SREG = _sreg
#endif //CRITICAL_SECTION_START

static const char VERSION_TEXT[] = "2.20.23 / 20.2.2014";

#ifdef PIDTEMP
unsigned int PID_Kp = PID_PGAIN, PID_Ki = PID_IGAIN, PID_Kd = PID_DGAIN;
#endif

#ifdef BED_PIDTEMP
unsigned int bed_PID_Kp = BED_PID_PGAIN, bed_PID_Ki = BED_PID_IGAIN, bed_PID_Kd = BED_PID_DGAIN;
#endif

// X, Y, Z and E max acceleration in mm/s^2 for printing moves or retracts
long  max_acceleration_units_per_sq_second[4] = _MAX_ACCELERATION_UNITS_PER_SQ_SECOND;

//adjustable feed factor for online tuning printer speed
volatile int feedmultiply=100; //100->original / 200 -> Factor 2 / 50 -> Factor 0.5
volatile int saved_feedmultiply=100;
volatile int feedmultiplychanged=0;

float destination[NUM_AXIS] = {0.0, 0.0, 0.0, 0.0};
float current_position[NUM_AXIS] = {0.0, 0.0, 0.0, 0.0};
float add_homing[3]={0,0,0};

int home_all_axis = 1;
uint8_t x_homed, y_homed, z_homed = 0;

//unsigned ?? ToDo: Check
int feedrate = 1500, next_feedrate, saved_feedrate;

int relative_mode = 0;  //Determines Absolute or Relative Coordinates

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
static char cmdbuf[MAX_CMD_SIZE + 1];
//unsigned char bufpos = 0;
uint32_t cmdseqnbr = 0;
// Create the 'struct command'.
struct command cmd;
int32_t code = -1;
static unsigned char ignore_comments = 0;
static unsigned char bufpos = 0;

//Send Temperature in °C to Host
int hotendtC = 0, bedtempC = 0;
       
//Inactivity shutdown variables
unsigned long previous_millis_cmd = 0;
unsigned long previous_millis_g_cmd = 0;
unsigned long max_inactive_time = INACTIVITY_HEATERS_TIMEOUT;
unsigned long stepper_inactive_time = INACTIVITY_STEPPERS_TIMEOUT;

uint8_t print_paused = 0;

//Temp Monitor for repetier
unsigned char manage_monitor = 255;

#if SDSUPPORT > 0
#define SDCARD_WRITEBUF_SIZE			2048
// SD Card Variables
struct fat_fs_struct* sdcard_fs = 0;
char sdard_filename[92];
unsigned char sdcard_print = 0;
static unsigned char sdcard_print_pause = 0;
static char sdcard_cmdbuf[MAX_CMD_SIZE + 1];
static unsigned char sdcard_bufpos = 0;
static unsigned char sdcard_ignore_comments = 0;
static unsigned char sdcard_write = 0;
static char sdcard_writebuf[SDCARD_WRITEBUF_SIZE];
static unsigned short sdcard_writebuf_pos = 0;
#endif

#define MIN(a,b) ((a)<(b)?(a):(b))                                               
#define MAX(a,b) ((a)>(b)?(a):(b))                                               
#define CONSTRAIN(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt))) 

int FreeRam1(void)
{
    extern int  __bss_end;
    extern int* __brkval;
    int free_memory;

    if ((int)(__brkval) == 0)
        {
            // if no heap use from end of bss section
            free_memory = (int)(&free_memory) - (int)(&__bss_end);
        }
    else
        {
            // use from top of stack to heap
            free_memory = (int)(&free_memory) - (int)(__brkval);
        }
  
    return free_memory;
}


//------------------------------------------------
// Init 
//------------------------------------------------
void setup()
{ 
    // Disable JTAG
    // Has to be called twice
    MCUCR |= (1 << JTD);
    MCUCR |= (1 << JTD);

    usb_serial_begin(); 
    serial_send(TXT_MAKIBOX_VER_STARTED_CRLF, VERSION_TEXT);
  
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
  
#if SET_MICROSTEP > 0
    //Initialize Microstep Pins - steppers default to 16 Step
#if (X_MS1_PIN > -1)
    SET_OUTPUT(X_MS1_PIN);
    WRITE(X_MS1_PIN, HIGH);
#endif
#if (X_MS2_PIN > -1)
    SET_OUTPUT(X_MS2_PIN);
    WRITE(X_MS2_PIN, HIGH);
#endif
#if (Y_MS1_PIN > -1)
    SET_OUTPUT(Y_MS1_PIN);
    WRITE(Y_MS1_PIN, HIGH);
#endif
#if (Y_MS2_PIN > -1)
    SET_OUTPUT(Y_MS2_PIN);
    WRITE(Y_MS2_PIN, HIGH);
#endif
#if (Z_MS1_PIN > -1)
    SET_OUTPUT(Z_MS1_PIN);
    WRITE(Z_MS1_PIN, HIGH);
#endif
#if (Z_MS2_PIN > -1)
    SET_OUTPUT(Z_MS2_PIN);
    WRITE(Z_MS2_PIN, HIGH);
#endif
#if (E_MS1_PIN > -1)
    SET_OUTPUT(E_MS1_PIN);
    WRITE(E_MS1_PIN, HIGH);
#endif
#if (E_MS2_PIN > -1)
    SET_OUTPUT(E_MS2_PIN);
    WRITE(E_MS2_PIN, HIGH);
#endif
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

#if SDSUPPORT > 0
    //Setup SD Card / SPI Pins
    DDRB |= (1 << PINB2);		// MOSI Pin
    DDRB |= (1 << PINB1);		// SCK Pin
    DDRB |= (1 << PINB0);		// CS Pin set as output
    DDRB &= ~(1 << PINB3);	// MISO

    PORTB |= (1 << PINB0);	// Unselect Card (CS pin high)
  
    DDRB |= (1 << PINB0);		// SS Pins set as output
    PORTB |= (1 << PINB0);	// SS Pin set high

#ifdef PRINTRBOARD_REVB
    DDRB &= ~(1 << DDB7);         // Configure card detect pin
    PORTB |= (1 << PINB7);        // Enable pull-up resistor
#endif
#ifdef MAKIBOX_5DPD8
    DDRE &= ~(1 << DDE3);         // Configure card detect pin
    PORTE |= (1 << PINE3);        // Enable pull-up resistor
#endif   

#endif
  
    // Initialise Timer 3 / PWM for Extruder Heater, Hotbed Heater, and Fan
    init_Timer3_HW_pwm();
  
    serial_send(TXT_PLANNER_INIT);
    plan_init();  // Initialize planner;

    serial_send(TXT_STEPPER_TIMER_INIT_CRLF);
    st_init();    // Initialize stepper

#ifdef USE_EEPROM_SETTINGS
    //first Value --> Init with default
    //second value --> Print settings to UART
    EEPROM_RetrieveSettings(0, 0);
#endif
  
#if DIGIPOTS > 0
    // SET_OUTPUT(DIGIPOT_RESET);
    // Ensure the digi-pot is in a known state by resetting it.
    // WRITE(DIGIPOT_RESET, LOW);
    // delay(1);
    // WRITE(DIGIPOT_RESET, HIGH);
    
    init_I2C_Master();
    delay(1);
    I2C_digipots_set_defaults();
#endif

#if BUZZER_SUPPORT > 0
    buzzer_init();
#endif

#ifdef PIDTEMP
    updatePID();
#endif
  
    //Free Ram
    serial_send(TXT_FREE_RAM_CRLF, FreeRam1());
  
    for(int8_t i=0; i < NUM_AXIS; i++)
        {
            axis_steps_per_sqr_second[i] = max_acceleration_units_per_sq_second[i] * axis_steps_per_unit[i];
        }

    // ensure all our init stuff gets sent
    usb_serial_flush();
}



//------------------------------------------------
//MAIN LOOP
//------------------------------------------------
void loop()
{

#if AUTOPRINT > 0
    if (autoprint_enabled) autoprint();
#endif

#if DIGIPOTS > 0
    Service_I2C_Master();
#endif

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
    int16_t ch = -1;
#if SDSUPPORT > 0
    uint8_t sd_ch[32];
    int16_t bytes_read = -1;
    uint8_t i;
#endif

    while (usb_serial_available() > 0){
#if (DEBUG > -1)
        PreemptionFlag |= 0x0002;
#endif
        
        ch = usb_serial_read();	
        if (ch == ';') ignore_comments = 1;	
        if ( !(ch < 0 || ch > 255) ){	
            if ((ch == '\n' || ch == '\r') || 
                (ch == ',' && !ignore_comments)){
                // Newline or comma marks end of this command;  
                // terminate string and process it.
                // Comma is not effective in a comment
                cmdbuf[bufpos] = '\0';
                if (bufpos > 0) process_command(cmdbuf);
                ignore_comments = 0;
                bufpos = 0;
                cmdbuf[bufpos] = '\0';
                // Flush any output which may not have been sent 
                // at the end of command execution.
                usb_serial_flush();
                break;
            }
                
            if (ignore_comments < 1){
                cmdbuf[bufpos++] = (uint8_t)ch;
                if (bufpos > MAX_CMD_SIZE - 1){
                    // TODO:  can we do something more intelligent than
                    // just silently truncating the command?
                    bufpos--;
                }
            }
        }
    }
    
#if SDSUPPORT > 0
    // Printing from SD Card file
    if ( (sdcard_print) && (!sdcard_print_pause) )
        {
            bytes_read = sdcard_file_read(sdcard_fd, sd_ch, sizeof(sd_ch));
            //serial_send("-- Bytes Read: %d\r\n", bytes_read);
            //serial_send("-- Chars: %s\r\n", (char *)sd_ch);
	
            if (bytes_read == 0)
                {	// End of File -> Close file
                    sdcard_closeFile(sdcard_fd);
                    sdcard_fd = 0;
                    sdcard_print = 0;
                    serial_send(TXT_CLOSED_SD_CARD_FILE_CRLF);
                    serial_send(TXT_DONE_PRINTING_FILE_CRLF);
                }
            else if (bytes_read < 0)
                {
                    serial_send(TXT_ERROR_READING_FILE_CRLF);
                    // Close file
                    sdcard_closeFile(sdcard_fd);
                    sdcard_fd = 0;
                    sdcard_print = 0;
                    serial_send(TXT_DONE_PRINTING_FILE_CRLF);
                }
	
            for (i = 0; i < bytes_read; i++)
                {
                    if ((char)sd_ch[i] == ';')
                        {
                            sdcard_ignore_comments = 1;
                        }
		
                    if ( !((char)sd_ch[i] < 0 || (char)sd_ch[i] > 255) )
                        {	
                            if ((char)sd_ch[i] == '\n' || (char)sd_ch[i] == '\r')
                                {
                                    // Newline marks end of this command;  terminate
                                    // string and process it.
                                    sdcard_cmdbuf[sdcard_bufpos] = '\0';
                                    if (sdcard_bufpos > 0)
                                        {
                                            process_command(sdcard_cmdbuf);
                                        }
                                    sdcard_ignore_comments = 0;
                                    sdcard_bufpos = 0;
                                    sdcard_cmdbuf[sdcard_bufpos] = '\0';
                                    // Flush any output which may not have been sent 
                                    // at the end of command execution.
                                    usb_serial_flush();
                                    //return;
                                }
                            else
                                {
                                    if (sdcard_ignore_comments < 1)
                                        {
                                            sdcard_cmdbuf[sdcard_bufpos++] = (char)sd_ch[i];
                                        }
				
                                    if (sdcard_bufpos > MAX_CMD_SIZE - 1)
                                        {
                                            // TODO:  can we do something more intelligent than
                                            // just silently truncating the command?
                                            sdcard_bufpos--;
                                        }
                                }
                        }
                } // for
        } // if (sdcard_print)
#endif
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

    while ((ch = cmd[++pos]) != '\0')
        {
            if (ch == word)
                {
                    if (first == -1) first = pos;
                    break;
                }
        }
    if (first == -1) return -1;
    if (first >= 0 && cmd[first+1] == ' ') return -1;
    if (first >= 0 && cmd[first+1] == '\0') return -1;
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


int parse_string(const char *cmd, char word, char *value)
{
    unsigned char i = 0, term_string = 0;
  
    // Find string
    // TODO:  check errno
    int16_t pos = find_word(cmd, word);
    if (pos < 0)
        return 0;

    while ( (!term_string) && ((pos + i) < MAX_CMD_SIZE) )
        {
            *value++ = cmd[pos + i + 1];
            if (cmd[pos + i + 1] == '\0')
                {
                    term_string = 1;
                }
            i++;
        }
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
    uint32_t checksum = 0;
    uint8_t qfree = 0;
#if SDSUPPORT > 0
    unsigned char i;
#endif
  
    // Validate the command's checksum, if provided.
    if (parse_hex(cmdstr, ';', &checksum)) {
        // 16-bit XMODEM cyclic redundancy check.
        uint16_t calculated_checksum = 0;
        if (checksum < 0x0000 || checksum > 0xFFFF)
            {
                serial_send(TXT_RS_SEQNUM_CHECKSUM_OUT_OF_RANGE_CRLF, cmdseqnbr);
                return;
            }
        for (int i = 0; i < MAX_CMD_SIZE && cmdstr[i] != ';'; i++)
            {
                calculated_checksum = _crc_xmodem_update(calculated_checksum, cmdstr[i]);
            }
        if (calculated_checksum != (uint16_t)checksum)
            {
                serial_send(TXT_RS_SEQNUM_INCORRECT_CHECKSUM_SHOULD_BE_CRLF, cmdseqnbr, calculated_checksum);
                return;
            }
    } else if (parse_uint(cmdstr, '*', &checksum)) {
        // 8-bit XOR checksum.
        uint8_t calculated_checksum = 0;
        if (checksum < 0 || checksum > 255)
            {
                serial_send(TXT_RS_SEQNUM_CHECKSUM_OUT_OF_RANGE_CRLF, cmdseqnbr);
                return;
            }
        for (int i = 0; i < MAX_CMD_SIZE && cmdstr[i] != '*'; i++)
            {
                calculated_checksum ^= cmdstr[i];
            }
        if (calculated_checksum != (uint8_t)checksum)
            {
                serial_send(TXT_RS_SEQNUM_INCORRECT_CHECKSUM_SHOULD_BE_CRLF, cmdseqnbr, calculated_checksum);
                return;
            }
    }

    // Validate the command's sequence number, if provided. 
    uint32_t seqnbr = 0;
    if (parse_uint(cmdstr, 'N', &seqnbr))
        {
            if (seqnbr == 0) cmdseqnbr = 0;
            //Ignore sequence number errors for now.
            /*
              if (seqnbr != cmdseqnbr) {
              serial_send("rs %ld (incorrect seqnbr) -> Expected: %ld, Found: %ld\r\n", cmdseqnbr, cmdseqnbr, seqnbr);
              return;
              }
            */
        }


    // Validate that the command has a single G or M code.
    int has_gcode = 0;
    int has_mcode = 0;
    code = -1;
    has_gcode = parse_int(cmdstr, 'G', &code);
    has_mcode = parse_int(cmdstr, 'M', &code);
    if (has_gcode && has_mcode)
        {
            // Take first command that appears as the command to process
            if ( find_word(cmdstr, 'G') < find_word(cmdstr, 'M') )
                {
                    has_mcode = 0;
                    has_gcode = parse_int(cmdstr, 'G', &code);
                }
            else
                {
                    has_gcode = 0;
                    has_mcode = parse_int(cmdstr, 'M', &code);
                }
        }
    if (!has_gcode && !has_mcode)
        {
            serial_send(TXT_RS_SEQNUM_COMMAND_CODE_MISSING_CRLF, cmdseqnbr, cmdstr);
            return;
        }
    if (code < 0 || code > 999)
        {
            serial_send(TXT_RS_SEQNUM_COMMAND_CODE_OUT_OF_RANGE_CRLF, cmdseqnbr);
            return;
        }

    // Create the 'struct command'.
    //struct command cmd;
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
    cmd.has_D = parse_int(cmdstr, 'D', &cmd.D);
    cmd.has_String = parse_string(&cmdstr[find_word(cmdstr, cmd.type)], ' ', &cmd.String[0]);

    // Dispatch command.
    unsigned long start_tm, end_tm, execute_tm;
    start_tm = millis();

#if SDSUPPORT > 0
    if (sdcard_write)
        {
            // if not M29 command
            if ( !((cmd.code == 29) & (cmd.type == 'M')) )
                {		
                    // if no more space in buffer
                    if ( (SDCARD_WRITEBUF_SIZE - sdcard_writebuf_pos) <= 0 )
                        {
                            // write data to file
                            sdcard_write_file(sdcard_fd, (uint8_t*) sdcard_writebuf, SDCARD_WRITEBUF_SIZE);		
			
                            // Reset buffer index / position
                            sdcard_writebuf_pos = 0;
                        }

                    for (i = 0; i < strlen(cmdstr); i++)
                        {
                            sdcard_writebuf[sdcard_writebuf_pos++] = cmdstr[i];
			
                            if ( (SDCARD_WRITEBUF_SIZE - sdcard_writebuf_pos) <= 0 )
                                {
                                    // write data to file
                                    sdcard_write_file(sdcard_fd, (uint8_t*) sdcard_writebuf, SDCARD_WRITEBUF_SIZE);		
				
                                    // Reset buffer index / position
                                    sdcard_writebuf_pos = 0;
                                }
                            // add remaining cmdstr to buffer
                        }

                    // Add line return / feed characters
                    sdcard_writebuf[sdcard_writebuf_pos++] = '\r';
		
                    if ( (SDCARD_WRITEBUF_SIZE - sdcard_writebuf_pos) <= 0 )
                        {
                            // write data to file
                            sdcard_write_file(sdcard_fd, (uint8_t*) sdcard_writebuf, SDCARD_WRITEBUF_SIZE);		
			
                            // Reset buffer index / position
                            sdcard_writebuf_pos = 0;
			
                            sdcard_writebuf[sdcard_writebuf_pos++] = '\n';
                        }
                    else
                        {
                            sdcard_writebuf[sdcard_writebuf_pos++] = '\n';
                        }
                }
            else
                {
                    // if there is any remaining data in write buffer write it to the SD 
                    // card file before closing it.
                    if (sdcard_writebuf_pos)
                        {
                            // write data to file
                            sdcard_write_file(sdcard_fd, (uint8_t*) sdcard_writebuf, sdcard_writebuf_pos);
					
                            // Reset buffer index / position
                            sdcard_writebuf_pos = 0;
                        }
		
                    // close the file
                    sdcard_closeFile(sdcard_fd);
                    sdcard_fd = 0;
		
                    sdcard_write = 0;
		
                    // issue "File upload complete" text to host
                    serial_send(TXT_FILE_UPLOAD_COMPLETE_CRLF);
                }
        }
    else
        {
#endif // SDSUPPORT
            serial_send(TXT_GO_SEQNUM_EXECUTING_CRLF, cmdseqnbr, cmd.type, cmd.code);
            switch (cmd.type) {
            case 'G':  execute_gcode(&cmd);  break;
            case 'M':  execute_mcode(&cmd);  break;
            default:	serial_send(TXT_UNKNOWN_COMMAND_TYPE); break;
            }
#if SDSUPPORT > 0
        }
#endif // SDSUPPORT
  
    end_tm = millis();
    execute_tm = end_tm - start_tm;
    qfree = blocks_available();
    serial_send(TXT_OK_SEQNUM_Q_MS_EXECUTE_CRLF, 
                cmdseqnbr, qfree, execute_tm);
    cmdseqnbr++;
    previous_millis_cmd = end_tm;
}


FORCE_INLINE void homing_routine(unsigned char axis)
{
    int min_pin, max_pin, home_dir, max_length, home_bounce;
    st_position_t pos;

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

            current_position[axis] = home_bounce/2.0 * home_dir;
            plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
            destination[axis] = 0;
            prepare_move();
            st_synchronize();

            current_position[axis] = -home_bounce * home_dir;
            plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
            destination[axis] = 0;
            feedrate = homing_feedrate[axis]/2.0;
            prepare_move();
            st_synchronize();

            current_position[axis] = (home_dir == -1) ? 0 : max_length;
            current_position[axis] += add_homing[axis];
            plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
            pos.x = current_position[X_AXIS] * axis_steps_per_unit[X_AXIS];
            pos.y = current_position[Y_AXIS] * axis_steps_per_unit[Y_AXIS];
            pos.z = current_position[Z_AXIS] * axis_steps_per_unit[Z_AXIS];
            pos.e = current_position[E_AXIS] * axis_steps_per_unit[E_AXIS];
            st_set_current_position(pos);
            destination[axis] = current_position[axis];
            feedrate = 0;
        }
  
    switch(axis){
    case X_AXIS:
        x_homed = 1;
        break;
    case Y_AXIS:
        y_homed = 1;
        break;
    case Z_AXIS:
        z_homed = 1;
        break;
    default:
        //never reached
        break;
    }
}

//------------------------------------------------
// CHECK COMMAND AND CONVERT VALUES
//------------------------------------------------
void execute_gcode(struct command *cmd)
{
    unsigned long codenum; //throw away variable
    st_position_t pos;
  
    switch(cmd->code) {
    case 0: // G0 -> G1
    case 1: // G1
        if (print_paused)
            {
                pos = st_get_current_position();
                current_position[X_AXIS] = pos.x ? (pos.x / (float)(axis_steps_per_unit[X_AXIS])) : 0;
                current_position[Y_AXIS] = pos.y ? (pos.y / (float)(axis_steps_per_unit[Y_AXIS])) : 0;
                current_position[Z_AXIS] = pos.z ? (pos.z / (float)(axis_steps_per_unit[Z_AXIS])) : 0;
                current_position[E_AXIS] = (pos.e != 0) ? (pos.e / (float)(axis_steps_per_unit[E_AXIS])) : 0;
			
                plan_set_position(current_position[X_AXIS], current_position[Y_AXIS],
                                  current_position[Z_AXIS], current_position[E_AXIS]);
            }
        get_coordinates(cmd); // For X Y Z E F
        prepare_move();
        previous_millis_cmd = millis();
        previous_millis_g_cmd = millis();
        break;
#ifdef USE_ARC_FUNCTION
    case 2: // G2  - CW ARC
        get_arc_coordinates(cmd);
        prepare_arc_move(1);
        previous_millis_cmd = millis();
        previous_millis_g_cmd = millis();
        break;
    case 3: // G3  - CCW ARC
        get_arc_coordinates(cmd);
        prepare_arc_move(0);
        previous_millis_cmd = millis();
        previous_millis_g_cmd = millis();
        break; 
#endif  
    case 4: // G4 dwell
        codenum = 0;
        if(cmd->has_P) codenum += cmd->P; // milliseconds to wait
        if(cmd->has_S) codenum += cmd->S * 1000; // seconds to wait
        serial_send(TXT_DELAYING_MS_CRLF, codenum);
        codenum += millis();  // keep track of when we started waiting
        st_synchronize();  // wait for all movements to finish
        while(millis()  < codenum ){
        }
        break;
    case 28: //G28 Home all Axis one at a time
        saved_feedrate = feedrate;
        saved_feedmultiply = feedmultiply;
        previous_millis_cmd = millis();
        previous_millis_g_cmd = millis();
        
        feedmultiply = 100;    
      
        enable_endstops(1);
      
        for(int i=0; i < NUM_AXIS; i++) 
            {
                destination[i] = current_position[i];
            }
        feedrate = 0;
        is_homing = 1;

        home_all_axis = !(cmd->has_X || cmd->has_Y || cmd->has_Z);

        if((home_all_axis) || cmd->has_X)
            homing_routine(X_AXIS);

        if((home_all_axis) || cmd->has_Y)
            homing_routine(Y_AXIS);

        if((home_all_axis) || cmd->has_Z)
            homing_routine(Z_AXIS);
        
#ifdef ENDSTOPS_ONLY_FOR_HOMING
        enable_endstops(0);
#endif
      
        is_homing = 0;
        feedrate = saved_feedrate;
        feedmultiply = saved_feedmultiply;
      
        previous_millis_cmd = millis();
        previous_millis_g_cmd = millis();
        break;
    case 90: // G90
        relative_mode = 0;
        break;
    case 91: // G91
        relative_mode = 1;
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
        serial_send(TXT_UNKNOWN_CODE_GX_CRLF, cmd->code);
        break;
    }
}


void execute_mcode(struct command *cmd) {
#if SDSUPPORT > 0
    long codenum, codenum2; //throw away variable
#endif // SDSUPPORT
    switch(cmd->code) {
#if SDSUPPORT > 0
    case 20: 	// M20 F0 - list SD card - value passed via the F are the flags
        // for displaying directories and file sizes.
        if (cmd->has_F)
            {
                serial_send(TXT_BEGIN_FILE_LIST_CRLF);	  
                sdcard_list_root((unsigned char)(cmd->F));
                serial_send(TXT_END_FILE_LIST_CRLF);
            }
        else
            {
                serial_send(TXT_BEGIN_FILE_LIST_CRLF);
                sdcard_list_root(LS_FILENAME_ONLY);
                serial_send(TXT_END_FILE_LIST_CRLF);
            }
        break;
	  
    case 21: // M21 - init SD card
        sdcard_initialise();
        break;
	  
    case 22: //M22 - release SD card
        sdcard_release();
        break;
	  
    case 23: //M23 - Select file
        if (cmd->has_String)
            {
		codenum = find_word(cmd->String, '*');
		
		if (codenum > 0)
                    {
			strlcpy(sdard_filename, cmd->String, codenum);
                    }
		else
                    {
			strcpy(sdard_filename, cmd->String);
                    }
		
		serial_send(TXT_OPENING_STR_CRLF, sdard_filename);
		
		if ( sdcard_openFile(sdard_filename) )
                    {
			serial_send(TXT_FILE_SELECTED_CRLF);				
                    }
		else
                    {
			serial_send(TXT_FILE_OPEN_FAILED_CRLF);
                    }
            }
        else
            {
                serial_send(TXT_FILE_OPEN_FAILED_CRLF);
            }
        break;
	  
    case 24: //M24 - Start SD print
        if ( (cmd->has_P) && (cmd->P <= -255) )
            {
		// Cancel SD print
		serial_send(TXT_CRLF_CANCELLED_SD_CARD_PRINT_CRLF);
		// Close file
		sdcard_closeFile(sdcard_fd);
		sdcard_fd = 0;
		sdcard_print = 0;
		serial_send(TXT_DONE_PRINTING_FILE_CRLF);
            }
        else
            {
		sdcard_print = 1;
		sdcard_print_pause = 0;
            }
        break;
	  
    case 25: //M25 - Pause SD print
        sdcard_print_pause = 1;
        break;
	  
        /*    case 26: //M26 - Set SD index
              break;
        */	  
    case 27: //M27 - Get SD print status
        codenum = sdcard_get_current_file_position(sdcard_fd);
        codenum2 = sdcard_get_filesize(sdcard_fd);
        serial_send(TXT_SD_PRINTING_BYTE_LU_OF_LU_CRLF, codenum, codenum2);
        break;
	  
    case 28: //M28 - Start SD write
        if ( (!sdcard_fd) && (cmd->has_String) )
            {
		codenum = find_word(cmd->String, '*');
		
		if (codenum > 0)
                    {
			strlcpy(sdard_filename, cmd->String, codenum);
                    }
		else
                    {
			strcpy(sdard_filename, cmd->String);
                    }
		
		serial_send(TXT_OPENING_STR_CRLF, sdard_filename);
		
		codenum = sdcard_create_file(sdard_filename);
		
		if ( codenum > 2)
                    {
			serial_send(TXT_COULD_NOT_CREATE_FILE_CRLF);
			serial_send(TXT_ERROR_CODE_LU_CRLF, codenum);
			serial_send(TXT_FILE_OPEN_FAILED_CRLF);
                    }
		else if ( codenum )
                    {
			if ( sdcard_openFile(sdard_filename) )
                            {
				// Move file position to end of file to prevent over-writing
				// any existing data if the file already existed
				sdcard_file_goto_eof(sdcard_fd);
				
				sdcard_write = 1;
				serial_send(TXT_WRITING_TO_FILE_CRLF);
                            }
			else
                            {
				serial_send(TXT_FILE_OPEN_FAILED_CRLF);
                            }
                    }
		else
                    {
			serial_send(TXT_COULD_NOT_CREATE_FILE_CRLF);
			serial_send(TXT_FILE_OPEN_FAILED_CRLF);
                    }
            }
        else
            {
		sdcard_write = 0;
		serial_send(TXT_FILE_OPEN_FAILED_CRLF);
            }
        break;
	  
    case 29: //M29 - Stop SD write
        //processed in write to file routine above
        //sdcard_write = 0;
        break;

    case 30: //M30 <filename> Delete File
        if ( cmd->has_String )
            {
		codenum = find_word(cmd->String, '*');
		
		if (codenum > 0)
                    {
			strlcpy(sdard_filename, cmd->String, codenum);
                    }
		else
                    {
			strcpy(sdard_filename, cmd->String);
                    }
		
		serial_send(TXT_DELETING_STR_CRLF, sdard_filename);
		
		sdcard_delete_file(sdard_filename);
            }
        break;
#endif //SDSUPPORT
#if AUTOPRINT > 0
    case 31: // M31 - Enable / Disable autoprint from SD card
        if (cmd->has_E) {
            if (cmd->E == 1) enable_autoprint();
            else if (cmd->E == 0) disable_autoprint();    
        }
        if (autoprint_enabled) serial_send(TXT_M31_AUTOPRINT_ENABLED_CRLF);
        else serial_send(TXT_M31_AUTOPRINT_DISABLED_CRLF);
#endif //AUTOPRINT	
    case 104: // M104 - Set Extruder Temperature
#ifdef CHAIN_OF_COMMAND
        st_synchronize(); // wait for all movements to finish
#endif
        if (cmd->has_S)
            {	
                if (cmd->S > HOTEND_MAX_TARGET_TEMP)
                    {
                        serial_send(TXT_MAX_ALLOWED_HOTEND_TEMP_IS_INT_DEGC_CRLF, HOTEND_MAX_TARGET_TEMP);
                        serial_send(TXT_SETTING_TARGET_HOTEND_TEMP_TO_INT_DEGC_CRLF, HOTEND_MAX_TARGET_TEMP);
                    }
                target_temp = (cmd->S <= HOTEND_MAX_TARGET_TEMP) ? cmd->S : HOTEND_MAX_TARGET_TEMP;
                target_raw = temp2analogh(target_temp);

#ifdef WATCHPERIOD
                if( target_temp > (analog2temp(current_raw) + MIN_TARGET_TEMP_CHANGE) )
                    {
                        watchmillis = MAX(1,millis());
                        watch_temp = analog2temp(current_raw);
                    }
                else
                    {
                        watchmillis = 0;
                    }		
#endif
            }
        break;
		
    case 140: // M140 set bed temp
#ifdef CHAIN_OF_COMMAND
        st_synchronize(); // wait for all movements to finish
#endif
#if TEMP_1_PIN > -1
        if (cmd->has_S)
            {
                if (cmd->S > BED_MAX_TARGET_TEMP)
                    {
                        serial_send(TXT_MAX_ALLOWED_BED_TEMP_IS_INT_DEGC_CRLF, BED_MAX_TARGET_TEMP);
                        serial_send(TXT_SETTING_TARGET_BED_TEMP_TO_INT_DEGC_CRLF, BED_MAX_TARGET_TEMP);
                    }
                target_bed_raw = temp2analogBed( (cmd->S <= BED_MAX_TARGET_TEMP) ? 
                                                 cmd->S : BED_MAX_TARGET_TEMP );
            }
#endif
        break;
		
    case 105: // M105
#if (TEMP_0_PIN > -1)
        hotendtC = analog2temp(current_raw);
#endif
#if TEMP_1_PIN > -1
        bedtempC = analog2tempBed(current_bed_raw);
#endif
#if (TEMP_0_PIN > -1)
        serial_send(TXT_OK_T_INT, hotendtC);
#ifdef PIDTEMP
        serial_send(TXT_D_INT_PERCENT, (int)( (heater_duty * 100) / (float)(HEATER_CURRENT) ));
#ifdef AUTOTEMP
        serial_send(TXT_A_INT, autotemp_setpoint);
#endif
#endif
#if TEMP_1_PIN > -1
        serial_send(TXT_B_INT, bedtempC);
#endif
#ifdef BED_PIDTEMP
        serial_send(TXT_D_INT_PERCENT, (int)( (bed_heater_duty * 100) / (float)(BED_HEATER_CURRENT) ));
#endif
        serial_send(TXT_CRLF);
#else
#error No temperature source available
#endif
        break;
		
    case 109:  // M109 - Wait for extruder heater to reach target.
#ifdef CHAIN_OF_COMMAND
        st_synchronize(); // wait for all movements to finish
#endif
        if (cmd->has_S)
            {
                if (cmd->S > HOTEND_MAX_TARGET_TEMP)
                    {
                        serial_send(TXT_MAX_ALLOWED_HOTEND_TEMP_IS_INT_DEGC_CRLF, HOTEND_MAX_TARGET_TEMP);
                        serial_send(TXT_SETTING_TARGET_HOTEND_TEMP_TO_INT_DEGC_CRLF, HOTEND_MAX_TARGET_TEMP);
                    }
                target_temp = (cmd->S <= HOTEND_MAX_TARGET_TEMP) ? cmd->S : HOTEND_MAX_TARGET_TEMP;
                target_raw = temp2analogh(target_temp);
			
#ifdef WATCHPERIOD
                if( target_temp > (analog2temp(current_raw) + MIN_TARGET_TEMP_CHANGE) )
                    {
                        watchmillis = MAX(1,millis());
                        watch_temp = analog2temp(current_raw);
                    }
                else
                    {
                        watchmillis = 0;
                    }
#endif
        
		wait_extruder_target_temp();
            }
        break;
	  
    case 190: // M190 - Wait for bed heater to reach target temperature.
#ifdef CHAIN_OF_COMMAND
        st_synchronize(); // wait for all movements to finish
#endif
#if TEMP_1_PIN > -1
        if (cmd->has_S)
            {
                if (cmd->S > BED_MAX_TARGET_TEMP)
                    {
                        serial_send(TXT_MAX_ALLOWED_BED_TEMP_IS_INT_DEGC_CRLF, BED_MAX_TARGET_TEMP);
                        serial_send(TXT_SETTING_TARGET_BED_TEMP_TO_INT_DEGC_CRLF, BED_MAX_TARGET_TEMP);
                    }
			
                target_bed_raw = temp2analogBed( (cmd->S <= BED_MAX_TARGET_TEMP) ? 
                                                 cmd->S : BED_MAX_TARGET_TEMP );
		
		wait_bed_target_temp();
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
                unsigned char l_fan_code_val = CONSTRAIN(cmd->S,0,ICR3);
            
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
          
                setFanPWMDuty(l_fan_code_val);			
            }
        else 
            {
                setFanPWMDuty(ICR3);
            }
        break;
		
    case 107: //M107 Fan Off
        setFanPWMDuty(0); 
        break;
#endif
#if (PS_ON_PIN > -1)
    case 80: // M80 - ATX Power On
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
        axis_relative_modes[3] = 0;
        break;
		
    case 83:
        axis_relative_modes[3] = 1;
        break;
		
    case 18:	// M18
    case 84:	// M84
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
                serial_send(TXT_S_PARAM_REQUIRED_CRLF);
            }
        break;
		
    case 92: // M92
        execute_m92(cmd);
        break;
		
    case 93: // M93 show current axis steps.
        serial_send(TXT_OK_X_Y_Z_E_CRLF,
                    axis_steps_per_unit[0],
                    axis_steps_per_unit[1],
                    axis_steps_per_unit[2],
                    axis_steps_per_unit[3]
                    );
        break;
		
    case 115: // M115
        serial_send(TXT_FIRMWARE_NAME_STR_CRLF);
        serial_send("00000000-0000-0000-0000-000000000000\r\n");
        serial_send(TXT_MAKIBOX_FIRMWARE_VERSION_STR_CRLF, VERSION_TEXT);
        break;
		
    case 114: // M114
        do_position_report();
        break;
		
    case 119: // M119
        serial_send(TXT_FORWARDSLASH_FORWARDSLASH);
#if (X_MIN_PIN > -1)
        serial_send(TXT_X_MIN_STR, (READ(X_MIN_PIN)^X_ENDSTOP_INVERT)?"H ":"L ");
#endif
#if (X_MAX_PIN > -1)
        serial_send(TXT_X_MAX_STR, (READ(X_MAX_PIN)^X_ENDSTOP_INVERT)?"H ":"L ");
#endif
#if (Y_MIN_PIN > -1)
        SET_INPUT(Y_MIN_PIN);
        serial_send(TXT_Y_MIN_STR, (READ(Y_MIN_PIN)^Y_ENDSTOP_INVERT)?"H ":"L ");
#endif
#if (Y_MAX_PIN > -1)
        SET_INPUT(Y_MAX_PIN);
        serial_send(TXT_Y_MAX_STR, (READ(Y_MAX_PIN)^Y_ENDSTOP_INVERT)?"H ":"L ");
#endif
#if (Z_MIN_PIN > -1)
        serial_send(TXT_Z_MIN_STR, (READ(Z_MIN_PIN)^Z_ENDSTOP_INVERT)?"H ":"L ");
#endif
#if (Z_MAX_PIN > -1)
        serial_send(TXT_Z_MAX_STR, (READ(Z_MAX_PIN)^Z_ENDSTOP_INVERT)?"H ":"L ");
#endif
        serial_send(TXT_CRLF); 
        serial_send(TXT_X_MAX_LENGTH_INT, X_MAX_LENGTH);
        serial_send(TXT_Y_MAX_LENGTH_INT, Y_MAX_LENGTH);
        serial_send(TXT_Z_MAX_LENGTH_INT_CRLF, Z_MAX_LENGTH);
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
		  
        if (cmd->has_P)
            {
                periodic_temp_report = cmd->P;
            }
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
        serial_send(TXT_M206_ADDHOME_X_Y_Z_CRLF, add_homing[0], add_homing[1], add_homing[2]);
        if (cmd->has_X) add_homing[X_AXIS] = cmd->X;
        if (cmd->has_Y) add_homing[Y_AXIS] = cmd->Y;
        if (cmd->has_Z) add_homing[Z_AXIS] = cmd->Z;
        break;  
	  
    case 220: // M220 S<factor in percent>- set speed factor override percentage
        if(cmd->has_S) 
            {
                feedmultiply = CONSTRAIN(cmd->S, 20, 200);
                feedmultiplychanged=1;
            }
        break;
	  
    case 221: // M221 S<factor in percent>- set extrude factor override percentage
        if(cmd->has_S) 
            extrudemultiply = CONSTRAIN(cmd->S, 40, 200);
        break;
	  
    case 226:	// M226 - M226 / M226 P1 = Pause print  M226 P0 = resume print
        execute_m226(cmd);
        break;

#if BUZZER_SUPPORT > 0 
    case 300: // M300 - Starts the buzzer with freqeuncy f and period p
        if(cmd->has_F) BUZZER_F = (unsigned short) cmd->F;
        if(cmd->has_P) BUZZER_P = (unsigned short) cmd->P;
        buzzer_tone();
        break;
#endif

#ifdef PIDTEMP
    case 301: // M301
        if(cmd->has_P) PID_Kp = (unsigned int)cmd->P;
        if(cmd->has_I) PID_Ki = (unsigned int)cmd->I;
        if(cmd->has_D) PID_Kd = (unsigned int)cmd->D;
        updatePID();
        serial_send(TXT_PID_SETTINGS_CHANGED_NOT_SAVED_TO_MEM_CRLF);
        break;
#endif //PIDTEMP      

#if PREVENT_DANGEROUS_EXTRUDE > 0
    case 302:	// M302 - Enable / Disable Cold Extrudes P1 = allow cold extrudes; P0 = Do not allow cold extrudes
        if (cmd->has_P)
            {
                if (cmd->P >= 1)
                    {
                        prevent_cold_extrude = 0;
                    }
                else if (cmd->P < 1)
                    {
                        prevent_cold_extrude = 1;
                    }
            }
        else
            {
                prevent_cold_extrude = 0;
            }
        break;
#endif
	  
#ifdef PID_AUTOTUNE
    case 303: // M303 PID autotune
        if (cmd->has_S)
            PID_autotune((int)(cmd->S));
        break;
#endif
#ifdef BED_PIDTEMP
    case 305: // M305 set hot bed max duty cycle
        if (cmd->has_S && cmd->has_D){
            short val = (short) ((cmd->D/100.0)*BED_HEATER_CURRENT);
            if (val > BED_HEATER_CURRENT) val=BED_HEATER_CURRENT;
            else if (val < 0 ) val=0;
            // Stage 1 (before 70C): Set max duty cycle
            if (cmd->S == 1) user_max_bed_heater_duty_before_full_pwr = (unsigned short)val;
            // Stage 2 (after 70C): Set max duty cycle
            else if (cmd->S == 2) user_max_bed_heater_duty = (unsigned short) val;
        }
        serial_send(TXT_MAX_BED_HEATER_DUTY_SETTINGS_CRLF_M305_CRLF,
                    user_max_bed_heater_duty_before_full_pwr*100/BED_HEATER_CURRENT,
                    user_max_bed_heater_duty*100/BED_HEATER_CURRENT);     
        break;
#endif
    case 400: // M400 - finish all moves
      	st_synchronize();	
        break;
	  
#ifdef USE_EEPROM_SETTINGS
    case 500: // M500 - Store settings in EEPROM
        EEPROM_StoreSettings();
        break;
	  
    case 501: // M501 - Read settings from EEPROM
        EEPROM_RetrieveSettings(0, 1);
        for(int8_t i=0; i < NUM_AXIS; i++)
            {
                axis_steps_per_sqr_second[i] = max_acceleration_units_per_sq_second[i] * axis_steps_per_unit[i];
            }
        break;
	  
    case 502: // M502 - Revert to default settings
        EEPROM_RetrieveSettings(1, 0);
        for(int8_t i=0; i < NUM_AXIS; i++)
            {
                axis_steps_per_sqr_second[i] = max_acceleration_units_per_sq_second[i] * axis_steps_per_unit[i];
            }
        EEPROM_StoreSettings();
        // TODO:
        // M502 command doesn't reset the digipot resistor values automatically
        // Check other settings too
        break;
	  
    case 503: // M503 - print current settings
        EEPROM_printSettings();
        break;  
#endif      
    case 603: // M603 - Free RAM
        serial_send(TXT_MAKIBOX_FIRMWARE_VERSION_STR_CRLF, VERSION_TEXT);
        serial_send(TXT_FREE_RAM_CRLF, FreeRam1());
        break;
	  
    case 604:	// M604 Show Timer 1 COMPA ISR Execution Time Debug Info
#if (DEBUG > -1)
        serial_send(TXT_LAST_TIMER1_COMPA_VECT_ISR_EXE_TIME_LU_US_CRLF, 
                    timer1_compa_isr_exe_micros);
        serial_send(TXT_MIN_TIMER1_COMPA_VECT_ISR_EXE_TIME_LU_US_CRLF, 
                    timer1_compa_isr_exe_micros_min);
        serial_send(TXT_MAX_TIMER1_COMPA_VECT_ISR_EXE_TIME_LU_US_CRLF, 
                    timer1_compa_isr_exe_micros_max);
#else
        serial_send(TXT_TIMER1_EXE_TIME_DEBUG_NOT_AVAILABLE_CRLF);
#endif
        break;
	  
    case 605:	// M605 Reset Timer 1 COMPA ISR Execution Time Min / Max Values
#if (DEBUG > -1)
        timer1_compa_isr_exe_micros_min = 0xFFFFFFFF;
        timer1_compa_isr_exe_micros_max = 0;
        serial_send(TXT_TIMER1_COMPA_VECT_ISR_EXE_TIME_MIN_MAX_RESET_CRLF);
#else
        serial_send(TXT_TIMER1_EXE_TIME_DEBUG_NOT_AVAILABLE_CRLF);
#endif
        break;
	  
    case 606: // M606 - Show CPU loading information
#if (DEBUG > -1)
        serial_send(TXT_CURRENT_CPU_LOADING_INT_PERCENT_CRLF, cpu_loading);
        serial_send(TXT_PEAK_CPU_LOAD_INT_PERCENT_CRLF, peak_cpu_load);
        serial_send(TXT_AVERAGE_CPU_LOAD_INT_PERCENT_CRLF, average_cpu_load);
#else
        serial_send(TXT_CPU_LOADING_INFO_NOT_AVAIL_IN_THIS_VER_CRLF);
#endif
        break;
	  
    case 607: // M607 - Reset Peak and Average CPU load values
#if (DEBUG > -1)
        peak_cpu_load = 0;
        average_cpu_load = 0;
        serial_send(TXT_PEAK_N_AVERAGE_CPU_LOAD_VALUES_RESET_CRLF);
#else
        serial_send(TXT_CPU_LOADING_INFO_NOT_AVAIL_IN_THIS_VER_CRLF);
#endif
        break;
	  
    case 608: // M608
        serial_send(TXT_MAKIBOX_FIRMWARE_VERSION_STR_CRLF, VERSION_TEXT);
        break;
	  
    case 610:	// M610 - Set Extruder Heater Max Current P = 0 - 100%.
        set_extruder_heater_max_current(cmd);
        break;
	  
    case 852: // M852 - Enter Boot Loader Command
        if (cmd->has_F)
            {
                if (cmd->F == BOOTLOADER_PASSCODE)
                    {
                        serial_send(TXT_CRLF_MAKIBOX_BOOTLOADER_CRLF);
                        serial_send(TXT_ENTERING_BOOTLOADER_CRLF);
                        serial_send(TXT_EXISTING_USB_CONN_WILL_BE_DISCONNECTED_CRLF);
                        serial_send(TXT_PLEASE_DISCONNECT_N_CLOSE_HOST_SW_CRLF_CRLF);
					
                        // Delay to allow time for message to be sent before disabling
                        // the USB in the JumpToBootloader() function.
                        delay(2000);
					
                        JumpToBootloader();
                    }
                else
                    {
                        serial_send(TXT_CRLF_MAKIBOX_BOOTLOADER_CRLF);
                        serial_send(TXT_CANNOT_ENTER_BOOTLOADER_INCORRECT_PASSCODE_CRLF);
                        serial_send(TXT_PLEASE_TRY_AGAIN_WITH_CORRECT_PASSCODE_CRLF);
                    }
            }
        else
            {
                serial_send(TXT_CRLF_MAKIBOX_BOOTLOADER_CRLF);
                serial_send(TXT_CANNOT_ENTER_BOOTLOADER_PASSCODE_NOT_FOUND_CRLF);
                serial_send(TXT_PLEASE_TRY_AGAIN_WITH_CORRECT_PASSCODE_CRLF);
            }
        break;
	  
    case 112:	// M112 - Emergency Stop
        kill();
        serial_send(TXT_CRLF_EMERGENCY_STOP_CRLF);
        serial_send(TXT_MOTORS_OFF_HEATERS_OFF_CRLF);
        _restart_Teensyduino_();
        break;
	  
    case 609: // M609 - Show last reset flags
        serial_send(TXT_CRLF_RESET_FLAGS_HEX_SPACE, reset_flags);
        serial_send(TXT_OPEN_BRACKETS_SPACE);
        if ( reset_flags & (1 << JTRF) )
            {
                serial_send(TXT_JTAG_SPACE);
            }
        if ( reset_flags & (1 << WDRF) )
            {
                serial_send(TXT_WDT_SPACE);
            }
        if ( reset_flags & (1 << BORF) )
            {
                serial_send(TXT_BOR_SPACE);
            }
        if ( reset_flags & (1 << EXTRF) )
            {
                serial_send(TXT_EXT_SPACE);
            }
        if ( reset_flags & (1 << PORF) )
            {
                serial_send(TXT_POR_SPACE);
            }
        serial_send((TXT_CLOSE_BRACKETS TXT_CRLF));
        break;
	  
#if DIGIPOTS > 0
    case 906: // M906 - Set current limits for stepper motors
        execute_m906(cmd);
        break;
#endif
#if SET_MICROSTEP > 0
    case 907: // M907 Set microstep settings for stepper motors
        execute_m907(cmd);
        break;
#endif

    default:
        serial_send(TXT_UNKNOWN_CODE_M_CRLF, cmd->code);
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
	
    // Update current position as steps per mm have been changed.
    st_position_t pos;
    pos.x = current_position[X_AXIS];
    pos.y = current_position[Y_AXIS];
    pos.z = current_position[Z_AXIS];
    pos.e = current_position[E_AXIS];
    st_set_current_position(pos);
	
    //plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], 
    //				current_position[Z_AXIS], current_position[E_AXIS]);
    serial_send(TXT_STEPS_PER_MM_SETTINGS_CHANGED_NOT_SAVED_TO_MEM_CRLF);
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


void execute_m226(struct command *cmd)
{	
    st_position_t pos;
	
    if ((cmd->has_P) && (cmd->P == 0))
	{	// Resume print
            if (print_paused)
		{
                    feedrate = 1000;
			
                    // Ensure target temperatures set to same as when print was paused
                    target_temp = paused_data.hotend_target_temp;
                    target_raw = paused_data.hotend_target_temp_raw;
                    target_bed_raw = paused_data.target_bed_temp_raw;
			
                    serial_send(TXT_CRLF_RESUMING_PRINT_PLEASE_WAIT);
			
                    // Allow time for hot-bed and hot-end to reach target
                    if ( target_bed_raw > temp2analogBed(BEDMINTEMP) )
			{
                            wait_bed_target_temp();
			}
                    if ( target_temp > MINTEMP )
			{
                            wait_extruder_target_temp();
			}
			
                    // Move print head and z-axis to position before print was paused
			
                    st_position_t pos = st_get_current_position();
			
                    // Move X and Y Axes
                    destination[X_AXIS] = paused_data.paused_pos_x;
                    destination[Y_AXIS] = paused_data.paused_pos_y;
                    destination[Z_AXIS] = pos.z ? (pos.z / (float)(axis_steps_per_unit[Z_AXIS])) : 0;
                    destination[E_AXIS] = (pos.e != 0) ? (pos.e / (float)(axis_steps_per_unit[E_AXIS])) : 0;
			
                    prepare_move();
			
                    // Wait for moves to complete
                    st_synchronize();
			
                    // Move Z-Axis
                    destination[Z_AXIS] = paused_data.paused_pos_z;
			
                    prepare_move();
			
                    // Wait for moves to complete
                    st_synchronize();
			
                    current_position[X_AXIS] = paused_data.current_position_x;
                    current_position[Y_AXIS] = paused_data.current_position_y;
                    current_position[Z_AXIS] = paused_data.current_position_z;
                    current_position[E_AXIS] = paused_data.current_position_e;
			
                    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], 
                                      current_position[Z_AXIS], current_position[E_AXIS]);
			
                    resume_normal_print_buffer();
			
                    print_paused = 0;
		}
	}
    else if ( (cmd->has_P) && (cmd->P <= -255) )
	{
            // Clear plan buffer & resume normal operation
            if (print_paused)
		{
                    feedrate = 1000;
			
                    serial_send(TXT_CRLF_CLEARING_BUFFERED_MOVES_RESUME_NORMAL_OP_CRLF);
                    resume_normal_buf_discard_all_buf_moves();
			
                    pos = st_get_current_position();
                    current_position[X_AXIS] = pos.x ? (pos.x / (float)(axis_steps_per_unit[X_AXIS])) : 0;
                    current_position[Y_AXIS] = pos.y ? (pos.y / (float)(axis_steps_per_unit[Y_AXIS])) : 0;
                    current_position[Z_AXIS] = pos.z ? (pos.z / (float)(axis_steps_per_unit[Z_AXIS])) : 0;
                    current_position[E_AXIS] = (pos.e != 0) ? (pos.e / (float)(axis_steps_per_unit[E_AXIS])) : 0;
			
                    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS],
                                      current_position[Z_AXIS], current_position[E_AXIS]);
								
                    print_paused = 0;
		}
	}
    else
	{
            if (!print_paused)
		{
                    // Pause print after current block has finished
                    pause_print_req = 1;
			
                    serial_send(TXT_CRLF_PAUSING_PRINT_CRLF);
			
                    paused_data.current_position_x = current_position[X_AXIS];
                    paused_data.current_position_y = current_position[Y_AXIS];
                    paused_data.current_position_z = current_position[Z_AXIS];
                    paused_data.current_position_e = current_position[E_AXIS];
			
                    feedrate = 1000;

                    print_paused = 1;
		}
	}
}


void update_axis_pos(int axis, float pos)
{
    destination[axis] = pos + (axis_relative_modes[axis] || relative_mode)*current_position[axis];
}

void get_coordinates(struct command *cmd)
{
    if (cmd->has_X)
        {
            update_axis_pos(X_AXIS, cmd->has_X ? cmd->X : current_position[X_AXIS]);
        }
    else
        {
            destination[X_AXIS] = current_position[X_AXIS];
        }
  
    if (cmd->has_Y)
        {
            update_axis_pos(Y_AXIS, cmd->has_Y ? cmd->Y : current_position[Y_AXIS]);
        }
    else
        {
            destination[Y_AXIS] = current_position[Y_AXIS];
        }
	
    if (cmd->has_Z)
        {
            update_axis_pos(Z_AXIS, cmd->has_Z ? cmd->Z : current_position[Z_AXIS]);
        }
    else
        {
            destination[Z_AXIS] = current_position[Z_AXIS];
        }
  
    if (cmd->has_E)
        {
            update_axis_pos(E_AXIS, cmd->has_E ? cmd->E : current_position[E_AXIS]);
        }
    else
        {
            destination[E_AXIS] = current_position[E_AXIS];
        }
  
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
	if (x_homed)
            {
		if (destination[X_AXIS] < 0) destination[X_AXIS] = 0.0;
            }
	
	if (y_homed)
            {
		if (destination[Y_AXIS] < 0) destination[Y_AXIS] = 0.0;
            }
	
	if (z_homed)
            {
		if (destination[Z_AXIS] < 0) destination[Z_AXIS] = 0.0;
            }
#endif

#if MAX_SOFTWARE_ENDSTOPS
        if (destination[X_AXIS] > X_MAX_LENGTH) destination[X_AXIS] = X_MAX_LENGTH;
        if (destination[Y_AXIS] > Y_MAX_LENGTH) destination[Y_AXIS] = Y_MAX_LENGTH;
        if (destination[Z_AXIS] > Z_MAX_LENGTH) destination[Z_AXIS] = Z_MAX_LENGTH;
#endif
    }

    if(destination[E_AXIS] > current_position[E_AXIS])
        {
            help_feedrate = (long)((long)feedrate*(long)feedmultiply);
        }
    else
        {
            help_feedrate = (long)((long)feedrate*(long)100);
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
    target_temp=0;
    setHeaterPWMDuty(HEATER_0_PIN, 0);
    WRITE(HEATER_0_PIN,LOW);
#endif
  
#if TEMP_1_PIN > -1
    target_bed_raw=0;
    if(HEATER_1_PIN > -1)
	{
            setHeaterPWMDuty(HEATER_1_PIN, 0);
            WRITE(HEATER_1_PIN,LOW);
	}
#endif

    disable_x();
    disable_y();
    disable_z();
    disable_e();
}


void manage_inactivity(unsigned char debug) 
{ 
    manage_heater();
  
    if( (millis()-previous_millis_cmd) >  max_inactive_time )
        {
            if(max_inactive_time)
                {
                    kill();
                    serial_send(TXT_HEATERS_AND_MOTORS_DISABLED_DUE_INACTIVITY_CRLF);
                    previous_millis_cmd = millis();
                }
        }
  
    if( (millis()-previous_millis_g_cmd) >  stepper_inactive_time ) if(stepper_inactive_time) 
                                                                        { 
#if (DEBUG > -1)
                                                                            PreemptionFlag |= 0x0004;
#endif
	
                                                                            disable_x(); 
                                                                            disable_y(); 
                                                                            disable_z(); 
                                                                            disable_e();
	
                                                                            serial_send(TXT_STEPPER_MOTORS_AUTO_DISABLED_DUE_TO_INACTIVITY_CRLF);
                                                                            previous_millis_g_cmd = millis();
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
                    if (FAN_PIN > -1)
                        {
                            WRITE(FAN_PIN, HIGH);
                            setFanPWMDuty(fan_org_start_speed); 
                        }
       
                    fan_org_start_speed = 0;
                }  
        }
}
#endif


/***************************************************
 * do_position_report(void)
 *
 * Reports the current position of the stepper motors
 * and sends the information to the host.
 *
 * The axes should be homed after power up to allow
 * an accurate report of the position. Otherwise, the
 * starting positions on power up will be taken to be
 * the zero positions. X, Y, and Z axes do not report
 * positions less than zero.
 ****************************************************/
void do_position_report(void)
{
    char x_mm_str[10];
    char y_mm_str[10];
    char z_mm_str[10];
    char e_mm_str[10];

    st_position_t pos = st_get_current_position();
    float x_mm = (pos.x != 0) ? (pos.x / (float)(axis_steps_per_unit[X_AXIS])) : 0;
    float y_mm = (pos.y != 0) ? (pos.y / (float)(axis_steps_per_unit[Y_AXIS])) : 0;
    float z_mm = (pos.z != 0) ? (pos.z / (float)(axis_steps_per_unit[Z_AXIS])) : 0;
    float e_mm = (pos.e != 0) ? (pos.e / (float)(axis_steps_per_unit[E_AXIS])) : 0;
	
    dtostrf(x_mm, 3, 5, x_mm_str);
    dtostrf(y_mm, 3, 5, y_mm_str);
    dtostrf(z_mm, 3, 5, z_mm_str);
    dtostrf(e_mm, 3, 5, e_mm_str);
	
    serial_send(TXT_C_X_Y_Z_E_MM_CRLF, 
                x_mm_str, y_mm_str, z_mm_str, e_mm_str);
									
    serial_send(TXT_X_Y_Z_E_STEPS_CRLF, 
                pos.x, pos.y, pos.z, pos.e);
												
    serial_send(TXT_AXES_HOMED_X_Y_Z_CRLF, x_homed, y_homed, z_homed);
	
    if (!x_homed || !y_homed || !z_homed)
	{
            serial_send(TXT_NOT_ALL_AXES_HOMED_POSITION_MAYBE_INCORRECT_CRLF);
	}
}


/***************************************************
 * wait_extruder_target_temp(void)
 *
 * Waits for extruder heater to reach target 
 * temperature (defined by target_temp variable).
 *
 ****************************************************/
void wait_extruder_target_temp(void)
{
    unsigned long timer;
	
    timer = millis(); 
       
    /* See if we are heating up or cooling down */
    // true if heating, false if cooling
    int target_direction = (current_raw < target_raw) ? 1 : 0;
	
    int target_raw_low = temp2analogh(target_temp - TEMP_HYSTERESIS);
    int target_raw_high = temp2analogh(target_temp + TEMP_HYSTERESIS);
	
    serial_send(TXT_CRLF_TARGET_TEMP_DEGC, target_temp);
    serial_send(TXT_WAITING_FOR_EXTRUDER_TO_REACH_TARGET_TEMP_CRLF);
	
    long hotend_timeout = millis();
	
#ifdef TEMP_RESIDENCY_TIME
    long residencyStart;
    residencyStart = -1;
    /* continue to loop until we have reached the target temp   
       _and_ until TEMP_RESIDENCY_TIME hasn't passed since we reached it */
    while( (target_direction ? (current_raw < target_raw_low) : (current_raw > target_raw_high))
           || (residencyStart > -1 && (millis() - residencyStart) < TEMP_RESIDENCY_TIME*1000)
           || (residencyStart == -1) ) {
#else
	while ( target_direction ? (current_raw < target_raw_low) : (current_raw > target_raw_high) ) {
#endif
            if( (millis() - timer) > 1000 ) //Print Temp Reading every 1 second while heating up/cooling down
                {
                    serial_send(TXT_T_D_B_D_CRLF, analog2temp(current_raw), 
                                (int)( (heater_duty * 100) / (float)(HEATER_CURRENT)),
                                analog2tempBed(current_bed_raw),
                                (int)( (bed_heater_duty * 100) / (float)(BED_HEATER_CURRENT) ));
		
                    timer = millis();
                }
#if (MINIMUM_FAN_START_SPEED > 0)
            manage_fan_start_speed();
#endif
#ifdef TEMP_RESIDENCY_TIME
            /* start/restart the TEMP_RESIDENCY_TIME timer whenever we reach target temp for the first time
               or when current temp falls outside the hysteresis after target temp was reached */
            if (   (residencyStart == -1)
                   || (residencyStart > -1 && labs(analog2temp(current_raw) - analog2temp(target_raw)) > TEMP_HYSTERESIS) ) {
                residencyStart = millis();
            }
#endif
	  
            // Timeout if target not reached after HOTEND_HEATUP_TIMEOUT 
            // milli-seconds has passed. Exit loop if timeout reached.
            if ( (millis() - hotend_timeout) > HOTEND_HEATUP_TIMEOUT )
                {
                    serial_send(TXT_CRLF_HOTEND_TOO_LONG_TO_REACH_TARGET_TIMED_OUT_CRLF);
                    break;
                }
	  
            if ( (target_temp == 0) || (target_raw == 0) )
                {
                    serial_send(TXT_HOTEND_HEATER_NOT_RESPONDING_CRLF);
                    serial_send(TXT_STOP_PRINT_POWER_OFF_PRINTER_DISCONNECT_CRLF);
                    serial_send(TXT_CHECK_HOTEND_N_HOTBED_THERMISTOR_CONNECTIONS_CRLF);
		
                    serial_send(TXT_FIRMWARE_WILL_CONTINUE_OP_AFTER_30_SECS_CRLF);
		
                    delay(30000);
		
                    serial_send(TXT_CONTINUING_CRLF);
                    break;
                }
	}
    }


    /***************************************************
     * wait_bed_target_temp(void)
     *
     * Waits for hotbed heater to reach target 
     * temperature (defined by target_temp variable).
     *
     ****************************************************/
    void wait_bed_target_temp(void)
    {
	unsigned long timer = millis();
	unsigned long bed_timeout = millis();
	
	serial_send(TXT_CRLF_TARGET_TEMP_DEGC, analog2tempBed(target_bed_raw));
	serial_send(TXT_CRLF_WAITING_FOR_HOTBED_HEATER_TO_REACH_TARGET_CRLF);

	while(current_bed_raw < target_bed_raw) 
            {
                if( (millis()-timer) > 1000 ) //Print Temp Reading every 1 second while heating up.
                    {
                        serial_send(TXT_T_D_B_D_CRLF, analog2temp(current_raw), 
                                    (int)( (heater_duty * 100) / (float)(HEATER_CURRENT) ),
                                    analog2tempBed(current_bed_raw),
                                    (int)( (bed_heater_duty * 100) / (float)(BED_HEATER_CURRENT) ));
		
                        timer = millis(); 
                    }
#if (MINIMUM_FAN_START_SPEED > 0)
		manage_fan_start_speed();
#endif
	  
                // Timeout if target not reached after HOTEND_HEATUP_TIMEOUT 
                // milli-seconds has passed. Exit loop if timeout reached.
                if ( (millis() - bed_timeout) > BED_HEATUP_TIMEOUT )
                    {
                        serial_send(TXT_CRLF_HOTBED_HEATER_TOOK_TOO_LONG_TIMED_OUT_CRLF);
                        break;
                    }
	  
                if (target_bed_raw == 0)
                    {
                        serial_send(TXT_CRLF_HOTBED_HEATER_APPEARS_NOT_RESPONDING_CRLF);
                        serial_send(TXT_CHECK_HOTBED_THERMISTOR_CONNECTIONS);
                        break;
                    }
            }
    }


    void set_extruder_heater_max_current(struct command *cmd)
    {
	if (cmd->has_P)
            {
		serial_send(TXT_EXISTING_EXTRUDER_MAX_CURRENT_INT_PERCENT_CRLF, (int)(user_max_heater_duty * 100 / 250.0));
		serial_send(TXT_SETTING_EXTRUDER_MAX_CURRENT_INT_PERCENT_CRLF, (int)(cmd->P));
		user_max_heater_duty = (int)(cmd->P * 2.5);
            }
	else
            {
		serial_send(TXT_EXISTING_EXTRUDER_MAX_CURRENT_INT_PERCENT_CRLF, (int)(user_max_heater_duty * 100 / 250.0));
            }
    }


#if DIGIPOTS > 0
    // M906 - Set current limits for stepper motors e.g. M906 X1700 Y1700 Z1700 E1700
    void execute_m906(struct command *cmd){
    if (cmd->has_S){
		if (cmd->S == 100 || cmd->S == 120) set_stepper_motors_sense_resistance((unsigned char)cmd->S);
        else set_stepper_motors_sense_resistance(ALLEGRO_A4982_RS);      
    }
	if (cmd->has_X){
		if (cmd->X > 0) set_stepper_motors_max_current(X_AXIS, (unsigned short)cmd->X);
		else set_stepper_motors_max_current(X_AXIS, 0);
    }
    else{
        set_stepper_motors_max_current(X_AXIS, max_x_motor_current);
    }
	if (cmd->has_Y){
		if (cmd->Y > 0) set_stepper_motors_max_current(Y_AXIS, (unsigned short)cmd->Y);
		else set_stepper_motors_max_current(Y_AXIS, 0);
    }
    else{
        set_stepper_motors_max_current(Y_AXIS, max_y_motor_current);
    }
	if (cmd->has_Z){
		if (cmd->Z > 0)	set_stepper_motors_max_current(Z_AXIS, (unsigned short)cmd->Z);
		else set_stepper_motors_max_current(Z_AXIS, 0);
    }
    else{
        set_stepper_motors_max_current(Z_AXIS, max_z_motor_current);
    }
	if (cmd->has_E){
		if (cmd->E > 0) set_stepper_motors_max_current(E_AXIS, (unsigned short)cmd->E);
        else set_stepper_motors_max_current(E_AXIS, 0);
    }
    else{
        set_stepper_motors_max_current(E_AXIS, max_e_motor_current);
    }
	
	/* M906 should initate a temprorary change
           and M500 should save the EEPROM Settings
          
           if ( (cmd->has_X) || (cmd->has_Y) || (cmd->has_Z) || (cmd->has_E) )
           {
           EEPROM_StoreSettings();
           serial_send(TXT_NEW_STEPPER_MOTOR_MAX_CURRENTS_SET_CRLF);
           serial_send(TXT_CRLF);
           }
	*/
	serial_send(TXT_MAX_MOTOR_CURRENTS_CRLF_M906_X_Y_Z_E_CRLF, max_x_motor_current, 
                max_y_motor_current,
                max_z_motor_current,
                max_e_motor_current, 
                stepper_sense_resistance);
    }
#endif

#if SET_MICROSTEP > 0
    void num2MS(float val, unsigned short* MS){
        if (val == 2 || val == 16) MS[0] = HIGH;
        else if (val == 1 || val == 4) MS[0] = LOW;
        else MS[0] = HIGH;

        if (val == 4 || val == 16) MS[1] = HIGH;
        else if (val == 1 || val == 2) MS[1] = LOW;
        else MS[1] = HIGH;

        // TODO: 
        // Need to add checking for invalid values
        // For now, any invalid values return the motor to microstepping
    }

    int MS2num(unsigned short* MS){
        if (MS[0]){
            if (MS[1]) return 16;
            else return 2;               
        }
        else{
            if (MS[1]) return 4;
            else return 1;  
        }
    }

    void execute_m907(struct command *cmd)
    {
#if DEBUG > -1
        if (cmd->has_X) num2MS(cmd->X, microstep_x);
        if (cmd->has_Y) num2MS(cmd->Y, microstep_y);
        if (cmd->has_Z) num2MS(cmd->Z, microstep_z);
        if (cmd->has_E) num2MS(cmd->E, microstep_e);
#endif
        serial_send(TXT_MAX_MOTOR_CURRENTS_CRLF_M907_X_Y_Z_E_CRLF, 
                    MS2num(microstep_x), MS2num(microstep_y),
                    MS2num(microstep_z), MS2num(microstep_e)
                    );
    }
#endif

    /***************************************************
     * JumpToBootloader(void)
     *
     * This function detaches the USB connection and
     * disables the mircocontroller peripherals. Then 
     * jumps to the DFU bootloader start address.
     *
     * Note: The DFU bootloader must be present and setup
     * correctly in the Atmel AT90USB1286 microcontroller.
     *
     * This function does not return to the main 
     * application!
     ****************************************************/
    void JumpToBootloader(void)
    {
	cli();
	delay(10);				// Delay milli-seconds
	UDCON = 1;				// Detach USB Connection
	USBCON = (1<<FRZCLK);	// Disable the USB clock
	delay(100);
	
	// Disable Peripherals
	EIMSK = 0; PCICR = 0; SPCR = 0; ACSR = 0; EECR = 0; ADCSRA = 0;
	TIMSK0 = 0; TIMSK1 = 0; TIMSK2 = 0; TIMSK3 = 0; UCSR1B = 0; TWCR = 0;
	DDRA = 0; DDRB = 0; DDRC = 0; DDRD = 0; DDRE = 0; DDRF = 0;
	PORTA = 0; PORTB = 0; PORTC = 0; PORTD = 0; PORTE = 0; PORTF = 0;
	
	// Jump to Bootloader start address
	asm volatile("jmp 0x1F000");
	while (1) ;
    }

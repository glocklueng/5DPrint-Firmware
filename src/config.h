/* config.h
*
* History:
* =======
*
* + 	02 NOV 2012		Author: JTK Wong (XTRONTEC Limited)
*		Commented out #define FAN_SOFT_PWM and #define PID_SOFT_PWM. This is to 
*		free up Timer 2. Slightly less processing as the Timer 2 ISRs are no 
*		longer called.
*
* +		14 NOV 2012		Author: JTK Wong (XTRONTEC Limited)
*		Added some defines associated with HOT BED PID control. PID parameters 
*		have been copied from the extruder heater PID settings. The PID 
*		parameters will need to be tuned for the hot bed and adjusted here.
*
* +		15 NOV 2012		Author: JTK Wong 	XTRONTEC Limited
*											www.xtrontec.com
*		Removed controller fan pin defines
*		Does not map to suitable control pin on PrintRBoard rev B.
*
*		Removed extruder fan pin defines.
*		Does not map to suitable control pin on PrintRBoard rev B.
*
* +		29 NOV 2012		Author: JTK Wong 	XTRONTEC Limited
*											www.xtrontec.com
*		Added global variables for CPU loading calculation.
*		Added DEBUG #define to enable/disable calculation.
*
* +		07 Dec 2012		Author: JTK Wong 	XTRONTEC Limited
*											www.xtrontec.com
*		HEATER_CHECK_INTERVAL decreased to 100 so that extruder temperature is 
*		checked (sampled) more frequently. Extruder PID parameters edited.
*/

#ifndef CONFIGURATION_H
#define CONFIGURATION_H

// Define DEBUG to > -1 to enable CPU loading calculations
#define DEBUG 1

// BASIC SETTINGS: select thermistor type, axis scaling, and endstop configuration

//// Thermistor settings:
// 1 is 100k thermistor
// 2 is 200k thermistor
// 3 is mendel-parts thermistor
// 4 is 10k thermistor
// 5 is ParCan supplied 104GT-2 100K
// 6 is EPCOS 100k
// 7 is 100k Honeywell thermistor 135-104LAG-J01
#define THERMISTORHEATER 1
#define THERMISTORBED 1

//// Calibration variables
// X, Y, Z, E steps per unit
// Makibox A6 has screws which advance 8mm per 200 steps.  25 steps = 1mm.
// Makiox A6 standard stepper motors have 1.8 deg/step. However with micro-
// stepping feature 1/16th step is achieved.
// Therefore 1mm movement in x, y, z = (200 / 8) * 16 = 400 micro-steps.
#define _AXIS_STEP_PER_UNIT {400, 400, 400, 150}


//// Endstop Settings
#define ENDSTOPPULLUPS // Comment this out (using // at the start of the line) to disable the endstop pullup resistors
// The pullups are needed if you directly connect a mechanical endswitch between the signal and ground pins.
//If your axes are only moving in one direction, make sure the endstops are connected properly.
//If your axes move in one direction ONLY when the endstops are triggered, set [XYZ]_ENDSTOP_INVERT to true here:
#define X_ENDSTOP_INVERT 0
#define Y_ENDSTOP_INVERT 0
#define Z_ENDSTOP_INVERT 0

//-----------------------------------------------------------------------
//// STORE SETTINGS TO EEPROM
//-----------------------------------------------------------------------
// the microcontroller can store settings in the EEPROM
// M500 - stores paramters in EEPROM
// M501 - reads parameters from EEPROM (if you need reset them after you changed them temporarily).
// M502 - reverts to the default "factory settings". You still need to store them in EEPROM afterwards if you want to.
// M503 - Print settings
// define this to enable eeprom support
#define USE_EEPROM_SETTINGS
#define PRINT_EEPROM_SETTINGS

//-----------------------------------------------------------------------
//// ARC Function (G2/G3 Command)
//-----------------------------------------------------------------------
//Uncomment to activate the arc (circle) function (G2/G3 Command)
#define USE_ARC_FUNCTION

//-----------------------------------------------------------------------
//// ADVANCED SETTINGS - to tweak parameters
//-----------------------------------------------------------------------

//-----------------------------------------------------------------------
// For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1
//-----------------------------------------------------------------------
#define X_ENABLE_ON 0
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 0
#define E_ENABLE_ON 0

//Uncomment if you have problems with a stepper driver enabeling too late, this will also set how many microseconds delay there will be after enabeling the driver
//#define DELAY_ENABLE 15

//-----------------------------------------------------------------------
// Disables axis when it's not being used.
//-----------------------------------------------------------------------
#define DISABLE_X 0
#define DISABLE_Y 0
#define DISABLE_Z 1
#define DISABLE_E 0

//-----------------------------------------------------------------------
// Inverting axis direction
//-----------------------------------------------------------------------
#define INVERT_X_DIR 0
#define INVERT_Y_DIR 0
#define INVERT_Z_DIR 0
#define INVERT_E_DIR 1

//-----------------------------------------------------------------------
//// ENDSTOP SETTINGS:
//-----------------------------------------------------------------------
// Sets direction of endstops when homing; 1=MAX, -1=MIN
#define X_HOME_DIR -1
#define Y_HOME_DIR -1
#define Z_HOME_DIR -1

//#define ENDSTOPS_ONLY_FOR_HOMING // If defined the endstops will only be used for homing

// If true, axis won't move to coordinates less than zero.
#define MIN_SOFTWARE_ENDSTOPS 0
// If true, axis won't move to coordinates greater than the defined lengths below.
#define MAX_SOFTWARE_ENDSTOPS 1


//-----------------------------------------------------------------------
//Max Length for Prusa Mendel, check the ways of your axis and set this Values
//-----------------------------------------------------------------------
#define X_MAX_LENGTH 115
#define Y_MAX_LENGTH 165
#define Z_MAX_LENGTH 100

//-----------------------------------------------------------------------
//// MOVEMENT SETTINGS
//-----------------------------------------------------------------------
#define _MAX_FEEDRATE {120, 120, 20, 45} // (mm/sec)    
#define _HOMING_FEEDRATE {1500,1500,120} // (mm/min) !!
#define _AXIS_RELATIVE_MODES {0, 0, 0, 0}

#define MAX_STEP_FREQUENCY 30000 // Max step frequency

//For the retract (negative Extruder) move this maxiumum Limit of Feedrate is used
//The next positive Extruder move use also this Limit, 
//then for the next (second after retract) move the original Maximum (_MAX_FEEDRATE) Limit is used
#define MAX_RETRACT_FEEDRATE 100    //mm/sec

//-----------------------------------------------------------------------
//// Acceleration settings
//-----------------------------------------------------------------------
// X, Y, Z, E maximum start speed for accelerated moves. E default values are good for skeinforge 40+, for older versions raise them a lot.
#define _ACCELERATION 1000         // Axis Normal acceleration mm/s^2
#define _RETRACT_ACCELERATION 2000 // Extruder Normal acceleration mm/s^2
#define _MAX_XY_JERK 20.0
#define _MAX_Z_JERK 0.4
#define _MAX_E_JERK 5.0    // (mm/sec)
// X, Y, Z and E max acceleration in mm/s^2 for printing moves or retracts
#define _MAX_ACCELERATION_UNITS_PER_SQ_SECOND {5000,5000,50,5000}    


// Minimum planner junction speed. Sets the default minimum speed the planner plans for at the end
// of the buffer and all stops. This should not be much greater than zero and should only be changed
// if unwanted behavior is observed on a user's machine when running at very slow speeds.
#define MINIMUM_PLANNER_SPEED 0.05 // (mm/sec)

#define DEFAULT_MINIMUMFEEDRATE       0.0     // minimum feedrate
#define DEFAULT_MINTRAVELFEEDRATE     0.0

// If defined the movements slow down when the look ahead buffer is only half full
#define SLOWDOWN


// everything with less than this number of steps will be ignored as move and
// joined with the next movement
#define DROP_SEGMENTS 0


//-----------------------------------------------------------------------
// Planner buffer Size
//-----------------------------------------------------------------------
// The number of linear motions that can be in the plan at any give time.
// This must be a power-of-two!  Currently, sizeof(struct block) is 72 
// bytes.
// 
//    SIZE    MASK    RAM (bytes)
//    8	      0x07    576
//    16      0x0F    1152
//    32      0x1F    2304
//    64      0x3F    4608
//    128     0x7F    9216
#define BLOCK_BUFFER_SIZE 32
#define BLOCK_BUFFER_MASK 0x1F

//-----------------------------------------------------------------------
//// SETTINGS FOR ARC FUNCTION (Command G2/G2)
//-----------------------------------------------------------------------

// Arc interpretation settings:
//Step to split a cirrcle in small Lines 
#define MM_PER_ARC_SEGMENT 1
//After this count of steps a new SIN / COS caluclation is startet to correct the circle interpolation
#define N_ARC_CORRECTION 25



//-----------------------------------------------------------------------
//// MINIMUM START SPEED FOR FAN
//-----------------------------------------------------------------------

//Minimum start speed for FAN when the last speed was zero
//Set to 0 to deaktivate
//If value is set the fan will drive with this minimum speed for MINIMUM_FAN_START_TIME
#define MINIMUM_FAN_START_SPEED  0

//This is the time how long the minimum FAN speed is set
#define MINIMUM_FAN_START_TIME  6000    //6sec

//-----------------------------------------------------------------------
//// HEATERCONTROL AND PID PARAMETERS
//-----------------------------------------------------------------------

//Testfunction to adjust the Hotend temperatur in case of Printingspeed
//If the Printer print slow the Temp is going to AUTO_TEMP_MIN
//At the moment this Value dont change the targettemp from the Hotend
//The result of this function is only send with the Temperaturerequest to the host
//#define AUTOTEMP 
#ifdef AUTOTEMP
    #define AUTO_TEMP_MAX 350
    #define AUTO_TEMP_MIN 205
    #define AUTO_TEMP_FACTOR 0.025
    #define AUTOTEMP_OLDWEIGHT 0.98
#endif

//// PID settings:
// Uncomment the following line to enable PID support. This is untested and could be disastrous. Be careful.
#define PIDTEMP 1
#ifdef PIDTEMP

// M303 - PID relay autotune S<temperature> sets the target temperature. 
// (default target temperature = 150C)
#define PID_AUTOTUNE

//PID Controler Settings
#define PID_INTEGRAL_DRIVE_MAX 80 // too big, and heater will lag after changing temperature, too small and it might not compensate enough for long-term errors
#define PID_PGAIN 6000  //256 is 1.0  // value of X means that error of 1 degree is changing PWM duty by X, probably no need to go over 25
#define PID_IGAIN 120   //256 is 1.0  // value of X (e.g 0.25) means that each degree error over 1 sec (2 measurements) changes duty cycle by 2X (=0.5) units (verify?)
#define PID_DGAIN 13000 //256 is 1.0  // value of X means that around reached setpoint, each degree change over one measurement (half second) adjusts PWM by X units to compensate

// magic formula 1, to get approximate "zero error" PWM duty. Take few measurements with low PWM duty and make linear fit to get the formula
// for my makergear hot-end: linear fit {50,10},{60,20},{80,30},{105,50},{176,100},{128,64},{208,128}
#define HEATER_DUTY_FOR_SETPOINT(setpoint) ((int)((187L*(long)setpoint)>>8)-27)  
// magic formula 2, to make led brightness approximately linear
#define LED_PWM_FOR_BRIGHTNESS(brightness) ((64*brightness-1384)/(300-brightness))
#endif

// Change this value (range 30-250) to limit the current to the nozzle
#define HEATER_CURRENT 250

// How often should the heater check for new temp readings, in milliseconds
#define HEATER_CHECK_INTERVAL 100
#define BED_CHECK_INTERVAL 3000

// Comment the following line to enable heat management during acceleration
#define DISABLE_CHECK_DURING_ACC
#ifndef DISABLE_CHECK_DURING_ACC
  // Uncomment the following line to disable heat management during moves
  //#define DISABLE_CHECK_DURING_MOVE
#endif


/// Hot Bed PID settings:
// Uncomment the following line to enable PID support. This is untested and could be disastrous. Be careful.
#define BED_PIDTEMP 1
#ifdef BED_PIDTEMP

//Hot Bed PID Controler Settings
#define BED_PID_INTEGRAL_DRIVE_MAX 120 // too big, and heater will lag after changing temperature, too small and it might not compensate enough for long-term errors
#define BED_PID_PGAIN 3560 //256 is 1.0  // value of X means that error of 1 degree is changing PWM duty by X, probably no need to go over 25
#define BED_PID_IGAIN 64 //256 is 1.0  
#define BED_PID_DGAIN 4096 //256 is 1.0  // value of X means that around reached setpoint, each degree change over one measurement (half second) adjusts PWM by X units to compensate

// Change this value (range 30-250) to limit the current to the HOT BED
#define BED_HEATER_CURRENT 250

#endif // #ifdef BED_PIDTEMP


// Uncomment the following line to disable heat management during travel moves (and extruder-only moves, eg: retracts), strongly recommended if you are missing steps mid print.
// Probably this should remain commented if are using PID.
// It also defines the max milliseconds interval after which a travel move is not considered so for the sake of this feature.
#define DISABLE_CHECK_DURING_TRAVEL 1000

//// Temperature smoothing - only uncomment this if your temp readings are noisy (Gen6 without EvdZ's 5V hack)
//#define SMOOTHING
//#define SMOOTHFACTOR 16 //best to use a power of two here - determines how many values are averaged together by the smoothing algorithm


//// Experimental watchdog and minimal temp
// The watchdog waits for the watchperiod in milliseconds whenever an M104 or M109 increases the target temperature
// If the temperature has not increased at the end of that period, the target temperature is set to zero. It can be reset with another M104/M109
//#define WATCHPERIOD 5000 //5 seconds

// Actual temperature must be close to target for this long before M109 returns success
//#define TEMP_RESIDENCY_TIME 20  // (seconds)
//#define TEMP_HYSTERESIS 5       // (C°) range of +/- temperatures considered "close" to the target one

//// The minimal temperature defines the temperature below which the heater will not be enabled
#define MINTEMP 5

//// Experimental max temp
// When temperature exceeds max temp, your heater will be switched off.
// This feature exists to protect your hotend from overheating accidentally, but *NOT* from thermistor short/failure!
// You should use MINTEMP for thermistor short/failure protection.
#define MAXTEMP 350

// Select one of these only to define how the nozzle temp is read.
#define HEATER_USES_THERMISTOR

// Select one of these only to define how the bed temp is read.
#define BED_USES_THERMISTOR

//#define CHAIN_OF_COMMAND 1 //Finish buffered moves before executing M42, fan speed, heater target, and so...


// Global Variables for CPU Loading Calculation
extern uint16_t PreemptionFlag;
extern uint32_t previous_bckgnd_task_start_time;
extern uint32_t bckgnd_task_time;
extern unsigned char cpu_loading, peak_cpu_load, average_cpu_load;
extern uint32_t previous_millis_cpu_util;
extern uint32_t bckgnd_loop_count;

#endif

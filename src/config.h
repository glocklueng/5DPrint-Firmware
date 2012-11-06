/* config.h
*
* History:
* =======
*
* + 	02 NOV 2012		Author: JTK Wong (XTRONTEC Limited)
*		Commented out #define FAN_SOFT_PWM and #define PID_SOFT_PWM. This is to free up 
*		Timer 2. Slightly less processing as the Timer 2 ISRs are no longer called.
*/

#ifndef CONFIGURATION_H
#define CONFIGURATION_H

// BASIC SETTINGS: select axis scaling and endstop configuration

//// Calibration variables
// X, Y, Z, E steps per unit - Metric Prusa Mendel with Wade extruder:
#define _AXIS_STEP_PER_UNIT {80, 80, 3200/1.25,700}
// Metric Prusa Mendel with Makergear geared stepper extruder:
//#define _AXIS_STEP_PER_UNIT {80,80,3200/1.25,1380}
// MakerGear Hybrid Prusa Mendel:
// Z axis value is for .9 stepper(if you have 1.8 steppers for Z, you need to use 2272.7272)
//#define _AXIS_STEP_PER_UNIT {104.987, 104.987, 4545.4544, 1487}


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
#define INVERT_E_DIR 0

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
#define _MAX_FEEDRATE {800, 400, 2, 45}       // (mm/sec)    
#define _HOMING_FEEDRATE {1500,1500,120}      // (mm/min) !!
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
#define DROP_SEGMENTS 5


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
//// FANCONTROL WITH SOFT PWM
//-----------------------------------------------------------------------

//With this option its possible to drive the fan with SOFT PWM (500hz) and use
//every Digital output for it, main usage for Sanguinololu
//#define FAN_SOFT_PWM

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

//// AD595 THERMOCOUPLE SUPPORT UNTESTED... USE WITH CAUTION!!!!

//// PID settings:
// Uncomment the following line to enable PID support. This is untested and could be disastrous. Be careful.
#define PIDTEMP 1
#ifdef PIDTEMP
//Sanguinololu 1.2 and above, the PWM Output Hotend Timer 1 is used for the Hardware PWM
//but in this Software use Timer1 for the Stepperfunction so it is not possible to use the "analogWrite" function.
//This Soft PWM use Timer 2 with 400 Hz to drive the PWM for the hotend
//#define PID_SOFT_PWM

// M303 - PID relay autotune S<temperature> sets the target temperature. 
// (default target temperature = 150C)
#define PID_AUTOTUNE

//PID Controler Settings
#define PID_INTEGRAL_DRIVE_MAX 80 // too big, and heater will lag after changing temperature, too small and it might not compensate enough for long-term errors
#define PID_PGAIN 2560 //256 is 1.0  // value of X means that error of 1 degree is changing PWM duty by X, probably no need to go over 25
#define PID_IGAIN 64 //256 is 1.0  // value of X (e.g 0.25) means that each degree error over 1 sec (2 measurements) changes duty cycle by 2X (=0.5) units (verify?)
#define PID_DGAIN 4096 //256 is 1.0  // value of X means that around reached setpoint, each degree change over one measurement (half second) adjusts PWM by X units to compensate

// magic formula 1, to get approximate "zero error" PWM duty. Take few measurements with low PWM duty and make linear fit to get the formula
// for my makergear hot-end: linear fit {50,10},{60,20},{80,30},{105,50},{176,100},{128,64},{208,128}
#define HEATER_DUTY_FOR_SETPOINT(setpoint) ((int)((187L*(long)setpoint)>>8)-27)  
// magic formula 2, to make led brightness approximately linear
#define LED_PWM_FOR_BRIGHTNESS(brightness) ((64*brightness-1384)/(300-brightness))
#endif

// Change this value (range 30-255) to limit the current to the nozzle
#define HEATER_CURRENT 255

// How often should the heater check for new temp readings, in milliseconds
#define HEATER_CHECK_INTERVAL 500
#define BED_CHECK_INTERVAL 5000

// Comment the following line to enable heat management during acceleration
#define DISABLE_CHECK_DURING_ACC
#ifndef DISABLE_CHECK_DURING_ACC
  // Uncomment the following line to disable heat management during moves
  //#define DISABLE_CHECK_DURING_MOVE
#endif

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
//#define HEATER_USES_AD595
//#define HEATER_USES_MAX6675

// Select one of these only to define how the bed temp is read.
#define BED_USES_THERMISTOR
//#define BED_USES_AD595

//This is for controlling a fan to cool down the stepper drivers
//it will turn on when any driver is enabled
//and turn off after the set amount of seconds from last driver being disabled again
//#define CONTROLLERFAN_PIN 23 //Pin used for the fan to cool controller, comment out to disable this function
#define CONTROLLERFAN_SEC 60 //How many seconds, after all motors were disabled, the fan should run

//This is for controlling a fan that will keep the extruder cool.
//#define EXTRUDERFAN_PIN 66 //Pin used to control the fan, comment out to disable this function
#define EXTRUDERFAN_DEC 50 //Hotend temperature from where the fan will be turned on

//#define CHAIN_OF_COMMAND 1 //Finish buffered moves before executing M42, fan speed, heater target, and so...


#endif

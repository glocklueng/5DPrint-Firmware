/*
 Makibox A6 heater functions based on Reprap.
 Reprap heater funtions based on Sprinter
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>. */

/*
 This softwarepart for Heatercontrol is based on Sprinter
 big thanks to kliment (https://github.com/kliment/Sprinter)
 
*/


#include "config.h"


#ifndef HEATER_H
#define HEATER_H


#define NUMTEMPS      61
#define BNUMTEMPS     30
extern const short temptable[NUMTEMPS][2];
extern const short bedtemptable[BNUMTEMPS][2];

// Bed temperature must be greater than this value for hotend heater to be allowed
// to draw 100% power. Only 1/3 of power will be allowed otherwise.
#define MIN_BED_TEMP_FOR_HOTEND_FULL_PWR 	50		// degC

#if defined HEATER_USES_THERMISTOR
#define temp2analogh( c ) temp2analog_thermistor(c,temptable,NUMTEMPS)
#define analog2temp( c ) analog2temp_thermistor(c,temptable,NUMTEMPS)
#endif

#if defined BED_USES_THERMISTOR
#define temp2analogBed( c ) temp2analog_thermistor((c),bedtemptable,BNUMTEMPS)
#define analog2tempBed( c ) analog2temp_thermistor((c),bedtemptable,BNUMTEMPS)
#endif

#if defined (HEATER_USES_THERMISTOR) || defined (BED_USES_THERMISTOR)
int temp2analog_thermistor(int celsius, const short table[][2], int numtemps);
int analog2temp_thermistor(int raw,const short table[][2], int numtemps);
#endif

extern int target_raw;
extern int target_temp;
extern int current_raw;
extern int current_raw_maxval;
extern int current_raw_minval;
extern int tt_maxval;
extern int tt_minval;
extern int target_bed_raw;
extern int current_bed_raw;
extern unsigned long previous_millis_heater, previous_millis_bed_heater;
extern unsigned char manage_monitor;

#if (PIDTEMP > -1)
	extern int temp_iState;
	extern int temp_dState;
	extern int prev_temp;
	extern int pTerm;
	extern int iTerm;
	extern int dTerm;
	extern int error;
	extern int heater_duty;

	extern unsigned int PID_Kp, PID_Ki, PID_Kd;
#endif


#if (BED_PIDTEMP > -1)
	extern int temp_bed_iState;
	extern int temp_bed_dState;
	extern int prev_bed_temp;
	extern int bed_pTerm;
	extern int bed_iTerm;
	extern int bed_dTerm;
	extern int bed_error;
	extern int bed_heater_duty;

	extern unsigned int bed_PID_Kp, bed_PID_Ki, bed_PID_Kd;
#endif


#ifdef AUTOTEMP
    extern float autotemp_max;
    extern float autotemp_min;
    extern float autotemp_factor;
    extern int   autotemp_setpoint;
    extern bool autotemp_enabled;
#endif


#ifdef SMOOTHING
	extern uint32_t nma;
#endif


#ifdef WATCHPERIOD
	extern int watch_temp;
	extern unsigned long watchmillis;
#endif


#ifdef PID_AUTOTUNE
	void PID_autotune(int PIDAT_test_temp);
#endif


#if (PIDTEMP > -1) || (BED_PIDTEMP > -1)
	void updatePID();
#endif

void manage_heater();
void init_Timer3_HW_pwm(void);
void setHeaterPWMDuty(uint8_t pin, int val);
void setFanPWMDuty(int val);

#endif // #ifndef HEATER_H
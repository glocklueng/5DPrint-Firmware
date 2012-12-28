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

/*
* History:
* =======
*
* +		08 NOV 2012		Author: JTK Wong (XTRONTEC Limited)
*		Added init_Timer3_HW_pwm(void) to initialise Timer 3 for hardware
*		PWM. Phase correct PWM rate set at 500Hz similar to SOFT_PWM.
*
* +		14 NOV 2012		Author: JTK Wong (XTRONTEC Limited)
*		First attempt to add PWM and PID control for Hot Bed heater. Basically
*		just replicating PID control code for the extruder heater which is also
*		within manage_heater(). The code could do with being tidied up some more
*		when time permits - too many lines of code in one function.
*
* +		15 NOV 2012		Author: JTK Wong 	XTRONTEC Limited
*											www.xtrontec.com
*		Removed some unused variables and functions.
*		Removed code for MAX6675 and AD595 thermocouple amplifier / interface
*		ICs since they are unlikely to be used with the Makibox.
*
*		controllerFan() removed as controller fan pin defined in previous code
*		does not map to suitable control pin on PrintRBoard rev B.
*
*		extruderFan() removed as extruder fan pin defined in previous code
*		does not map to suitable control pin on PrintRBoard rev B.
*/


#include <avr/interrupt.h>
#include "pgmspace.h"
#include <math.h>
#include <stdlib.h>

#include "heater.h"
#include "board_io.h"
#include "pins.h"
#include "pins_teensy.h"
#include "makibox.h"
#include "usb.h"

#include "heater_table.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))


// Manage heater variables. For a thermistor, raw values refer to the 
// reading from the analog pin. 
int target_raw = 0;
int target_temp = 0;
int current_raw = 0;
int current_raw_maxval = -32000;
int current_raw_minval = 32000;
int tt_maxval;
int tt_minval;
int target_bed_raw = 0;
int current_bed_raw = 0;
unsigned long previous_millis_heater, previous_millis_bed_heater, previous_millis_monitor;

#ifdef PIDTEMP
  int temp_iState = 0;
  int temp_dState = 0;
  int prev_temp = 0;
  int pTerm;
  int iTerm;
  int dTerm;
  int error;
  int heater_duty = 0;
  int temp_iState_min = (int)( 256L * -PID_INTEGRAL_DRIVE_MAX / (float)(PID_IGAIN) );
  int temp_iState_max = (int)( 256L * PID_INTEGRAL_DRIVE_MAX / (float)(PID_IGAIN) );
#endif

#ifdef BED_PIDTEMP
  int temp_bed_iState = 0;
  int temp_bed_dState = 0;
  int prev_bed_temp = 0;
  int bed_pTerm;
  int bed_iTerm;
  int bed_dTerm;
  int bed_error;
  int bed_heater_duty = 0;

  int temp_bed_iState_min = (int)( 256L * -BED_PID_INTEGRAL_DRIVE_MAX / (float)(BED_PID_IGAIN) );
  int temp_bed_iState_max = (int)( 256L * BED_PID_INTEGRAL_DRIVE_MAX / (float)(BED_PID_IGAIN) );
#endif


#ifdef AUTOTEMP
    float autotemp_max=AUTO_TEMP_MAX;
    float autotemp_min=AUTO_TEMP_MIN;
    float autotemp_factor=AUTO_TEMP_FACTOR;
    int   autotemp_setpoint=0;
    int   autotemp_enabled=1;
#endif

#ifndef HEATER_CURRENT
  #define HEATER_CURRENT 255
#endif

#ifdef SMOOTHING
	uint32_t nma = 0;
#endif

#ifdef WATCHPERIOD
	int watch_raw = -1000;
	unsigned long watchmillis = 0;
#endif

#ifdef MINTEMP
	int minttemp; /* = temp2analogh(MINTEMP); */
#endif

#ifdef MAXTEMP
	int maxttemp; /* = temp2analogh(MAXTEMP); */
#endif


#define HEAT_INTERVAL 250

//------------------------------------------------------------------------
// Function Prototypes
// For functions used within this module only.
//------------------------------------------------------------------------
void service_TemperatureMonitor(void);
void service_ExtruderHeaterPIDControl(int current_temp, int target_temp);
void service_BedHeaterPIDControl(int current_bed_temp, int target_bed_temp);
void service_ExtruderHeaterSimpleControl(int current_raw, int target_raw);
void service_BedHeaterSimpleControl(int current_bed_raw, int target_bed_raw);


//------------------------------------------------------------------------
// Setup Timer and PWM for Heater and FAN
//------------------------------------------------------------------------

void init_Timer3_HW_pwm(void)
{
	// This is hardware PWM with 500 Hz for Extruder Heating and Fan
	// We want to have at least 500Hz - equivalent to previous SOFT_PWM
	
	// For phase correct PWM mode:
	// PWM Freq = Fclk / (2 * prescaler * TOP)

	TIFR3 = (1 << TOV3) | (1 << OCF3A) | (1 << OCF3B);  	// clear interrupt and output compare match flags
	TCCR3B = (1 << CS31) | (1 << CS30) | (1 << WGM33);	// start timer (ck/64 prescalar)
	TCCR3A = (1 << WGM31); 								// phase correct PWM
															// 'TOP' set by ICR3
	ICR3 = 250;												// TOP
  
	// EXTRUDER HEATER PWM
	OCR3B = 0;						// Start with 0% duty
	TCCR3A |= (1 << COM3B1);    	// Compare Output Mode for channel B
	TIMSK3 &= ~(1 << OCIE3B);    	// Disable Timer 3B output compare match interrupt
									// (no need for an ISR)
	// HOT BED HEATER PWM
	OCR3C = 0;						// Start with 0% duty
	TCCR3A |= (1 << COM3C1);    	// Compare Output Mode for channel B
	TIMSK3 &= ~(1 << OCIE3C);    	// Disable Timer 3C output compare match interrupt
									// (no need for an ISR)
  
	OCR3A = 0;						// Start with 0% duty
	TCCR3A |= (1 << COM3A1);    	// Compare Output Mode for channel A
	TIMSK3 &= ~(1 << OCIE3A);    	// Disable Timer 3A output compare match interrupt
									// (no need for an ISR)
}

//--------------------END Timer and PWM Setup---------------------------

//-------------------- START PID AUTOTUNE ---------------------------
// Based on PID relay test 
// Thanks to Erik van der Zalm for this idea to use it for Marlin
// Some information see:
// http://brettbeauregard.com/blog/2012/01/arduino-pid-autotune-library/
//------------------------------------------------------------------
#ifdef PID_AUTOTUNE
void PID_autotune(int PIDAT_test_temp)
{
  float PIDAT_input = 0;
  int PIDAT_input_help = 0;
  unsigned char PIDAT_count_input = 0;

  float PIDAT_min = 0.0;
  float PIDAT_max = 0.0;
 
  unsigned char PIDAT_PWM_val = HEATER_CURRENT;
  
  unsigned char PIDAT_cycles=0;
  int PIDAT_heating = 1;

  unsigned long PIDAT_temp_millis = millis();
  unsigned long PIDAT_t1=PIDAT_temp_millis;
  unsigned long PIDAT_t2=PIDAT_temp_millis;
  unsigned long PIDAT_T_check_AI_val = PIDAT_temp_millis;

  unsigned char PIDAT_cycle_cnt = 0;
  
  long PIDAT_t_high = 0;
  long PIDAT_t_low = 0;

  long PIDAT_bias= HEATER_CURRENT/2.0;
  long PIDAT_d  =  HEATER_CURRENT/2.0;
  
  float PIDAT_Ku, PIDAT_Tu;
  float PIDAT_Kp, PIDAT_Ki, PIDAT_Kd;
  
  #define PIDAT_TIME_FACTOR ((HEATER_CHECK_INTERVAL*256.0) / 1000.0)
  
  serial_send("PID Autotune start\r\n");

  target_temp = PIDAT_test_temp;
  
  #ifdef BED_USES_THERMISTOR
   WRITE(HEATER_1_PIN,LOW);
  #endif
  
  for(;;) 
  {
 
    if((millis() - PIDAT_T_check_AI_val) > 100 )
    {
      PIDAT_T_check_AI_val = millis();
      PIDAT_cycle_cnt++;
      
      #ifdef HEATER_USES_THERMISTOR
        current_raw = analogRead(TEMP_0_PIN); 
        current_raw = 1023 - current_raw;
        PIDAT_input_help += analog2temp(current_raw);
        PIDAT_count_input++;
      #endif
    }

    // Initialize the min/max values to the first PIDAT_input value we get.
    if (PIDAT_cycle_cnt == 1)
    {
        PIDAT_input = (float)PIDAT_input_help / (float)PIDAT_count_input;
        PIDAT_min = PIDAT_input;
        PIDAT_max = PIDAT_input;
    }
    
    if(PIDAT_cycle_cnt >= 10 )
    {
      
      PIDAT_cycle_cnt = 0;
      
      PIDAT_input = (float)PIDAT_input_help / (float)PIDAT_count_input;
      PIDAT_input_help = 0;
      PIDAT_count_input = 0;
      
      PIDAT_max=fmax(PIDAT_max,PIDAT_input);
      PIDAT_min=fmin(PIDAT_min,PIDAT_input);
      
      if(PIDAT_heating && PIDAT_input > PIDAT_test_temp) 
      {
        if(millis() - PIDAT_t2 > 5000) 
        { 
          PIDAT_heating = 0;
          PIDAT_PWM_val = (PIDAT_bias - PIDAT_d);
          PIDAT_t1 = millis();
          PIDAT_t_high = PIDAT_t1 - PIDAT_t2;
          PIDAT_max = PIDAT_test_temp;
        }
      }
      
      if(!PIDAT_heating && PIDAT_input < PIDAT_test_temp) 
      {
        if(millis() - PIDAT_t1 > 5000) 
        {
          PIDAT_heating = 1;
          PIDAT_t2 = millis();
          PIDAT_t_low = PIDAT_t2 - PIDAT_t1;
          
          if(PIDAT_cycles > 0) 
          {
            PIDAT_bias += (PIDAT_d*(PIDAT_t_high - PIDAT_t_low))/(float)(PIDAT_t_low + PIDAT_t_high);
            if (PIDAT_bias < 20)
            {
              PIDAT_bias = 20;
            }
            if (PIDAT_bias > HEATER_CURRENT - 20)
            {
              PIDAT_bias = HEATER_CURRENT - 20;
            }
            if(PIDAT_bias > (HEATER_CURRENT/2.0)) PIDAT_d = (HEATER_CURRENT - 1) - PIDAT_bias;
            else PIDAT_d = PIDAT_bias;

            serial_send(" bias: %ld d: %ld min: %f max: %f", PIDAT_bias, PIDAT_d, PIDAT_min, PIDAT_max);
            
            if(PIDAT_cycles > 2) 
            {
              PIDAT_Ku = (4.0*PIDAT_d)/(3.14159*(PIDAT_max-PIDAT_min));
              PIDAT_Tu = ((float)(PIDAT_t_low + PIDAT_t_high)/1000.0);
              
              serial_send(" Ku: %f Tu: %f\r\n", PIDAT_Ku, PIDAT_Tu);

              PIDAT_Kp = 0.60*PIDAT_Ku;
              PIDAT_Ki = 2.0*PIDAT_Kp/(float)(PIDAT_Tu);
              PIDAT_Kd = PIDAT_Kp*PIDAT_Tu/8.0;
              serial_send(" Clasic PID \r\n");
              serial_send(" CFG Kp: %d\r\n", (unsigned int)(PIDAT_Kp*256));
              serial_send(" CFG Ki: %d\r\n", (unsigned int)(PIDAT_Ki*PIDAT_TIME_FACTOR));
              serial_send(" CFG Kd: %d\r\n", (unsigned int)(PIDAT_Kd*PIDAT_TIME_FACTOR));
              
              PIDAT_Kp = 0.30*PIDAT_Ku;
              PIDAT_Ki = PIDAT_Kp/(float)PIDAT_Tu;
              PIDAT_Kd = PIDAT_Kp*PIDAT_Tu/3.0;
              serial_send(" Some overshoot \r\n");
              serial_send(" CFG Kp: %d\r\n", (unsigned int)(PIDAT_Kp*256));
              serial_send(" CFG Ki: %d\r\n", (unsigned int)(PIDAT_Ki*PIDAT_TIME_FACTOR));
              serial_send(" CFG Kd: %d\r\n", (unsigned int)(PIDAT_Kd*PIDAT_TIME_FACTOR));
              /*
              PIDAT_Kp = 0.20*PIDAT_Ku;
              PIDAT_Ki = 2*PIDAT_Kp/PIDAT_Tu;
              PIDAT_Kd = PIDAT_Kp*PIDAT_Tu/3;
              serial_send(" No overshoot \r\n");
              serial_send(" CFG Kp: %d\r\n", (unsigned int)(PIDAT_Kp*256));
              serial_send(" CFG Ki: %d\r\n", (unsigned int)(PIDAT_Ki*PIDAT_TIME_FACTOR));
              serial_send(" CFG Kd: %d\r\n", (unsigned int)(PIDAT_Kd*PIDAT_TIME_FACTOR));
              */
            }
          }
          PIDAT_PWM_val = (PIDAT_bias + PIDAT_d);
          PIDAT_cycles++;
          PIDAT_min = PIDAT_test_temp;
        }
      } 

	  setHeaterPWMDuty(HEATER_0_PIN, PIDAT_PWM_val);
    }
    
    if((PIDAT_input > (PIDAT_test_temp + 55)) || (PIDAT_input > 255))
    {
      serial_send("PID Autotune failed! Temperature to high\r\n");
      target_temp = 0;
      return;
    }
    
    if(millis() - PIDAT_temp_millis > 2000) 
    {
      PIDAT_temp_millis = millis();
      serial_send("ok T:%f @:%d\r\n", PIDAT_input, (unsigned char)PIDAT_PWM_val*1);
    }
    
    if(((millis() - PIDAT_t1) + (millis() - PIDAT_t2)) > (10L*60L*1000L*2L)) 
    {
      serial_send("PID Autotune failed! timeout\r\n");
      return;
    }
    
    if(PIDAT_cycles > 5) 
    {
      serial_send("PID Autotune finished ! Place the Kp, Ki and Kd constants in the configuration.h\r\n");
      return;
    }
  }
}
#endif  
//---------------- END AUTOTUNE PID ------------------------------

 void updatePID()
 {
	if (PIDTEMP)
	{
		temp_iState_min = (int)( (256L * -PID_INTEGRAL_DRIVE_MAX) / (float)(PID_Ki) );
		temp_iState_max = (int)( (256L * PID_INTEGRAL_DRIVE_MAX) / (float)(PID_Ki) );
	}

	if (BED_PIDTEMP)
	{
		temp_bed_iState_min = (int)( (256L * -BED_PID_INTEGRAL_DRIVE_MAX) / (float)(bed_PID_Ki) );
		temp_bed_iState_max = (int)( (256L * BED_PID_INTEGRAL_DRIVE_MAX) / (float)(bed_PID_Ki) );
	}
 }
 
 void manage_heater()
 {
  service_TemperatureMonitor();
 
  if( (millis() - previous_millis_heater) < HEATER_CHECK_INTERVAL )
  {
    return;
  }
  
  PreemptionFlag |= 0x0008;

  previous_millis_heater = millis();
  
  #ifdef HEATER_USES_THERMISTOR
    current_raw = analogRead(TEMP_0_PIN); 
    // When using thermistor, when the heater is colder than targer temp, we get a higher analog reading than target, 
    // this switches it up so that the reading appears lower than target for the control logic.
    current_raw = 1023 - current_raw;
  #endif
  
  //MIN / MAX save to display the jitter of Heaterbarrel
  if(current_raw > current_raw_maxval)
    current_raw_maxval = current_raw;
    
  if(current_raw < current_raw_minval)
    current_raw_minval = current_raw;
 
  #ifdef SMOOTHING
    if (!nma) nma = SMOOTHFACTOR * current_raw;
    nma = (nma + current_raw) - ( nma / (float)(SMOOTHFACTOR) );
    current_raw = nma / (float)(SMOOTHFACTOR);
  #endif
  
  #ifdef WATCHPERIOD
    if(watchmillis && millis() - watchmillis > WATCHPERIOD)
    {
        if(watch_raw >= current_raw)
        {
            target_temp = target_raw = 0;
            WRITE(HEATER_0_PIN,LOW);

            setHeaterPWMDuty(HEATER_0_PIN, 0);
        }
        else
        {
            watchmillis = 0;
        }
    }
  #endif
  
  //If tmp is lower then MINTEMP stop the Heater
  //or it os better to deaktivate the uutput PIN or PWM ?
  #ifdef MINTEMP
	minttemp = temp2analogh(MINTEMP);
    if(current_raw <= minttemp)
        target_temp = target_raw = 0;
  #endif
  
  #ifdef MAXTEMP
	maxttemp = temp2analogh(MAXTEMP);
    if(current_raw >= maxttemp)
    {
        target_temp = target_raw = 0;
    }
  #endif

  if (TEMP_0_PIN > -1)
  {
    if (PIDTEMP)
	{	  
      service_ExtruderHeaterPIDControl(analog2temp(current_raw), target_temp);
    }
    else // !PIDTEMP
	{
      service_ExtruderHeaterSimpleControl(current_raw, target_raw);
	} 	
  }		// end if (TEMP_0_PIN > -1)
    
  if(millis() - previous_millis_bed_heater < BED_CHECK_INTERVAL)
    return;
  
  previous_millis_bed_heater = millis();

  #ifndef TEMP_1_PIN
    return;
  #endif
  
  if (TEMP_1_PIN == -1)
  {
    return;
  }
  else
  {  
	//If tmp is lower then MINTEMP stop the Heater
	//or it os better to deaktivate the uutput PIN or PWM ?
	#ifdef MINTEMP
		minttemp = temp2analogBed(MINTEMP);
		if(current_bed_raw <= minttemp)
		{
			target_bed_raw = 0;
		}
	#endif // #ifdef MINTEMP
  
	#ifdef MAXTEMP
		maxttemp = temp2analogBed(MAXTEMP);
		if(current_bed_raw >= maxttemp)
		{
			target_bed_raw = 0;
		}
	#endif // #ifdef MAXTEMP
  
	#ifdef BED_USES_THERMISTOR
		current_bed_raw = analogRead(TEMP_1_PIN);   
	  
		// If using thermistor, when the heater is colder than targer temp, we get a higher analog reading than target, 
		// this switches it up so that the reading appears lower than target for the control logic.
		current_bed_raw = 1023 - current_bed_raw;
	#endif // #ifdef BED_USES_THERMISTOR
	  
	// PID Control for HOT BED
	if (BED_PIDTEMP > -1)
	{		
		service_BedHeaterPIDControl(analog2tempBed(current_bed_raw), 
												analog2tempBed(target_bed_raw));
	} 
	else // #if not (BED_PIDTEMP > -1)
	{
		service_BedHeaterSimpleControl(current_bed_raw, target_bed_raw);
	} 	// #ifdef BED_PIDTEMP
  }		//end if (TEMP_1_PIN == -1)
}


#if defined (HEATER_USES_THERMISTOR) || defined (BED_USES_THERMISTOR)
int temp2analog_thermistor(int celsius, const short table[][2], int numtemps) 
{
    int raw = 0;
    unsigned char i;
    
    for (i=1; i<numtemps; i++)
    {
      if (table[i][1] < celsius)
      {
        raw = table[i-1][0] + 
          (celsius - table[i-1][1]) * 
          (table[i][0] - table[i-1][0]) /
          (float)(table[i][1] - table[i-1][1]);
      
        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == numtemps) raw = table[i-1][0];

    return 1023 - raw;
}
#endif

#if defined (HEATER_USES_THERMISTOR) || defined (BED_USES_THERMISTOR)
int analog2temp_thermistor(int raw,const short table[][2], int numtemps) {
    int celsius = 0;
    unsigned char i;
    
    raw = 1023 - raw;

    for (i=1; i<numtemps; i++)
    {
      if (table[i][0] > raw)
      {
        celsius  = table[i-1][1] + 
          (raw - table[i-1][0]) * 
          (table[i][1] - table[i-1][1]) /
          (float)(table[i][0] - table[i-1][0]);

        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == numtemps) celsius = table[i-1][1];

    return celsius;
}
#endif


void service_ExtruderHeaterPIDControl(int current_temp, int target_temp)
{
	error = target_temp - current_temp;
	int delta_temp = current_temp - prev_temp;

	prev_temp = current_temp;
	pTerm = ((long)PID_Kp * error) / 256.0;
	//int H0 = MIN(HEATER_DUTY_FOR_SETPOINT(target_temp),HEATER_CURRENT);
	//heater_duty = H0 + pTerm;
	heater_duty = pTerm;

	if(error < 30)
	{
		temp_iState += error;
		if (temp_iState < temp_iState_min)
		{
			temp_iState = temp_iState_min;
		}
		
		if (temp_iState > temp_iState_max)
		{
			temp_iState = temp_iState_max;
		}
	
		iTerm = ((long)PID_Ki * temp_iState) / 256.0;
		heater_duty += iTerm;
	}

	int prev_error = abs(target_temp - prev_temp);
	int log3 = 1; // discrete logarithm base 3, plus 1

	if(prev_error > 81){ prev_error /= 81.0; log3 += 4; }
	if(prev_error >  9){ prev_error /= 9.0; log3 += 2; }
	if(prev_error >  3){ log3 ++; }

	dTerm = ((long)PID_Kd * delta_temp) / (float)(256.0*log3);
	heater_duty += dTerm;
	
	if (heater_duty < 0)
	{
		heater_duty = 0;
	}
	
	if (heater_duty > HEATER_CURRENT)
	{
		heater_duty = HEATER_CURRENT;
	}

	if(target_temp != 0)
	{
		setHeaterPWMDuty(HEATER_0_PIN, heater_duty);
	}
	else
	{
		setHeaterPWMDuty(HEATER_0_PIN, 0);
	}
}


void service_ExtruderHeaterSimpleControl(int current_raw, int target_raw)
{
	if(current_raw >= target_raw)
    {
		WRITE(HEATER_0_PIN,LOW);
    }
    else 
    {
		if(target_raw != 0)
        {
			WRITE(HEATER_0_PIN,HIGH);
        }
    }
}


void service_BedHeaterPIDControl(int current_bed_temp, int target_bed_temp)
{
	bed_error = target_bed_temp - current_bed_temp;
	int delta_bed_temp = current_bed_temp - prev_bed_temp;
	  
	prev_bed_temp = current_bed_temp;
	bed_pTerm = ((long)bed_PID_Kp * bed_error) / 256.0;
	int H0 = MIN(HEATER_DUTY_FOR_SETPOINT(target_temp),HEATER_CURRENT);
	bed_heater_duty = H0 + bed_pTerm;
  
	if(bed_error < 30)
	{
		temp_bed_iState += bed_error;
		
		if (temp_bed_iState < temp_bed_iState_min)
		{
		  temp_bed_iState = temp_bed_iState_min;
		}
		
		if (temp_bed_iState > temp_bed_iState_max)
		{
		  temp_bed_iState = temp_bed_iState_max;
		}
		  
		bed_iTerm = ((long)bed_PID_Ki * temp_bed_iState) / 256.0;
		bed_heater_duty += bed_iTerm;
	}
	  
	int prev_bed_error = abs(target_bed_temp - prev_bed_temp);
	int log3 = 1; // discrete logarithm base 3, plus 1
	  
	if(prev_bed_error > 81){ prev_bed_error /= 81.0; log3 += 4; }
	if(prev_bed_error >  9){ prev_bed_error /=  9.0; log3 += 2; }
	if(prev_bed_error >  3){ log3 ++; }
	  
	bed_dTerm = ((long)bed_PID_Kd * delta_bed_temp) / (float)(256.0*log3);
	bed_heater_duty += bed_dTerm;
	
	if (bed_heater_duty < 0)
	{
		bed_heater_duty = 0;
	}
	
	if (bed_heater_duty > BED_HEATER_CURRENT)
	{
		bed_heater_duty = BED_HEATER_CURRENT;
	}

	if(target_bed_temp != 0)
	{
		setHeaterPWMDuty(HEATER_1_PIN, bed_heater_duty);
	}
	else
	{
		setHeaterPWMDuty(HEATER_1_PIN, 0);
	}
}


void service_BedHeaterSimpleControl(int current_bed_raw, int target_bed_raw)
{
	#ifdef MINTEMP
		if(current_bed_raw >= target_bed_raw || current_bed_raw < minttemp)
	#else
		if(current_bed_raw >= target_bed_raw)
	#endif // #ifdef MINTEMP
		{
			WRITE(HEATER_1_PIN,LOW);
		}
		else 
		{
			WRITE(HEATER_1_PIN,HIGH);
		}
}


void service_TemperatureMonitor(void)
{
  //Temperature Monitor for repetier
  if((millis() - previous_millis_monitor) > 250 )
  {
    previous_millis_monitor = millis();

    if(manage_monitor <= 1)
    {
      serial_send("MTEMP:%lu", millis());
      if(manage_monitor<1)
      {
        serial_send(" %d %d ", analog2temp(current_raw), target_temp);
        if (PIDTEMP > -1)
		{
          serial_send("%d\r\n", heater_duty);
		}
        else
		{
			if (HEATER_0_PIN > -1)
			{
				if(READ(HEATER_0_PIN))
				{
					serial_send("255\r\n");
				}
				else
				{
					serial_send("0\r\n");
				}
			}
			else
			{
				serial_send("0\r\n");
			}
        }
      }
      else  // if not (manage_monitor<1)
      {
        serial_send(" %d %d ", analog2tempBed(current_bed_raw), 
											analog2tempBed(target_bed_raw));
        if (HEATER_1_PIN > -1)
		{
			if(READ(HEATER_1_PIN))
            {
				serial_send("255\r\n");
			}
			else
            {
				serial_send("0\r\n");
			}
        }
		else
		{
			serial_send("0\r\n");
        } 
      }		// end if(manage_monitor<1)
    }	// end if(manage_monitor <= 1)
  
  }	// end if((millis() - previous_millis_monitor) > 250 )
	// END Temperature Monitor for repetier
}


/***************************************************
* setHeaterPWMDuty(uint8_t pin, int val)
*
* pin: 	Heater pin to set PWM duty for
* val: 	Value Between 0 and ICR3 (250 at present)
*		val = 125 -> 50% duty
*
* Sets the PWM duty of selected heater
****************************************************/
void setHeaterPWMDuty(uint8_t pin, int val)
{
	if (val < 0)
	{
		val = 0;
	}
	
	if (val > ICR3)
	{
		val = ICR3;
	}
	
	switch (pin)
	{
		case HEATER_0_PIN:	// Extruder Heater
		{
			OCR3B = val;
			TCCR3A |= (1<<COM3B1);
			break;
		}
		
		case HEATER_1_PIN:	// Hot-Bed Heater
		{
			OCR3C = val;
			TCCR3A |= (1<<COM3C1);
			break;
		}
		
		default:
		{
			break;
		}
	}
}


/***************************************************
* setFanPWMDuty(int val)
*
* val: 	Value Between 0 and ICR3 (250 at present)
*		val = 125 -> 50% duty
*
* Sets the PWM duty of the fan.
****************************************************/
void setFanPWMDuty(int val)
{
	if (val < 0)
	{
		val = 0;
	}
	
	if (val > ICR3)
	{
		val = ICR3;
	}
	
	OCR3A = val;
	TCCR3A |= (1<<COM3A1);
}
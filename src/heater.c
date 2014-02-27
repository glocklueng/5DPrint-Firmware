/*
  5D Print Firmware
  Based on Sprinter (master branch, 1 Sep 2012).
  Designed for Printrboard (Rev B) and 5D Print D8 Dirver Board.
  ---
  Copyright (c) 2012-2014 by Makible Limited.
 
  This file is part of the 5D Print Firmware.
 
  5D Print Firmware is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  The 5D Print Firmware is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with the 5D Print Firmware.  If not, see <http://www.gnu.org/licenses/>.
*/
/**
   \file heater.c
   \brief Heater control, using PWM 
  
 */

#include <avr/interrupt.h>
#include <math.h>
#include <stdlib.h>
#include "config.h"
#include "heater_table.h"
#include "heater.h"
#include "board_io.h"
#include "pins.h"
#include "pins_teensy.h"
//#include "5dprint.h"
#include "usb.h"
#include "pgmspace.h"
#include "language.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))


// Manage heater variables. For a thermistor, raw values refer to the 
// reading from the analog pin. 
int target_raw = 0;
int target_temp = 0;
int current_raw = 0;
int current_raw_maxval = -32000;
int current_raw_minval = 32000;
int target_bed_raw = 0;
int current_bed_raw = 0;
unsigned long previous_millis_heater, previous_millis_bed_heater;
unsigned long previous_millis_monitor = 0, previous_millis_periodic_temp_report = 0;

#ifdef PIDTEMP
int temp_iState = 0;
int temp_dState = 0;
int prev_temp = 0;
int pTerm;
int iTerm;
int dTerm;
int error;
int heater_duty = 0;
int max_heater_duty = HEATER_CURRENT, user_max_heater_duty = HEATER_CURRENT;
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
unsigned short max_bed_heater_duty = BED_HEATER_CURRENT_BEFORE_FULL_PWR;
unsigned short user_max_bed_heater_duty = BED_HEATER_CURRENT_FULL_PWR;
unsigned short user_max_bed_heater_duty_before_full_pwr = BED_HEATER_CURRENT_BEFORE_FULL_PWR;
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
int watch_temp = 0;
unsigned long watchmillis = 0;
#endif

#ifdef MINTEMP
int minttemp; /* = temp2analogh(MINTEMP); */
#endif

#ifdef MAXTEMP
int maxttemp; /* = temp2analogh(MAXTEMP); */
#endif

unsigned short periodic_temp_report = 0;

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
// Setup Timers and PWM for Heater and FAN
//------------------------------------------------------------------------

/**
   \fn ISR(TIMER1_COMPC_vect)
   \brief 
 */
ISR(TIMER1_COMPC_vect)
{
    manage_heater();
}

/**
   \fn void init_Timer3_HW_pwm(void)
   \brief
   This is hardware PWM with 500 Hz for Extruder Heating and Fan
   We want to have at least 500Hz - equivalent to previous SOFT_PWM
 */
void init_Timer3_HW_pwm(void)
{
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
    TCCR3A |= (1 << COM3C1);    	// Compare Output Mode for channel C
    TIMSK3 &= ~(1 << OCIE3C);    	// Disable Timer 3C output compare match interrupt
    // (no need for an ISR)
  
    OCR3A = 0;						// Start with 0% duty
    TCCR3A |= (1 << COM3A1);    	// Compare Output Mode for channel A
    TIMSK3 &= ~(1 << OCIE3A);    	// Disable Timer 3A output compare match interrupt
    // (no need for an ISR)
}

//--------------------END Timer and PWM Setup---------------------------

//-------------------- START PID AUTOTUNE ---------------------------
#ifdef PID_AUTOTUNE
/**
   \fn void PID_autotune(int PIDAT_test_temp)
   \brief 
   Based on PID relay test 
   Thanks to Erik van der Zalm for this idea to use it for Marlin
   Some information see:
   http://brettbeauregard.com/blog/2012/01/arduino-pid-autotune-library/
 */
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
  
    serial_send(TXT_PID_AUTOTUNE_START_CRLF);

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

                                            serial_send(TXT_BIAS_MIN_MAX, PIDAT_bias, PIDAT_d, PIDAT_min, PIDAT_max);
            
                                            if(PIDAT_cycles > 2) 
                                                {
                                                    PIDAT_Ku = (4.0*PIDAT_d)/(3.14159*(PIDAT_max-PIDAT_min));
                                                    PIDAT_Tu = ((float)(PIDAT_t_low + PIDAT_t_high)/1000.0);
              
                                                    serial_send(TXT_KU_TU_CRLF, PIDAT_Ku, PIDAT_Tu);

                                                    PIDAT_Kp = 0.60*PIDAT_Ku;
                                                    PIDAT_Ki = 2.0*PIDAT_Kp/(float)(PIDAT_Tu);
                                                    PIDAT_Kd = PIDAT_Kp*PIDAT_Tu/8.0;
                                                    serial_send(TXT_CLASSIC_PID_CRLF);
                                                    serial_send(TXT_CFG_KP_CRLF, (unsigned int)(PIDAT_Kp*256));
                                                    serial_send(TXT_CFG_KI_CRLF, (unsigned int)(PIDAT_Ki*PIDAT_TIME_FACTOR));
                                                    serial_send(TXT_CFG_KD_CRLF, (unsigned int)(PIDAT_Kd*PIDAT_TIME_FACTOR));
              
                                                    PIDAT_Kp = 0.30*PIDAT_Ku;
                                                    PIDAT_Ki = PIDAT_Kp/(float)PIDAT_Tu;
                                                    PIDAT_Kd = PIDAT_Kp*PIDAT_Tu/3.0;
                                                    serial_send(TXT_SOME_OVERSHOOT_CRLF);
                                                    serial_send(TXT_CFG_KP_CRLF, (unsigned int)(PIDAT_Kp*256));
                                                    serial_send(TXT_CFG_KI_CRLF, (unsigned int)(PIDAT_Ki*PIDAT_TIME_FACTOR));
                                                    serial_send(TXT_CFG_KD_CRLF, (unsigned int)(PIDAT_Kd*PIDAT_TIME_FACTOR));
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
                    serial_send(TXT_PID_AUTOTUNE_FAILED_TEMP_HIGH_CRLF);
                    target_temp = 0;
                    return;
                }
    
            if(millis() - PIDAT_temp_millis > 2000) 
                {
                    PIDAT_temp_millis = millis();
                    serial_send(TXT_OK_T_AT_DUTY_CRLF, PIDAT_input, (unsigned char)PIDAT_PWM_val*1);
                }
    
            if(((millis() - PIDAT_t1) + (millis() - PIDAT_t2)) > (10L*60L*1000L*2L)) 
                {
                    serial_send(TXT_PID_AUTOTUNE_FAILED_TIMEOUT_CRLF);
                    return;
                }
    
            if(PIDAT_cycles > 5) 
                {
                    serial_send(TXT_PID_AUTOTUNE_FINISHED_CRLF);
                    return;
                }
        }
}
#endif  
//---------------- END AUTOTUNE PID ------------------------------

/**
   \fn void updatePID()
   \brief 
 */
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
/**
   \fn void manage_heater()
   \brief Called in the interrupt service routine
 */
void manage_heater()
{
    int current_temp = 0;
 
    service_TemperatureMonitor();
 
    if ( (TEMP_0_PIN > -1) 
         && (millis() - previous_millis_heater >= HEATER_CHECK_INTERVAL) )
        {
#if (DEBUG > -1)
            PreemptionFlag |= 0x0008;
#endif
	
            previous_millis_heater = millis();
  
#ifdef HEATER_USES_THERMISTOR
            current_raw = analogRead(TEMP_0_PIN); 
            // When using thermistor, when the heater is colder than targer temp, we get a higher analog reading than target, 
            // this switches it up so that the reading appears lower than target for the control logic.
            current_raw = 1023 - current_raw;
	
            current_temp = analog2temp(current_raw);
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
            if ( (watchmillis > 0) && (millis() - watchmillis > WATCHPERIOD) )
                {
                    if( watch_temp >= current_temp )
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
            //minttemp = temp2analogh(MINTEMP);
            if(current_temp <= MINTEMP)
		target_temp = target_raw = 0;
#endif
  
#ifdef MAXTEMP
            //maxttemp = temp2analogh(MAXTEMP);
            if(current_temp >= MAXTEMP)
                {
                    target_temp = target_raw = 0;
                }
#endif

	
            if (PIDTEMP)
                {
                    // Only allow extruder heater to be turned on if bed temperature has reached 
                    // 80% of target. This is to try and limit the max power drawn from the power 
                    // supply.
                    if ( ( analog2tempBed(current_bed_raw) > MIN_BED_TEMP_FOR_HOTEND_FULL_PWR )
                         && ( analog2tempBed(target_bed_raw) > BEDMINTEMP ) )
                        {
                            max_heater_duty = user_max_heater_duty;
                        }
                    else if ( analog2tempBed(target_bed_raw) < BEDMINTEMP )
                        {
                            max_heater_duty = user_max_heater_duty;
                        }
                    else
                        {
                            max_heater_duty = user_max_heater_duty / 3.0;
                        }
	  
                    service_ExtruderHeaterPIDControl(current_temp, target_temp);
                }
            else // !PIDTEMP
                {
                    service_ExtruderHeaterSimpleControl(current_raw, target_raw);
                }
        }		// end if ( (TEMP_0_PIN > -1)
    //	&& (millis() - previous_millis_heater >= HEATER_CHECK_INTERVAL) )
    
  
    if ( (TEMP_1_PIN > -1) 
         && (millis() - previous_millis_bed_heater >= BED_CHECK_INTERVAL) )
        {
#if (DEBUG > -1)
            PreemptionFlag |= 0x0010;
#endif
	
            previous_millis_bed_heater = millis();
	
            //If tmp is lower then MINTEMP stop the Heater
            //or it os better to deaktivate the uutput PIN or PWM ?
#ifdef BEDMINTEMP
            minttemp = temp2analogBed(BEDMINTEMP);
            if(current_bed_raw <= minttemp)
		{
                    target_bed_raw = 0;
		}
#endif // #ifdef MINTEMP
  
#ifdef BEDMAXTEMP
            maxttemp = temp2analogBed(BEDMAXTEMP);
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
            if (BED_PIDTEMP > -1){
                    // Only allow bed heater to be run at full power if its temperature is 
                    // greater than MIN_BED_TEMP_FOR_HOTBED_FULL_PWR. This is to limit the 
                    // max current drawn by the bed
                    if ( ( analog2tempBed(current_bed_raw) > MIN_BED_TEMP_FOR_HOTBED_FULL_PWR )
                         && ( analog2tempBed(target_bed_raw) > BEDMINTEMP ) )
                            max_bed_heater_duty = user_max_bed_heater_duty;
                    else if ( analog2tempBed(target_bed_raw) < BEDMINTEMP )
                            max_bed_heater_duty = user_max_bed_heater_duty_before_full_pwr;
                    else max_bed_heater_duty = user_max_bed_heater_duty_before_full_pwr;
		
                    service_BedHeaterPIDControl(analog2tempBed(current_bed_raw), 
                                                analog2tempBed(target_bed_raw));
            } 
            else service_BedHeaterSimpleControl(current_bed_raw, target_bed_raw);
        }		//end if (TEMP_1_PIN == -1)
  
  
    // Safety check to try and catch case when bed and extruder heater connectors 
    // have been swapped around accidentally.
    if ( ( analog2tempBed(current_bed_raw) >= MAXTEMP ) 
         || ( current_temp >= MAXTEMP ) )
        {
            if ( (target_bed_raw != 0) || (target_temp != 0) )
                {
                    target_bed_raw = 0;				// Switch off bed heater
                    target_temp = target_raw = 0;	// Switch off extruder heater
                    serial_send(TXT_ALL_HEATERS_DISABLED_SAFETY_TEMP_EXCEEDED_CRLF);
                    serial_send(TXT_CHECK_HEATER_AND_THERMISTOR_CONNS_ARE_CORRECT_CRLF);
                }
        }
}


#if defined (HEATER_USES_THERMISTOR) || defined (BED_USES_THERMISTOR)
/**
   \fn int temp2analog_thermistor(int celsius, const short table[][2], int numtemps) 
   \brief 
 */
int temp2analog_thermistor(int celsius, const short table[][2], int numtemps) 
{
    int raw = 0;
    unsigned char i;
    
    for (i=1; i<numtemps; i++)
        {
            if ( table[i][1] < celsius )
                {
                    raw = table[i-1][0] + 
                        ( celsius - table[i-1][1] ) * 
                        ( table[i][0] - table[i-1][0] ) /
                        (float)( table[i][1] - table[i-1][1] );
      
                    break;
                }
        }

    // Overflow: Set to last value in the table
    if (i == numtemps) raw = table[i-1][0];

    return 1023 - raw;
}
#endif

#if defined (HEATER_USES_THERMISTOR) || defined (BED_USES_THERMISTOR)
/**
   \fn int analog2temp_thermistor(int raw,const short table[][2], int numtemps) {
   \brief 
 */
int analog2temp_thermistor(int raw,const short table[][2], int numtemps) {
    int celsius = 0;
    unsigned char i;
    
    raw = 1023 - raw;

    for (i=1; i<numtemps; i++)
        {
            if ( table[i][0] > raw )
                {
                    celsius  = table[i-1][1] + 
                        ( raw - table[i-1][0] ) * 
                        ( table[i][1] - table[i-1][1] ) /
                        (float)( table[i][0] - table[i-1][0] );

                    break;
                }
        }

    // Overflow: Set to last value in the table
    if (i == numtemps) celsius = table[i-1][1];

    return celsius;
}
#endif

/**
   \fn void service_ExtruderHeaterPIDControl(int current_temp, int target_temp)
   \brief 
 */
void service_ExtruderHeaterPIDControl(int current_temp, int target_temp)
{
    error = target_temp - current_temp;
    int delta_temp = current_temp - prev_temp;

    prev_temp = current_temp;
    pTerm = ((long)PID_Kp * error) >> 8;
    heater_duty = pTerm;

    if( abs(error) < PID_FUNCTIONAL_RANGE )
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
	
            iTerm = ((long)PID_Ki * temp_iState) >> 8;
            heater_duty += iTerm;
		
            dTerm = ((long)PID_Kd * delta_temp) >> 8;
            heater_duty -= dTerm;
	}
    else
	{
            temp_iState = 0;
	}

	
    if (heater_duty < 0)
	{
            heater_duty = 0;
	}
	
    if (heater_duty > max_heater_duty)
	{
            heater_duty = max_heater_duty;
	}

    if(target_temp != 0)
	{
            setHeaterPWMDuty(HEATER_0_PIN, heater_duty);
	}
    else
	{
            heater_duty = 0;
            setHeaterPWMDuty(HEATER_0_PIN, heater_duty);
	}
}

/**
   \fn void service_ExtruderHeaterSimpleControl(int current_raw, int target_raw)
   \brief 
 */
void service_ExtruderHeaterSimpleControl(int current_raw, int target_raw)
{
    if(current_raw >= target_raw)
        {
            WRITE(HEATER_0_PIN,LOW);
            heater_duty = 0;
        }
    else 
        {
            if(target_raw != 0)
                {
                    WRITE(HEATER_0_PIN,HIGH);
                    heater_duty = 100;
                }
            else
		{
                    WRITE(HEATER_0_PIN,LOW);
                    heater_duty = 0;
		}
        }
}

/**
   \fn void service_BedHeaterPIDControl(int current_bed_temp, int target_bed_temp)
   \brief 
 */
void service_BedHeaterPIDControl(int current_bed_temp, int target_bed_temp)
{
    bed_error = target_bed_temp - current_bed_temp;
    int delta_bed_temp = current_bed_temp - prev_bed_temp;
	  
    prev_bed_temp = current_bed_temp;
    bed_pTerm = ((long)bed_PID_Kp * bed_error) >> 8;
    bed_heater_duty = bed_pTerm;
  
    if( abs(bed_error) < BED_PID_FUNCTIONAL_RANGE )
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
		  
            bed_iTerm = ((long)bed_PID_Ki * temp_bed_iState) >> 8;
            bed_heater_duty += bed_iTerm;
		
            bed_dTerm = ((long)bed_PID_Kd * delta_bed_temp) >> 8;
            bed_heater_duty -= bed_dTerm;
	}
    else
	{
            temp_bed_iState = 0;
	}
	  
	
    if (bed_heater_duty < 0)
	{
            bed_heater_duty = 0;
	}
	
    if (bed_heater_duty > max_bed_heater_duty)
	{
            bed_heater_duty = max_bed_heater_duty;
	}

    if(target_bed_temp != 0)
	{
            setHeaterPWMDuty(HEATER_1_PIN, bed_heater_duty);
	}
    else
	{
            bed_heater_duty = 0;
            setHeaterPWMDuty(HEATER_1_PIN, bed_heater_duty);
	}
}

/**
   \fn void service_BedHeaterSimpleControl(int current_bed_raw, int target_bed_raw)
   \brief 
 */
void service_BedHeaterSimpleControl(int current_bed_raw, int target_bed_raw)
{
#ifdef MINTEMP
    if(current_bed_raw >= target_bed_raw || current_bed_raw < minttemp)
#else
        if(current_bed_raw >= target_bed_raw)
#endif // #ifdef MINTEMP
            {
                WRITE(HEATER_1_PIN,LOW);
                bed_heater_duty = 0;
            }
        else 
            {
                WRITE(HEATER_1_PIN,HIGH);
                bed_heater_duty = 100;
            }
}

/**
   \fn void service_TemperatureMonitor(void)
   \brief 
 */
void service_TemperatureMonitor(void)
{
    int hotendtC = 0, bedtempC = 0;
  
    //Temperature Monitor for repetier
    if((millis() - previous_millis_monitor) > 250 )
        {
            previous_millis_monitor = millis();

            if(manage_monitor <= 1)
                {
                    serial_send(TXT_MTEMP, millis());
                    if(manage_monitor<1)
                        {
                            serial_send(TXT_INT_INT, analog2temp(current_raw), target_temp);
                            if (PIDTEMP > -1)
                                {
                                    serial_send(TXT_INT_CRLF, heater_duty);
                                }
                            else
                                {
                                    if (HEATER_0_PIN > -1)
                                        {
                                            if(READ(HEATER_0_PIN))
                                                {
                                                    serial_send(TXT_255_CRLF);
                                                }
                                            else
                                                {
                                                    serial_send(TXT_0_CRLF);
                                                }
                                        }
                                    else
                                        {
                                            serial_send(TXT_0_CRLF);
                                        }
                                }
                        }
                    else  // if not (manage_monitor<1)
                        {
                            serial_send(TXT_INT_INT, analog2tempBed(current_bed_raw), 
                                        analog2tempBed(target_bed_raw));
                            if (HEATER_1_PIN > -1)
                                {
                                    if(READ(HEATER_1_PIN))
                                        {
                                            serial_send(TXT_255_CRLF);
                                        }
                                    else
                                        {
                                            serial_send(TXT_0_CRLF);
                                        }
                                }
                            else
                                {
                                    serial_send(TXT_0_CRLF);
                                } 
                        }		// end if(manage_monitor<1)
                }	// end if(manage_monitor <= 1)
  
        }	// end if((millis() - previous_millis_monitor) > 250 )
    // END Temperature Monitor for repetier
	
	
    if (periodic_temp_report)
        {
            if( (millis() - previous_millis_periodic_temp_report) > (periodic_temp_report * 1000) )
                {
                    previous_millis_periodic_temp_report = millis();
		
#if (TEMP_0_PIN > -1)
                    hotendtC = analog2temp(current_raw);
#endif
#if TEMP_1_PIN > -1
                    bedtempC = analog2tempBed(current_bed_raw);
#endif
		
                    serial_send(TXT_DASH_DASH_T_INT, hotendtC);
#ifdef PIDTEMP
                    serial_send(TXT_D_INT_PERCENT, (int)( (heater_duty * 100) / (float)(HEATER_CURRENT) ));
#endif
#if TEMP_1_PIN > -1
                    serial_send(TXT_B_INT, bedtempC);
#endif
#ifdef BED_PIDTEMP
                    serial_send(TXT_D_INT_PERCENT, (int)( (bed_heater_duty * 100) / (float)(BED_HEATER_CURRENT) ));
#endif
                    serial_send(TXT_CRLF);
                } // if( (millis() - previous_millis_periodic_temp_report) > (periodic_temp_report * 1000) )
        } // if (periodic_temp_report)
}


/**
 \fn void setHeaterPWMDuty(uint8_t pin, int val)
 \brief  Sets the PWM duty of selected heater
 \param pin Heater pin to set PWM duty for
 \param val Value Between 0 and ICR3 (250 at present), val = 125 -> 50% duty
 */
void setHeaterPWMDuty(uint8_t pin, int val)
{
    if (val < 0)
	{
            val = 0;
	}
	
    if ( (unsigned int)val > ICR3)
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

/**
   \fn void setFanPWMDuty(int val)
   \brief Sets the PWM duty of the fan.
   \param  val Value Between 0 and ICR3 (250 at present), val = 125 -> 50% duty
*/
void setFanPWMDuty(int val)
{
    if (val < 0)
	{
            val = 0;
	}
	
    if ( (unsigned int)val > ICR3)
	{
            val = ICR3;
	}
	
    OCR3A = val;
    TCCR3A |= (1<<COM3A1);
}

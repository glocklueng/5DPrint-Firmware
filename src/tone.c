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
   \file tone.c
   \brief Generates tone with a square wave, using Timer 2A
*/

#include <avr/interrupt.h>
#include <stdlib.h>

#include "config.h"
#include "board_io.h"
#include "pins.h"
#include "pins_teensy.h"
#include "tone.h"

//------------------------------------------------------------------------
// Variable Declarations
// For varaibles used outside this module.
//------------------------------------------------------------------------
unsigned short BUZZER_F = 0;
unsigned short BUZZER_P = 0; 
unsigned char  BUZZER_ON = 0;

//------------------------------------------------------------------------
// Variable Declarations
// For variables used within this module only.
//------------------------------------------------------------------------
unsigned long previous_millis_buzzer = 0;

//------------------------------------------------------------------------
// Function Prototypes
// For functions used within this module only.
//------------------------------------------------------------------------
void setBuzzerFrequency(void);

#if BUZZER_SUPPORT > 0
/**
   \fn void buzzer_init(void)
   \brief Buzzer initialization code
 */
void buzzer_init(void){
    SET_OUTPUT(BUZZER_PIN);
    WRITE(BUZZER_PIN, LOW);

	TIFR2 = (1 << TOV2);       // clear interrupt flag
	
	// Timer (ck/1024 prescalar) => 16MHz / 256 = 62.5kHz
	TCCR2B = (1 << CS22) | (1 << CS21);
    TCCR2B &= ~(1 << CS20);
	
	TCCR2A = (1 << WGM21);		// CTC Mode: Compare match counter is cleared
								// on compare.
  
	OCR2A = 0;					// 0 = Max Frequency
								// Freq = ck / 2 * prescaler * (1 + OCR2A)
	TIMSK2 &= ~(1 << OCIE2A);   // disable timer2 output compare match interrupt

    // output mode = 00 (disconnected)
    TCCR2A &= ~(3<<COM2A0); 
    TCCR2A &= ~(3<<COM2B0); 

    sei();
}

/**
   \fn void buzzer_tone(void)
   \brief Starts a tone when M300 command is received
 */
void buzzer_tone(void){
    setBuzzerFrequency();
    
    if (BUZZER_P > MAX_BUZZER_PERIOD) BUZZER_P = MAX_BUZZER_PERIOD;
    else if (BUZZER_P < MIN_BUZZER_PERIOD) BUZZER_P = MIN_BUZZER_PERIOD;
    
    previous_millis_buzzer = millis();
    BUZZER_ON = 1;
    ENABLE_BUZZER();
}

/**
   \fn void setBuzzerFrequency(void)
   \brief Sets the OCR2A value based on the input frequency
 */
void setBuzzerFrequency(void){
    int val;

    if (BUZZER_F > TIMER2A_CLOCK_FREQ) BUZZER_F = TIMER2A_CLOCK_FREQ;
    else if (BUZZER_F < 0) BUZZER_F = 0;
    
    val = (int) (BUZZER_F / TIMER2A_CLOCK_FREQ * 255);
    //serial_send ("buzzer val: %i \n", val);
    val = 255 - val;
    //serial_send ("final buzzer val: %i \n", val);
    OCR2A = val;
}

ISR(TIMER2_COMPA_vect){
    if (BUZZER_ON > 0){
        if (millis() - previous_millis_buzzer <= BUZZER_P &&
            millis() - previous_millis_buzzer <= BUZZER_TIMEOUT_PERIOD){
            TOGGLE(BUZZER_PIN);
        }
        else{
            BUZZER_ON = 0;
            WRITE(BUZZER_PIN, LOW);
            DISABLE_BUZZER();
        }
    }
}

#endif

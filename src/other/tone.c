/*
  5DPrint Firmware
  Based on Sprinter (master branch, 1 Sep 2012).
  Designed for Printrboard (Rev B) and 5DPrint D8 Controller Board.
  ---
  Copyright (c) 2012-2014 by Makible Limited.
 
  This file is part of the 5DPrint Firmware.
 
  5DPrint Firmware is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  The 5DPrint Firmware is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with the 5DPrint Firmware.  If not, see <http://www.gnu.org/licenses/>.
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
#include "usb.h" 

// The buzzer on 5DPD8 is rated at 4kHz.
// Frequency higher than that sounds rough, therefore upper limit is set to 4 kHz
// Lower frequency tones are alright, lower limit is avoid overflowing the OCR2A register
#define MAX_BUZZER_FREQUENCY      5000         // Hz
#define MIN_BUZZER_FREQUENCY      123          // Hz

#define MAX_BUZZER_PERIOD         2000         // ms
#define MIN_BUZZER_PERIOD         0            // ms

#define TIMER2A_CLOCK_FREQ        31250.0       // Hz

#define ENABLE_BUZZER() TIMSK2 |= (1<<OCIE2A)
#define DISABLE_BUZZER() TIMSK2 &= ~(1<<OCIE2A)

//------------------------------------------------------------------------
// Variable Declarations
// For varaibles used outside this module.
//------------------------------------------------------------------------
short int BUZZER_F = 0;
short int BUZZER_P = 0; 

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
	
	// Timer (ck/256 prescalar) => 16MHz / 256 = 62.5kHz
	TCCR2B = (1 << CS22) | (1 << CS21);
    TCCR2B &= ~(1 << CS20);
	
	TCCR2A = (1 << WGM21);		// CTC Mode: Compare match counter is cleared
    TCCR2A &= ~(1 << WGM22);    // on compare.
    TCCR2A &= ~(1 << WGM20); 
  
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
    if (BUZZER_P > MAX_BUZZER_PERIOD) BUZZER_P = MAX_BUZZER_PERIOD;
    else if (BUZZER_P <= MIN_BUZZER_PERIOD) return;

    setBuzzerFrequency(); // Set top value
    TCNT2 = 0; // Reset counter
    
    previous_millis_buzzer = millis();
    ENABLE_BUZZER();
}

/**
   \fn void setBuzzerFrequency(void)
   \brief Sets the OCR2A value based on the input frequency
 */
void setBuzzerFrequency(void){
    if (BUZZER_F > MAX_BUZZER_FREQUENCY) BUZZER_F = MAX_BUZZER_FREQUENCY;
    else if (BUZZER_F < MIN_BUZZER_FREQUENCY) BUZZER_F = MIN_BUZZER_FREQUENCY;
    OCR2A = (int) (TIMER2A_CLOCK_FREQ / BUZZER_F);
}

ISR(TIMER2_COMPA_vect){
    if (millis() - previous_millis_buzzer <= BUZZER_P) TOGGLE(BUZZER_PIN);
    else{
        WRITE(BUZZER_PIN, LOW);
        DISABLE_BUZZER();
    }
}

#endif

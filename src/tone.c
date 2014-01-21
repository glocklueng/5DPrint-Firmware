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

#include "config.h"
#include <stdlib.h>

#include "board_io.h"
#include "pins.h"
#include "pins_teensy.h"
#include "tone.h"

#if BUZZER_SUPPORT > 0
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
unsigned char buzzer_status = 0;

//------------------------------------------------------------------------
// Function Prototypes
// For functions used within this module only.
//------------------------------------------------------------------------
void setBuzzerPWMDuty(void);


/***************************************************
 * buzzer_init(void)
 *
 * Initial set up for buzzer
 ****************************************************/
void buzzer_init(void){
    SET_OUTPUT(BUZZER_PIN);
    WRITE(BUZZER_PIN, LOW);
    
    // waveform generation = 111 = Fast PWM
    TCCR2B |= (1<<WGM22);
    TCCR2A |= (1<<WGM21); 
    TCCR2A |= (1<<WGM20);

    // output mode = 00 (disconnected)
    TCCR2A &= ~(3<<COM2A0); 
    TCCR2A &= ~(3<<COM2B0); 

    // Set the timer pre-scaler
    // Divider of 256 is used, resulting in a 62.5 kHz timer
    // frequency on a 16MHz MCU. 
    TCCR2B = (TCCR2B & ~(0x07<<CS20)) | (3<<CS20); // 62.5 kHz timer

    // Timer 1A -> Enabled
    OCR2A = 0x04; // Initial interrupt frequency = 15.625 kHz
    TCNT2 = 0;
    
    // Timer 1B -> Disabled
    OCR2B = 0xFF;
    TIMSK2 &= ~(1 << OCIE2B);
    
    sei();
}

/***************************************************
 * tone(unsigned short frequency, unit32_t period)
 *
 * frequency:    Value Between 0 and 20 kHz
 * period:       Value Between 0 and 5s

 * Function to start a tone
 ****************************************************/
void buzzer_tone(void){
    setBuzzerPWMDuty();
    
    if (BUZZER_P > MAX_BUZZER_PERIOD) BUZZER_P = MAX_BUZZER_PERIOD;
    else if (BUZZER_P < MIN_BUZZER_PERIOD) BUZZER_P = MIN_BUZZER_PERIOD;
    
    previous_millis_buzzer = millis();
    BUZZER_ON = 1;
    ENABLE_BUZZER();
}

/***************************************************
 * setBuzzerPWMDuty(void)
 *
 * FREQUENCY: 	VALUE BETWEEN 0 AND 4 KHZ
 *
 * SETS THE PWM DUTY OF THE BUZZER.
 ****************************************************/
void setBuzzerPWMDuty(void){
    if (BUZZER_F > MAX_BUZZER_FREQUENCY) BUZZER_F = MAX_BUZZER_FREQUENCY;
    else if (BUZZER_F < MIN_BUZZER_FREQUENCY) BUZZER_F = MIN_BUZZER_FREQUENCY;
    
    int val;
    val = (int) TIMER2A_CLOCK_FREQ * 255.0 / BUZZER_F;
    
    OCR2A = val;
}

/***************************************************
 * Interrupt Service Routine for buzzer
 ****************************************************/
ISR(TIMER2_COMPA_vect){
    if (BUZZER_ON > 0){
        if (millis() - previous_millis_buzzer <= BUZZER_P &&
            millis() - previous_millis_buzzer <= BUZZER_TIMEOUT_PERIOD){
                        
            // TODO: Generate a waveform closer to sinewave to reduce the distortion in sound

            // Toggle buzzer to generate square wave
            WRITE(BUZZER_PIN, buzzer_status);
            buzzer_status = ~buzzer_status;
        }
        else{
            BUZZER_ON = 0;
            WRITE(BUZZER_PIN, LOW);
            DISABLE_BUZZER();
        }
    }
}

#endif

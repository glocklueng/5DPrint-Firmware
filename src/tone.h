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
   \file tone.h
   \brief Header file for tone.c
   
 */

#ifndef TONE_H
#define TONE_H

#define TIMER2A_CLOCK_FREQ        31250.0       // Hz

// The buzzer on 5DPD8 is rated at 4kHz.
// Frequency higher than that sounds rough, therefore upper limit is set to 4 kHz
// Lower frequency tones are alright, lower limit is avoid overflowing the OCR2A register
#define MAX_BUZZER_FREQUENCY      4000         // Hz
#define MIN_BUZZER_FREQUENCY      123          // Hz

#define MAX_BUZZER_PERIOD         2000         // ms
#define MIN_BUZZER_PERIOD         0            // ms

#define ENABLE_BUZZER() TIMSK2 |= (1<<OCIE2A)
#define DISABLE_BUZZER() TIMSK2 &= ~(1<<OCIE2A)

extern short int BUZZER_F;
extern short int BUZZER_P;

void buzzer_init(void);
void buzzer_tone(void);

#endif // #ifndef TONE_H

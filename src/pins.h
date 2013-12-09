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

#ifndef PINS_H
#define PINS_H
#define ALARM_PIN          -1


/****************************************************************************************
* Printrboard Rev. B pin assingments (ATMEGA90USB1286)
* Requires the Teensyduino software with Teensy2.0++ selected in arduino IDE!
* See http://reprap.org/wiki/Printrboard for more info
****************************************************************************************/

#define X_STEP_PIN         28 // PINA0
#define X_DIR_PIN          29 // PINA1
#define X_ENABLE_PIN       12 // PINC2
#define X_MS1_PIN          25 // PINB5
#define X_MS2_PIN          26 // PINB6
#define X_MIN_PIN          18 // PINE6
#define X_MAX_PIN          -1

#define Y_STEP_PIN         30 // PINA2
#define Y_DIR_PIN          31 // PINA3
#define Y_ENABLE_PIN       11 // PINC1
#define Y_MS1_PIN          8  // PINE0
#define Y_MS2_PIN          9  // PINE1
#define Y_MIN_PIN          19 // PINE7
#define Y_MAX_PIN          -1

#define Z_STEP_PIN         32 // PINA4
#define Z_DIR_PIN          33 // PINA5
#define Z_ENABLE_PIN       17 // PINC7
#define Z_MS1_PIN          4  // PIND4
#define Z_MS2_PIN          5  // PIND5
#define Z_MIN_PIN          36 // PINE4
#define Z_MAX_PIN          -1

#define E_STEP_PIN         34 // PINA6
#define E_DIR_PIN          35 // PINA7
#define E_ENABLE_PIN       13 // PINC3
#define E_MS1_PIN          6  // PIND6
#define E_MS2_PIN          7  // PIND7

#define HEATER_0_PIN       15 // PINC5, Extruder
#define HEATER_1_PIN       14 // PINC4, Bed
#define FAN_PIN            16 // PINC6, Fan

#define TEMP_0_PIN          1 // ADC0, Extruder (ADC input number)
#define TEMP_1_PIN          0 // ADC1, Bed (ADC input number)

#define LED_PIN            -1
#define USB_LED_PIN        24 // PINB4
#define PS_ON_PIN          -1
#define KILL_PIN           37 // PINE5

#define BUZZER_PIN         10 // PINC0      

#define SDCD_PIN           47 // PINE3
#define SDCS_PIN           20 // PINB0
#define SCK_PIN            21 // PINB1
#define MISO_PIN           23 // PINB3
#define MOSI_PIN           22 // PINB2

#define I2C_SCL			    0
#define I2C_SDA			    1
#define DIGIPOT_A0		   -1
#define DIGIPOT_A1		   -1
//#define DIGIPOT_RESET		    2

#endif // PINS_H

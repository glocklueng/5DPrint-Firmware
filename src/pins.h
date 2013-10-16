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

#define X_STEP_PIN         28
#define X_DIR_PIN          29
#define X_ENABLE_PIN       19
#define X_MIN_PIN          47
#define X_MAX_PIN          -1

#define Y_STEP_PIN         30
#define Y_DIR_PIN          31
#define Y_ENABLE_PIN       18

// config.h must be included before this file
// Hopefully, should be able to remove this if-else once either new hardware is 
// release or we instruct users to use the E-STOP headers on the PrintRBoard 
// for the Y-STOP / limit switch.
#if SDSUPPORT > 0
	#define Y_MIN_PIN          37
#else
	#define Y_MIN_PIN          20
#endif

#define Y_MAX_PIN          -1

#define Z_STEP_PIN         32
#define Z_DIR_PIN          33
#define Z_ENABLE_PIN       17
#define Z_MIN_PIN          36
#define Z_MAX_PIN          -1

#define E_STEP_PIN         34
#define E_DIR_PIN          35
#define E_ENABLE_PIN       13

#define HEATER_0_PIN       15  // Extruder
#define HEATER_1_PIN       14  // Bed
#define FAN_PIN            16  // Fan

#define TEMP_0_PIN          1  // Extruder (ADC input number)
#define TEMP_1_PIN          0  // Bed (ADC input number)

#define LED_PIN            -1
#define PS_ON_PIN          -1
#define KILL_PIN           -1

#define SCK_PIN            21
#define MISO_PIN           22
#define MOSI_PIN           23

#define I2C_SCL				0
#define I2C_SDA				1
#define DIGIPOT_A0			-1
#define DIGIPOT_A1			-1
#define DIGIPOT_RESET		2


#endif      // PINS_H

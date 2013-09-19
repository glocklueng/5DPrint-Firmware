/* command.h - host commands 
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

#ifndef COMMAND_H
#define COMMAND_H

#include <stdint.h>


struct command {
	uint32_t seqnbr;
	uint8_t type;
	uint8_t dummy;	// byte align (even number of bytes)
	uint16_t code;
	uint16_t has_X : 1;
	uint16_t has_Y : 1;
	uint16_t has_Z : 1;
	uint16_t has_E : 1;
	uint16_t has_F : 1;
	uint16_t has_I : 1;
	uint16_t has_J : 1;
	uint16_t has_P : 1;
	uint16_t has_S : 1;
	uint16_t has_T : 1;
	uint16_t has_D : 1;
	uint16_t has_String : 1;
	float X;
	float Y;
	float Z;
	float E;
	float F;
	float I;
	float J;
	int32_t P;
	int32_t S;
	int32_t T;
	int32_t D;
	char String[92];
};

#endif

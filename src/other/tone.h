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
   \file tone.h
   \brief Header file for tone.c
   
 */

#ifndef TONE_H
#define TONE_H

extern short int BUZZER_F;
extern short int BUZZER_P;

void buzzer_init(void);
void buzzer_tone(void);

#endif // #ifndef TONE_H

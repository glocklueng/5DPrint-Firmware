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

#ifndef _MAKIBOX_H_
#define _MAKIBOX_H_

// A bug in avr-gcc causes spurious warnings when printing a float value:
//   warning: format ‘%f’ expects type ‘double’, but argument 2 has type ‘float’
// (This is because the '%f' format is actually defined to take a double.)
//
// On the AVR architecture, float and double are identical.  This seems to be
// confusing the compiler (see gcc bug #46372).  However, because they are
// identical, we can simply #define away all our worries:


#define float double

#define NUM_AXIS 4
#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2
#define E_AXIS 3

extern unsigned char reset_flags;

void manage_inactivity(unsigned char debug);
//void st_synchronize();

void enable_x();
void enable_y();
void enable_z();
void enable_e();
void disable_x();
void disable_y();
void disable_z();
void disable_e();


#endif // _MAKIBOX_H_

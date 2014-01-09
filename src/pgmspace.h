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

#ifndef FIXED_PGMSPACE
#define FIXED_PGMSPACE

#include <avr/pgmspace.h>

// Silence bogus warnings in avr-gcc < 4.6.3
#undef PROGMEM
#define PROGMEM __attribute__(( section(".progmem.text") ))
#undef PSTR
#define PSTR(s) (__extension__({static const char __c[] PROGMEM = (s); &__c[0];}))

#endif

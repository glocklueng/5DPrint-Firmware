/*
  5DPrint Firmware
  Based on Sprinter (master branch, 1 Sep 2012).
  Designed for Printrboard (Rev B) and 5DPrint D8 Dirver Board.
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
   \file usb.h
   \brief Header file for usb.c
   
 */
#ifndef USB_H
#define USB_H

#include <stddef.h>
#include <stdint.h>
#include "pgmspace.h"


void usb_init();
void usb_shutdown();
void usb_serial_begin();
void usb_serial_end();
int usb_serial_available();
int usb_serial_peek();
int usb_serial_read();
void usb_serial_discard();
size_t usb_serial_write(const uint8_t *buffer, uint16_t size);
void usb_serial_flush();
uint32_t usb_serial_baud();
uint8_t usb_serial_stopbits();
uint8_t usb_serial_paritytype();
uint8_t usb_serial_numbits();
uint8_t usb_serial_dtr();
uint8_t usb_serial_rts();
void toggle_usb_led();
void update_usb_led_status();

// Serial output with printf()-style string formatting.  The version that works
// with format strings in RAM is straightforward;  we add the GCC format(...)
// attribute to the declaration so that we get helpful compiler warnings when
// the format string doesn't match the parameters.
void usb_serial_printf(const char *fmt, ...) 
//__attribute__(( format(gnu_printf, 1, 2) ));
    __attribute__(( format(printf, 1, 2) ));
void usb_serial_printf_P(PGM_P fmt, ...);


// But the one that works with format strings in flash memory is...  tricky.  It
// takes some amount of coaxing to get GCC to type-check format string parameters
// without also placing the format string into RAM.  The __dummy_printf() inline
// function seems to do the trick.
inline void __dummy_usb_serial_printf(const char *fmt, ...)
//__attribute__(( format(gnu_printf, 1, 2) ));
    __attribute__(( format(printf, 1, 2) ));
inline void __dummy_usb_serial_printf(const char *fmt, ...) { return; }


// Note that this macro *EVALUATES ITS ARGUMENTS TWICE*.  This is (sadly) an
// unavoidable consequence of needing to call __dummy_printf().  Do not do things
// such as:
//     serial_send("i=%d\n", i++);
#define serial_send(fmt, args...)                       \
    do {                                                \
        static const char __c[] PROGMEM = (fmt);        \
        __dummy_usb_serial_printf((fmt), ##args);       \
        usb_serial_printf_P(__c, ##args);               \
    } while(0)
	
#endif // USB_H

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

#ifndef STEPPER_H
#define STEPPER_H

#define ENABLE_STEPPER_DRIVER_INTERRUPT()  TIMSK1 |= (1<<OCIE1A)
#define DISABLE_STEPPER_DRIVER_INTERRUPT() TIMSK1 &= ~(1<<OCIE1A)

#if (DEBUG > -1)
	extern uint32_t timer1_compa_isr_exe_micros;
	extern uint32_t timer1_compa_isr_exe_micros_min;
	extern uint32_t timer1_compa_isr_exe_micros_max;
#endif

extern unsigned short virtual_steps_x;
extern unsigned short virtual_steps_y;
extern unsigned short virtual_steps_z;

extern uint8_t is_homing;
extern uint8_t x_homed, y_homed, z_homed;

extern uint8_t pause_print_req;
extern uint8_t print_paused;

typedef struct {
	long x;
	long y;
	long z;
	long e;
} st_position_t;

void st_init();
void st_wake_up();
void st_sleep();
void st_synchronize();
void st_set_current_position(st_position_t new_position);
st_position_t st_get_current_position(void);
void get_current_printer_state(void);
void set_print_paused_buffer(void);
void clear_plan_buffer(void);
void resume_normal_print_buffer(void);

#endif
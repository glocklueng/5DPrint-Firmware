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
   \file store_eeprom.h
   \brief Header file for store_eeprom.c
   
 */

#ifndef __EEPROMH
#define __EEPROMH

#define EEPROM_OFFSET 100

// EEPROM_VERSION is not really used for checking anything now but left in here 
// and saved to the EEPROM for the moment.
#define EEPROM_VERSION "B00"

extern float axis_steps_per_unit[4]; 
extern float max_feedrate[4];
extern long  max_acceleration_units_per_sq_second[4];
extern float move_acceleration;
extern float retract_acceleration;
extern float mintravelfeedrate;
extern float minimumfeedrate;
extern float max_xy_jerk;
extern float max_z_jerk;
extern float max_e_jerk;

#ifdef SLOWDOWN
extern unsigned long minsegmenttime;
#endif


// EEPROM addresses for the various configuration values stored.
// Take extra care when changing or modifying these.
#define axis_steps_per_unit_address 					  (EEPROM_OFFSET + 4*sizeof(char))
#define max_feedrate_address 							  (axis_steps_per_unit_address + 4*sizeof(float))
#define max_acceleration_units_per_sq_second_address	  (max_feedrate_address + 4*sizeof(float))
#define move_acceleration_address 						  (max_acceleration_units_per_sq_second_address + 4*sizeof(long))
#define retract_acceleration_address 					  (move_acceleration_address + sizeof(float))
#define mintravelfeedrate_address 						  (retract_acceleration_address + sizeof(float))
#define minimumfeedrate_address 						  (mintravelfeedrate_address + sizeof(float))
#define max_xy_jerk_address 							  (minimumfeedrate_address + sizeof(float))
#define max_z_jerk_address 								  (max_xy_jerk_address + sizeof(float))
#define max_e_jerk_address 								  (max_z_jerk_address + sizeof(float))
#define Kp_address 										  (max_e_jerk_address + sizeof(unsigned long))
#define Ki_address 										  (Kp_address + sizeof(unsigned int))
#define Kd_address 										  (Ki_address + sizeof(unsigned int))
#define max_x_motor_current_address						  (Kd_address + sizeof(unsigned int))
#define max_y_motor_current_address						  (max_x_motor_current_address + sizeof(unsigned short))
#define max_z_motor_current_address						  (max_y_motor_current_address + sizeof(unsigned short))
#define max_e_motor_current_address						  (max_z_motor_current_address + sizeof(unsigned short))
#define stepper_sense_resistance_address			      (max_e_motor_current_address + sizeof(unsigned short))
#define user_max_bed_heater_duty_before_full_pwr_address  (stepper_sense_resistance_address + sizeof(unsigned char))
#define user_max_bed_heater_duty_address                  (user_max_bed_heater_duty_before_full_pwr_address + sizeof(unsigned short))
#define autoprint_enabled_address                         (user_max_bed_heater_duty_address + sizeof(unsigned short))

#define EEPROM_START_ADDR		EEPROM_OFFSET
#define EEPROM_END_ADDR			(autoprint_enabled_address + sizeof(unsigned char))
#define EEPROM_CHECKSUM_ADDR	EEPROM_END_ADDR

extern void EEPROM_RetrieveSettings(int def, int printout);
extern void EEPROM_printSettings();
extern void EEPROM_StoreSettings();


#endif

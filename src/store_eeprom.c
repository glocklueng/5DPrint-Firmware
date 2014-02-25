/*
  EEPROM routines to save settings 
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
   \file store_eeprom.c
   \brief Store settings in EEPROM for future use
   
 */


#include <avr/eeprom.h>
#include "pgmspace.h"
#include <stdlib.h>
#include <string.h>
#include <util/crc16.h>

#include "config.h"
#include "store_eeprom.h"
#include "usb.h"
#include "heater.h"
#include "language.h"
#include "stepper.h"
#include "heater.h"
#include "autoprint.h"
#include "i2c/Master_I2C_Comms.h"

#ifdef PIDTEMP
//extern unsigned int PID_Kp, PID_Ki, PID_Kd;
#endif


#ifdef USE_EEPROM_SETTINGS

// Function Prototypes
unsigned short EEPROM_Checksum(void);
unsigned char EEPROM_read_setting(int address, void * valuePtr, size_t NumBytes);
unsigned char EEPROM_write_setting(int address, void * valuePtr, size_t NumBytes);


//======================================================================================
//========================= Read / Write EEPROM =======================================
unsigned char EEPROM_write_setting(int address, void * valuePtr, size_t NumBytes)
{  
    unsigned char i;
    unsigned char *p;
  
    p = valuePtr;
  
    for (i = 0; i < NumBytes; i++)
        eeprom_write_byte((unsigned char *)address++, *p++);
  
    return i;
}

unsigned char EEPROM_read_setting(int address, void * valuePtr, size_t NumBytes)
{
    unsigned char i;
    unsigned char *p;
  
    p = valuePtr;
  
    for (i = 0; i < NumBytes; i++)
        *p++ = eeprom_read_byte((unsigned char *)address++);
	
    return i;
}
//======================================================================================


void EEPROM_StoreSettings() 
{
    char ver[4]= EEPROM_VERSION;
    EEPROM_write_setting(EEPROM_OFFSET, &ver, sizeof(ver)); // invalidate data first
  
    EEPROM_write_setting(axis_steps_per_unit_address, &axis_steps_per_unit, 
                         sizeof(axis_steps_per_unit));
													
    EEPROM_write_setting(max_feedrate_address, &max_feedrate, sizeof(max_feedrate));
  
    EEPROM_write_setting(max_acceleration_units_per_sq_second_address, 
                         &max_acceleration_units_per_sq_second, 
                         sizeof(max_acceleration_units_per_sq_second));
									
    EEPROM_write_setting(move_acceleration_address, &move_acceleration, 
                         sizeof(move_acceleration));
													
    EEPROM_write_setting(retract_acceleration_address, &retract_acceleration, 
                         sizeof(retract_acceleration));
													
    EEPROM_write_setting(minimumfeedrate_address, &minimumfeedrate, 
                         sizeof(minimumfeedrate));
													
    EEPROM_write_setting(mintravelfeedrate_address, &mintravelfeedrate, 
                         sizeof(mintravelfeedrate));
													
    EEPROM_write_setting(max_xy_jerk_address, &max_xy_jerk, sizeof(max_xy_jerk));
  
    EEPROM_write_setting(max_z_jerk_address, &max_z_jerk, sizeof(max_z_jerk));
  
    EEPROM_write_setting(max_e_jerk_address, &max_e_jerk, sizeof(max_e_jerk));

    //PID Settings
#ifdef PIDTEMP
    EEPROM_write_setting(Kp_address, &PID_Kp, sizeof(PID_Kp));   //Kp
    EEPROM_write_setting(Ki_address, &PID_Ki, sizeof(PID_Ki));   //Ki
    EEPROM_write_setting(Kd_address, &PID_Kd, sizeof(PID_Kd));   //Kd
#else
    unsigned int kp, ki, kd;
    kp = PID_PGAIN;
    ki = PID_IGAIN;
    kd = PID_DGAIN;
    EEPROM_write_setting(Kp_address, &kp, sizeof(kp));     //Kp
    EEPROM_write_setting(Ki_address, &ki, sizeof(ki));     //Ki
    EEPROM_write_setting(Kd_address, &kd, sizeof(kd));     //Kd
#endif

#ifdef BED_PIDTEMP
    EEPROM_write_setting(user_max_bed_heater_duty_before_full_pwr_address, 
                         &user_max_bed_heater_duty_before_full_pwr, 
                         sizeof(user_max_bed_heater_duty_before_full_pwr));
    EEPROM_write_setting(user_max_bed_heater_duty_address,
                         &user_max_bed_heater_duty, 
                         sizeof(user_max_bed_heater_duty_address));
#endif
  
#if DIGIPOTS > 0
    EEPROM_write_setting(max_x_motor_current_address, &max_x_motor_current, sizeof(max_x_motor_current));
    EEPROM_write_setting(max_y_motor_current_address, &max_y_motor_current, sizeof(max_y_motor_current));
    EEPROM_write_setting(max_z_motor_current_address, &max_z_motor_current, sizeof(max_z_motor_current));
    EEPROM_write_setting(max_e_motor_current_address, &max_e_motor_current, sizeof(max_e_motor_current));
    EEPROM_write_setting(stepper_sense_resistance_address, &stepper_sense_resistance, sizeof(stepper_sense_resistance));
#endif
  
#if AUTOPRINT > 0
    EEPROM_write_setting(autoprint_enabled_address, &autoprint_enabled, sizeof(autoprint_enabled));
#endif

    unsigned short checksum = EEPROM_Checksum();
    EEPROM_write_setting(EEPROM_CHECKSUM_ADDR, &checksum, sizeof(checksum));
    serial_send(TXT_SETTINGS_STORED_CRLF);
}


void EEPROM_printSettings()
{
    char str_buf0[10], str_buf1[10], str_buf2[10], str_buf3[10], str_buf4[10];
    unsigned short stored_checksum;
	
#ifdef PRINT_EEPROM_SETTINGS
    serial_send(TXT_STEPS_PER_UNIT_CRLF_M92_X_Y_Z_E_CRLF,
                (unsigned short)axis_steps_per_unit[0],
                (unsigned short)axis_steps_per_unit[1],
                (unsigned short)axis_steps_per_unit[2],
                (unsigned short)axis_steps_per_unit[3]);
      
    dtostrf(max_feedrate[0], 3, 3, str_buf0);
    dtostrf(max_feedrate[1], 3, 3, str_buf1);
    dtostrf(max_feedrate[2], 3, 3, str_buf2);
    dtostrf(max_feedrate[3], 3, 3, str_buf3);
	  
    serial_send(TXT_MAX_FEEDRATES_CRLF_M202_X_Y_Z_E_CRLF,
                str_buf0,
                str_buf1,
                str_buf2,
                str_buf3);

    serial_send(TXT_MAX_ACCEL_CRLF_M201_X_Y_Z_E_CRLF,
                max_acceleration_units_per_sq_second[0],
                max_acceleration_units_per_sq_second[1],
                max_acceleration_units_per_sq_second[2],
                max_acceleration_units_per_sq_second[3]);

      
    dtostrf(move_acceleration, 5, 3, str_buf0);
    dtostrf(retract_acceleration, 5, 3, str_buf1);
	  
    serial_send(TXT_ACCEL_S_T_CRLF);
    serial_send(TXT_M204_S_T_CRLF, str_buf0, str_buf1);


    dtostrf(minimumfeedrate, 3, 3, str_buf0);
    dtostrf(mintravelfeedrate, 3,3, str_buf1);
    dtostrf(max_xy_jerk, 3, 3, str_buf2);
    dtostrf(max_z_jerk, 3, 3, str_buf3);
    dtostrf(max_e_jerk, 3, 3, str_buf4);
	  
    serial_send(TXT_ADAVNCED_VARIABLES_S_T_XY_Z_E_JERK_CRLF);
    serial_send(TXT_M205_S_T_XY_Z_E_CRLF,	str_buf0,
                str_buf1,
                str_buf2,
                str_buf3,
                str_buf4);

#ifdef PIDTEMP
    serial_send(TXT_PID_SETTINGS_CRLF_M301_P_I_D_CRLF, PID_Kp, PID_Ki, PID_Kd); 
#endif

#ifdef BED_PIDTEMP
    serial_send(TXT_MAX_BED_HEATER_DUTY_SETTINGS_CRLF_M305_CRLF,
                user_max_bed_heater_duty_before_full_pwr*100/BED_HEATER_CURRENT,
                user_max_bed_heater_duty*100/BED_HEATER_CURRENT);     
#endif
	
#if DIGIPOTS > 0
    serial_send(TXT_MAX_MOTOR_CURRENTS_CRLF_M906_X_Y_Z_E_CRLF, max_x_motor_current, 
                max_y_motor_current,
                max_z_motor_current,
                max_e_motor_current, 
                stepper_sense_resistance);
#endif

#if AUTOPRINT > 0
    if (autoprint_enabled == 1) serial_send(TXT_M31_AUTOPRINT_ENABLED_CRLF);
    else if (autoprint_enabled == 0) serial_send(TXT_M31_AUTOPRINT_DISABLED_CRLF);
#endif

#else
    serial_send(TXT_PRINTING_OF_EEPROM_SETTINGS_DISABLED_CRLF);
#endif	// #ifdef PRINT_EEPROM_SETTINGS
		
    EEPROM_read_setting(EEPROM_CHECKSUM_ADDR, &stored_checksum, 
                        sizeof(stored_checksum));
	
    serial_send(TXT_STORED_EEPROM_CHECKSUM_HEX_CRLF, stored_checksum);
    serial_send(TXT_EXPECTED_EEPROM_CHECKSUM_HEX_CRLF, EEPROM_Checksum());
} 


void EEPROM_RetrieveSettings(int def, int printout)
{  // if def=true, the default values will be used
	
    unsigned short stored_checksum = 0;
    
    EEPROM_read_setting(EEPROM_CHECKSUM_ADDR, &stored_checksum, 
                        sizeof(stored_checksum));
		
    if ( (!def) && (stored_checksum == EEPROM_Checksum()) )
        {   // checksum match
            EEPROM_read_setting(axis_steps_per_unit_address, &axis_steps_per_unit, 
                                sizeof(axis_steps_per_unit));
												
            EEPROM_read_setting(max_feedrate_address, &max_feedrate, 
                                sizeof(max_feedrate));
														
            EEPROM_read_setting(max_acceleration_units_per_sq_second_address, 
                                &max_acceleration_units_per_sq_second, 
                                sizeof(max_acceleration_units_per_sq_second));
										
            EEPROM_read_setting(move_acceleration_address, &move_acceleration, 
                                sizeof(move_acceleration));
													
            EEPROM_read_setting(retract_acceleration_address, &retract_acceleration, 
                                sizeof(retract_acceleration));
												
            EEPROM_read_setting(minimumfeedrate_address, &minimumfeedrate, 
                                sizeof(minimumfeedrate));
														
            EEPROM_read_setting(mintravelfeedrate_address, &mintravelfeedrate, 
                                sizeof(mintravelfeedrate));
													
            EEPROM_read_setting(max_xy_jerk_address, &max_xy_jerk, sizeof(max_xy_jerk));
            EEPROM_read_setting(max_z_jerk_address, &max_z_jerk, sizeof(max_z_jerk));
            EEPROM_read_setting(max_e_jerk_address, &max_e_jerk, sizeof(max_e_jerk));

#ifdef PIDTEMP
            EEPROM_read_setting(Kp_address, &PID_Kp, sizeof(PID_Kp));
            EEPROM_read_setting(Ki_address, &PID_Ki, sizeof(PID_Ki));
            EEPROM_read_setting(Kd_address, &PID_Kd, sizeof(PID_Kd));
#endif

#ifdef BED_PIDTEMP
            EEPROM_read_setting(user_max_bed_heater_duty_address, 
                                &user_max_bed_heater_duty, 
                                sizeof(user_max_bed_heater_duty));
            EEPROM_read_setting(user_max_bed_heater_duty_before_full_pwr_address, 
                                &user_max_bed_heater_duty_before_full_pwr, 
                                sizeof(user_max_bed_heater_duty_before_full_pwr));
#endif

#if DIGIPOTS > 0
            EEPROM_read_setting(max_x_motor_current_address, &max_x_motor_current, sizeof(max_x_motor_current));
            EEPROM_read_setting(max_y_motor_current_address, &max_y_motor_current, sizeof(max_y_motor_current));
            EEPROM_read_setting(max_z_motor_current_address, &max_z_motor_current, sizeof(max_z_motor_current));
            EEPROM_read_setting(max_e_motor_current_address, &max_e_motor_current, sizeof(max_e_motor_current));
            EEPROM_read_setting(stepper_sense_resistance_address, &stepper_sense_resistance, sizeof(stepper_sense_resistance));
            I2C_digipots_set_defaults();
#endif

#if AUTOPRINT > 0
            EEPROM_read_setting(autoprint_enabled_address, &autoprint_enabled, sizeof(autoprint_enabled));
#endif

            serial_send(TXT_STORED_SETTINGS_RETRIEVED_CRLF);
        }
    else 
        {
            float tmp1[]=_AXIS_STEP_PER_UNIT;
            float tmp2[]=_MAX_FEEDRATE;
            long tmp3[]=_MAX_ACCELERATION_UNITS_PER_SQ_SECOND;
            for (short i=0;i<4;i++) 
                {
                    axis_steps_per_unit[i]=tmp1[i];  
                    max_feedrate[i]=tmp2[i];  
                    max_acceleration_units_per_sq_second[i]=tmp3[i];
                }
            move_acceleration=_ACCELERATION;
            retract_acceleration=_RETRACT_ACCELERATION;
            minimumfeedrate=DEFAULT_MINIMUMFEEDRATE;
            mintravelfeedrate=DEFAULT_MINTRAVELFEEDRATE;
            max_xy_jerk=_MAX_XY_JERK;
            max_z_jerk=_MAX_Z_JERK;
            max_e_jerk=_MAX_E_JERK;
      
#ifdef PIDTEMP
            PID_Kp = PID_PGAIN;
            PID_Ki = PID_IGAIN;
            PID_Kd = PID_DGAIN;
#endif

#ifdef BED_PIDTEMP
            user_max_bed_heater_duty_before_full_pwr = BED_HEATER_CURRENT_BEFORE_FULL_PWR;
            user_max_bed_heater_duty = BED_HEATER_CURRENT_FULL_PWR;              
#endif
	  
#if DIGIPOTS > 0
            max_x_motor_current = XAXIS_DEFAULT_MAX_CURRENT;
            max_y_motor_current = YAXIS_DEFAULT_MAX_CURRENT;
            max_z_motor_current = ZAXIS_DEFAULT_MAX_CURRENT;
            max_e_motor_current = EAXIS_DEFAULT_MAX_CURRENT;
            stepper_sense_resistance = ALLEGRO_A4982_RS;
            I2C_digipots_set_defaults();
#endif

#if AUTOPRINT > 0
            autoprint_enabled = AUTOPRINT_DEFAULT_ENABLED;
#endif
            serial_send(TXT_USING_DEFAULT_SETTINGS_CRLF);
        }
    
    if(printout)
        {
            EEPROM_printSettings();
        }
}  

unsigned short EEPROM_Checksum(void)
{
    unsigned int i;
    unsigned short calculated_checksum = 0;
    unsigned char byte;
	
    for (i = EEPROM_START_ADDR; i < EEPROM_END_ADDR; i++)
	{
            byte = eeprom_read_byte((unsigned char *)i);
            calculated_checksum = _crc_xmodem_update(calculated_checksum, byte);
	}
	
    return calculated_checksum;
}

#endif

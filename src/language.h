/*
  5DPrint Firmware
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
   \file language.h
   \brief Return strings for serial communication
   
   Currently, only English is available
 */

#ifndef _LANGUAGE_H_
#define _LANGUAGE_H_

// LANGUAGE SELECTION
// 0 = ENGLISH

#ifndef LANGUAGE
#define LANGUAGE 0
#endif // #ifndef LANGUAGE

#if (LANGUAGE == 0)		// ENGLISH

// Basic Characters
#define TXT_OPEN_BRACKETS_SPACE									"( "
#define TXT_CLOSE_BRACKETS										")"
#define TXT_INT_INT												" %d %d "
#define TXT_INT_CRLF											"%d\r\n"
#define TXT_255_CRLF											"255\r\n"
#define TXT_0_CRLF												"0\r\n"
#define TXT_CRLF												"\r\n"
#define TXT_FORWARDSLASH_FORWARDSLASH							"/" "/ "
#define TXT_STR													"%s"
#define TXT_SPACE												" "
#define TXT_LU													"%lu"

// PID Control
#define TXT_PID_AUTOTUNE_START_CRLF								"PID Autotune start\r\n"
#define TXT_BIAS_MIN_MAX										" bias: %s d: %s min: %s max: %s"
#define TXT_KU_TU_CRLF				 							" Ku: %s Tu: %s\r\n"
#define TXT_CLASSIC_PID_CRLF									" Clasic PID \r\n"
#define TXT_SOME_OVERSHOOT_CRLF									" Some overshoot \r\n"
#define TXT_NO_OVERSHOOT_CRLF									" No overshoot \r\n"
#define TXT_CFG_KP_CRLF											" CFG Kp: %d\r\n"
#define TXT_CFG_KI_CRLF											" CFG Ki: %d\r\n"
#define TXT_CFG_KD_CRLF											" CFG Kd: %d\r\n"
#define TXT_PID_AUTOTUNE_FAILED_TEMP_HIGH_CRLF					"PID Autotune failed! Temperature to high\r\n"
#define TXT_PID_AUTOTUNE_FAILED_TIMEOUT_CRLF					"PID Autotune failed! timeout\r\n"
#define TXT_PID_AUTOTUNE_FINISHED_CRLF							"PID Autotune finished ! Place the Kp, Ki and Kd constants in the configuration.h\r\n"
#define TXT_PID_SETTINGS_CHANGED_NOT_SAVED_TO_MEM_CRLF			"-- PID settings changed (but not saved to memory).\r\n"
#define TXT_PID_SETTINGS_CRLF_M301_P_I_D_CRLF					"PID settings:\r\n  M301 P%d I%d D%d\r\n"

// Serial
#define TXT_5DPRINT_VER_STARTED_CRLF							"/" "/ 5DPrint %s started.\r\n"
#define TXT_OK_T_AT_DUTY_CRLF									"ok T:%s @:%d\r\n"									// *** Host / System Text String
#define TXT_OK_T_INT											"ok T:%d"											// *** Host / System Text String
#define TXT_OK_SEQNUM_Q_MS_EXECUTE_CRLF							"ok %ld Q%d (%lums execute)\r\n"					// *** Host / System Text String
#define TXT_GO_SEQNUM_EXECUTING_CRLF							"go %ld (executing %c%d)\r\n"						// *** Host / System Text String
#define TXT_UNKNOWN_COMMAND_TYPE								"-- Unknown Command Type.\r\n"
#define TXT_UNKNOWN_CODE_GX_CRLF								"-- Unknown code G%d.\r\n"
#define TXT_DELAYING_MS_CRLF									"-- delaying %lums\r\n"
#define TXT_ERROR_CODE_LU_CRLF									"-- Error Code: %lu.\r\n"
#define TXT_DONE_PRINTING_FILE_CRLF								"Done printing file\r\n"							// *** Host / System Text String
#define TXT_ERROR_READING_FILE_CRLF								"-- Error reading file.\r\n"
#define TXT_A_INT												" A%d"												// *** Host / System Text String
#define TXT_B_INT												" B:%d"												// *** Host / System Text String
#define TXT_OK_X_Y_Z_E_CRLF										"ok X%f Y%f Z%f E%f\r\n"							// *** Host / System Text String
#define TXT_S_PARAM_REQUIRED_CRLF								"!! S param required.\r\n"
#define TXT_EXISTING_USB_CONN_WILL_BE_DISCONNECTED_CRLF			"/" "/ Existing USB connection will be disconnected.\r\n"
#define TXT_PLEASE_DISCONNECT_N_CLOSE_HOST_SW_CRLF_CRLF			"/" "/ Please disconnect and close your current host software.\r\n\r\n"
#define TXT_UNKNOWN_CODE_M_CRLF									"-- Unknown code M%d.\r\n"
#define TXT_CRLF_RESUMING_PRINT_PLEASE_WAIT						"\r\n/" "/ Resuming print. Please wait..."
#define TXT_CRLF_PAUSING_PRINT_CRLF								"\r\n/" "/ Pausing print...\r\n"
#define TXT_STOP_PRINT_POWER_OFF_PRINTER_DISCONNECT_CRLF		"/" "/ *** STOP PRINT!!! - Power Off Printer - Disconnect and close host software.\r\n"
#define TXT_FIRMWARE_WILL_CONTINUE_OP_AFTER_30_SECS_CRLF		"\r\n/" "/ *** Firmware will continue operation after 30 seconds...\r\n"
#define TXT_CONTINUING_CRLF										"/" "/ *** Continuing...\r\n"

// Tempearture
#define TXT_MTEMP												"MTEMP:%lu"
#define TXT_MAX_ALLOWED_HOTEND_TEMP_IS_INT_DEGC_CRLF			"/" "/ Max allowed hotend temperature is %ddegC\r\n"
#define TXT_SETTING_TARGET_HOTEND_TEMP_TO_INT_DEGC_CRLF			"/" "/ Setting target hotend temperature to %ddegC\r\n"
#define TXT_MAX_ALLOWED_BED_TEMP_IS_INT_DEGC_CRLF				"/" "/ Max allowed bed temperature is %ddegC\r\n"
#define TXT_SETTING_TARGET_BED_TEMP_TO_INT_DEGC_CRLF			"/" "/ Setting target bed temperature to %ddegC\r\n"
#define TXT_DASH_DASH_T_INT										"-- T:%d"
#define TXT_D_INT_PERCENT										" D%d%%"											// *** Host / System Text String
#define TXT_CRLF_TARGET_TEMP_DEGC								"\r\n/"	"/ Target Temperature: %ddegC"
#define TXT_WAITING_FOR_EXTRUDER_TO_REACH_TARGET_TEMP_CRLF		"\r\n/" "/ Waiting for extruder heater to reach target temperature...\r\n"
#define TXT_T_D_B_D_CRLF										"T:%d D%d%% B:%d D%d%% \r\n"
#define TXT_CRLF_HOTEND_TOO_LONG_TO_REACH_TARGET_TIMED_OUT_CRLF	"\r\n/" "/ *** Hot-end heater took too long to reach target. Timed Out!\r\n"
#define TXT_HOTEND_HEATER_NOT_RESPONDING_CRLF					"\r\n/" "/ *** Hot-end heater does not appear to be responding.\r\n"
#define TXT_CHECK_HOTEND_N_HOTBED_THERMISTOR_CONNECTIONS_CRLF	"/" "/ *** Check hot-end thermistor connections!!!\r\n"
#define TXT_CRLF_WAITING_FOR_HOTBED_HEATER_TO_REACH_TARGET_CRLF	"\r\n/" "/ Waiting for hot-bed heater to reach target temperature...\r\n"
#define TXT_CRLF_HOTBED_HEATER_TOOK_TOO_LONG_TIMED_OUT_CRLF		"\r\n/" "/ *** Hot bed heater took too long to reach target. Timed Out!\r\n"
#define TXT_CRLF_HOTBED_HEATER_APPEARS_NOT_RESPONDING_CRLF		"\r\n/"	"/ *** Hot-bed heater does not appear to be responding.\r\n"
#define TXT_CHECK_HOTBED_THERMISTOR_CONNECTIONS					"/" "/ *** Check hot-bed thermistor connections!!!\r\n"
#define TXT_EXISTING_EXTRUDER_MAX_CURRENT_INT_PERCENT_CRLF		"/"	"/ Existing Extruder Heater Max Current: %d%%\r\n"
#define TXT_SETTING_EXTRUDER_MAX_CURRENT_INT_PERCENT_CRLF		"/"	"/ Setting Extruder Heater Max Current: %d%%\r\n"
#define TXT_MAX_BED_HEATER_DUTY_SETTINGS_CRLF_M305_CRLF		    "Max Bed Heater Duty Cycle settings (M305):\r\n before full power: %u%% \r\n after full power: %u%% \r\n"

// Motor
#define TXT_X_MIN_STR											"x_min:%s"
#define TXT_X_MAX_STR											"x_max:%s"
#define TXT_Y_MIN_STR											"y_min:%s"
#define TXT_Y_MAX_STR											"y_max:%s"
#define TXT_Z_MIN_STR											"z_min:%s"
#define TXT_Z_MAX_STR											"z_max:%s"
#define TXT_X_MAX_LENGTH_INT									"/" "/ X_MAX_LENGTH:%d "
#define TXT_Y_MAX_LENGTH_INT									"Y_MAX_LENGTH:%d "
#define TXT_Z_MAX_LENGTH_INT_CRLF								"Z_MAX_LENGTH:%d \r\n"
#define TXT_M206_ADDHOME_X_Y_Z_CRLF								"/" "/ M206 Addhome X:%f Y:%f Z:%f\r\n"
#define TXT_STEPS_PER_MM_SETTINGS_CHANGED_NOT_SAVED_TO_MEM_CRLF	"-- Steps per mm settings changed (but not saved to memory).\r\n"
#define TXT_MOTORS_OFF_HEATERS_OFF_CRLF							"/"	"/ Motors off. Heaters Off.\r\n"
#define TXT_C_X_Y_Z_E_MM_CRLF									"-- C: X:%s Y:%s Z:%s E:%s (mm)\r\n"
#define TXT_X_Y_Z_E_STEPS_CRLF									"-- X:%ld Y:%ld Z:%ld E:%ld (steps)\r\n"
#define TXT_AXES_HOMED_X_Y_Z_CRLF								"-- Axes Homed X:%d Y:%d Z:%d\r\n"
#define TXT_NOT_ALL_AXES_HOMED_POSITION_MAYBE_INCORRECT_CRLF	"-- *Not all axes homed! Positions reported may be incorrect!!!\r\n"
#define TXT_STEPS_PER_UNIT_CRLF_M92_X_Y_Z_E_CRLF				"Steps per unit:\r\n  M92 X%d Y%d Z%d E%d\r\n"
#define TXT_MAX_FEEDRATES_CRLF_M202_X_Y_Z_E_CRLF				"Maximum feedrates (mm/s):\r\n  M202 X%s Y%s Z%s E%s\r\n"
#define TXT_MAX_ACCEL_CRLF_M201_X_Y_Z_E_CRLF					"Maximum Acceleration (mm/s2):\r\n  M201 X%ld Y%ld Z%ld E%ld\r\n"
#define TXT_ACCEL_S_T_CRLF										"Acceleration: S=acceleration, T=retract acceleration\r\n"
#define TXT_M204_S_T_CRLF										"  M204 S%s T%s\r\n"
#define TXT_ADAVNCED_VARIABLES_S_T_XY_Z_E_JERK_CRLF				"Advanced variables (mm/s): S=Min feedrate, T=Min travel feedrate, XY=max xY jerk,  Z=max Z jerk, E=max E jerk\r\n"
#define TXT_PLANNER_INIT										"/" "/ Planner Init\r\n"
#define TXT_M205_S_T_XY_Z_E_CRLF								"  M205 S%s T%s XY%s Z%s E%s\r\n"

// SD Card
#define TXT_BEGIN_FILE_LIST_CRLF								"Begin file list\r\n"								// *** Host / System Text String
#define TXT_END_FILE_LIST_CRLF									"End file list\r\n"									// *** Host / System Text String
#define TXT_FILE_UPLOAD_COMPLETE_CRLF							"File upload complete\r\n"							// *** Host / System Text String
#define TXT_WRITING_TO_FILE_CRLF								"Writing to file\r\n"								// *** Host / System Text String
#define TXT_OPENING_STR_CRLF									"-- Opening %s...\r\n"
#define TXT_FILE_SELECTED_CRLF									"File selected\r\n"									// *** Host / System Text String
#define TXT_FILE_OPEN_FAILED_CRLF								"file.open failed\r\n"								// *** Host / System Text String
#define TXT_SD_PRINTING_BYTE_LU_OF_LU_CRLF						"SD printing byte %lu/%lu\r\n"						// *** Host / System Text String
#define TXT_COULD_NOT_CREATE_FILE_CRLF							"-- Could not create file.\r\n"
#define TXT_DELETING_STR_CRLF									"-- Deleting %s...\r\n"
#define TXT_CLOSED_SD_CARD_FILE_CRLF							"-- Closed SD card file.\r\n"
#define TXT_SD_CARD_INIT_FAILED_CRLF							"-- *** SD Card Initialisation Failed.\r\n"
#define TXT_SD_CARD_INITIALISED_CRLF							"-- SD Card Initialised.\r\n"
#define TXT_FAILED_TO_OPEN_PARTITION_CRLF						"-- *** Failed to open partition.\r\n"
#define TXT_OPENING_FILESYSTEM_FAILED_CRLF						"-- *** Opening filesystem failed\r\n"
#define TXT_MANUFACTURER_HEX_CRLF								"-- Manufacturer: 0x%04X\r\n"
#define TXT_OEM_STR_CRLF										"-- OEM: %s\r\n"
#define TXT_PRODUCT_STR_CRLF									"-- Product: %s\r\n"
#define TXT_REVISION_HEX_CRLF									"-- Revision:  0x%04X\r\n"
#define TXT_SERIAL_HEX_CRLF										"-- Serial: 0x%X\r\n"
#define TXT_DATE_INT_INT_CRLF									"-- Date: %02d / %02d\r\n"
#define TXT_SIZE_INT_MB_CRLF									"-- Size: %dMB\r\n"
#define TXT_COPY_INT_CRLF										"-- Copy: %d\r\n"
#define TXT_WRITE_PROTECT_INT_INT_CRLF							"-- Write Protect: %d / %d\r\n"
#define TXT_FORMAT_INT_CRLF										"-- Format: %d\r\n"
#define TXT_FREE_SPACE_LU_LU_BYTES_CRLF							"-- Free Space: %lu / %lu Bytes\r\n"
#define TXT_SD_CARD_RELEASED_CRLF								"-- SD Card Released.\r\n"
#define TXT_OPENING_DIRECTORY_FAILED_CRLF						"-- Opening directory failed.\r\n"
#define TXT_SD_CARD_FILE_SYSTEM_NOT_INITIALISED_CRLF			"-- SD card file system not initialised.\r\n"
#define TXT_DELETED_FILE_STR_CRLF								"-- Deleted File: %s\r\n"
#define TXT_COULD_NOT_DELETE_STR_FILE_DELETE_FAILED_CRLF		"-- Could not delete %s. File delete failed.\r\n"

// EEPROM Settings
#define TXT_PRINTING_OF_EEPROM_SETTINGS_DISABLED_CRLF			"(printing of EEPROM settings disabled)\r\n"
#define TXT_STORED_EEPROM_CHECKSUM_HEX_CRLF						"Stored EEPROM Checksum: 0x%X\r\n"
#define TXT_EXPECTED_EEPROM_CHECKSUM_HEX_CRLF					"Expected EEPROM Checksum: 0x%X\r\n"
#define TXT_STORED_SETTINGS_RETRIEVED_CRLF						"Stored settings retreived\r\n"
#define TXT_USING_DEFAULT_SETTINGS_CRLF							"Using Default settings\r\n"
#define TXT_CRLF_CLEARING_BUFFERED_MOVES_RESUME_NORMAL_OP_CRLF	"\r\n-- Clearing buffered moves and resuming normal operation...\r\n"
#define TXT_CRLF_CANCELLED_SD_CARD_PRINT_CRLF					"\r\n-- Cancelled SD card print.\r\n"
#define TXT_STEPPER_MOTORS_AUTO_DISABLED_DUE_TO_INACTIVITY_CRLF	"-- Stepper motors automatically disabled due to inactivity timeout.\r\n"
#define TXT_HEATERS_AND_MOTORS_DISABLED_DUE_INACTIVITY_CRLF 	"-- Heaters and motors automatically disabled due to inactivity timeout.\r\n"
#define TXT_ALL_HEATERS_DISABLED_SAFETY_TEMP_EXCEEDED_CRLF		"-- *** All heaters disabled. Safety temperature exceeded.\r\n"
#define TXT_CHECK_HEATER_AND_THERMISTOR_CONNS_ARE_CORRECT_CRLF	"-- Check all heater and thermistor connections are correct.\r\n"
#define TXT_COLD_EXTRUSION_PREVENTED_CRLF						"-- Cold extrusion prevented.\r\n"
#define TXT_LONG_EXTRUSION_PREVENTED_CRLF						"-- Long extrusion prevented.\r\n"
#define TXT_SETTINGS_STORED_CRLF								"Settings Stored\r\n"

// Digipot Settings
#define TXT_NEW_STEPPER_MOTOR_MAX_CURRENTS_SET_CRLF				"-- New stepper motor maximum currents have been set.\r\n"
#define TXT_MAX_MOTOR_CURRENTS_CRLF_M906_X_Y_Z_E_CRLF			"-- Max Motor Currents:\r\n-- M906 X=%dmA, Y=%dmA, Z=%dmA, E=%dmA\n Current Sense Resistance=%dmOhm\r\n"

// Microstepping Setting
#define TXT_MAX_MOTOR_CURRENTS_CRLF_M907_X_Y_Z_E_CRLF			"-- Microstep resolution:\r\n-- M907 X=%d Y=%d Z=%d E=%d\r\n"

// Autoprint
#define TXT_M31_AUTOPRINT_ENABLED_CRLF                          "Autoprint from SD card is enabled!\n"
#define TXT_M31_AUTOPRINT_DISABLED_CRLF                         "Autoprint from SD card is disabled!\n"
#define TXT_START_AUTOPRINT_CRLF                                "SD card inserted, Start autoprint now!\n"
#define TXT_END_AUTOPRINT_CRLF                                  "SD card removed, Autoprint ended!\n"
#define TXT_ERROR_AUTOPRINT_CRLF                                "SD card error, cannot find autoprint file\n"

// Debug Information
#define TXT_STEPPER_TIMER_INIT_CRLF								"/" "/ Stepper Timer init\r\n"
#define TXT_FREE_RAM_CRLF										"/" "/ Free Ram: %d\r\n"
#define TXT_RS_SEQNUM_CHECKSUM_OUT_OF_RANGE_CRLF				"rs %ld (checksum out of range)\r\n"				// *** Host / System Text String
#define TXT_RS_SEQNUM_INCORRECT_CHECKSUM_SHOULD_BE_CRLF			"rs %ld (incorrect checksum - should be %u)\r\n"	// *** Host / System Text String
#define TXT_RS_SEQNUM_COMMAND_CODE_MISSING_CRLF					"rs %ld (command code missing): %s\r\n"				// *** Host / System Text String
#define TXT_RS_SEQNUM_COMMAND_CODE_OUT_OF_RANGE_CRLF			"rs %ld (command code out of range)\r\n"			// *** Host / System Text String
#define TXT_FIRMWARE_NAME_STR_CRLF								"FIRMWARE_NAME: 5DPrint PROTOCOL_VERSION:1.0 MACHINE_TYPE:Mendel EXTRUDER_COUNT:1\r\n"

#ifdef PRINTRBOARD_REVB
#define TXT_5DPRINT_FIRMWARE_VERSION_STR_CRLF					"/" "/ 5DPrint Firmware Version: %s\r\n// Printrboard Rev. B\r\n"
#elif  _5DPD8
#define TXT_5DPRINT_FIRMWARE_VERSION_STR_CRLF					"/" "/ 5DPrint Firmware Version: %s\r\n// 5DPrint D8 Driver Board\r\n"
#endif

#define TXT_LAST_TIMER1_COMPA_VECT_ISR_EXE_TIME_LU_US_CRLF		"/" "/ Last TIMER1_COMPA_vect ISR Execution Time:  %lu us\r\n"
#define TXT_MIN_TIMER1_COMPA_VECT_ISR_EXE_TIME_LU_US_CRLF		"/" "/ MIN TIMER1_COMPA_vect ISR Execution Time:  %lu us\r\n"
#define TXT_MAX_TIMER1_COMPA_VECT_ISR_EXE_TIME_LU_US_CRLF		"/" "/ MAX TIMER1_COMPA_vect ISR Execution Time:  %lu us\r\n"
#define TXT_TIMER1_EXE_TIME_DEBUG_NOT_AVAILABLE_CRLF			"/" "/ Timer 1 execution time debug info not availale in this version of firmaware.\r\n"
#define TXT_TIMER1_COMPA_VECT_ISR_EXE_TIME_MIN_MAX_RESET_CRLF	"/"	"/ TIMER1_COMPA_vect ISR Execution Time MIN / MAX Reset.\r\n"
#define TXT_CURRENT_CPU_LOADING_INT_PERCENT_CRLF				"/"	"/ Current CPU Loading:	%d %%\r\n"
#define TXT_PEAK_CPU_LOAD_INT_PERCENT_CRLF						"/"	"/ Peak CPU Load:		%d %%\r\n"
#define TXT_AVERAGE_CPU_LOAD_INT_PERCENT_CRLF					"/" "/ Average CPU Load: 	%d %%\r\n"
#define TXT_CPU_LOADING_INFO_NOT_AVAIL_IN_THIS_VER_CRLF			"/"	"/ CPU loading info not available in this version of firmware.\r\n"
#define TXT_PEAK_N_AVERAGE_CPU_LOAD_VALUES_RESET_CRLF			"/"	"/ Peak and Average CPU Load Values Reset.\r\n"
#define TXT_CRLF_5DPRINT_BOOTLOADER_CRLF						"\r\n/" "/ 5DPrint Bootloader\r\n"
#define TXT_ENTERING_BOOTLOADER_CRLF							"/" "/ ENTERING BOOTLOADER...\r\n"
#define TXT_CANNOT_ENTER_BOOTLOADER_INCORRECT_PASSCODE_CRLF		"/" "/ *** CAN NOT Enter Bootloader - Incorrect Pass Code!\r\n"
#define TXT_PLEASE_TRY_AGAIN_WITH_CORRECT_PASSCODE_CRLF			"/" "/ Please try again with correct pass code.\r\n\r\n"
#define TXT_CANNOT_ENTER_BOOTLOADER_PASSCODE_NOT_FOUND_CRLF		"/"	"/ *** CAN NOT Enter Bootloader - Pass code not found!\r\n"
#define TXT_CRLF_EMERGENCY_STOP_CRLF							"\r\n/" "/ Emergency Stop!\r\n"
#define TXT_CRLF_RESET_FLAGS_HEX_SPACE							"\r\n/" "/ Reset Flags: 0x%X "
#define TXT_JTAG_SPACE											"JTAG "
#define TXT_WDT_SPACE											"WDT "
#define TXT_BOR_SPACE											"BOR "
#define TXT_EXT_SPACE											"EXT "
#define TXT_POR_SPACE											"POR "

// End of (LANGUAGE == 0), ENGLISH

#else
// Generate error for compiler
#error "ERROR with language selection (in language.h)."
#endif	// #if (LANGUAGE == x)

#endif	// #ifndef _LANGUAGE_H_

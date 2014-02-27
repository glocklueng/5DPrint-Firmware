/*
  5D Print Firmware
  Designed for Printrboard (Rev B) and 5D Print D8 Driver Board.
  ---
  Copyright (c) 2012-2014 by Makible Limited.
 
  This file is part of the 5D Print Firmware.
 
  5D Print Firmware is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  The 5D Print Firmware is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with the 5D Print Firmware.  If not, see <http://www.gnu.org/licenses/>.
*/
/**
   \file makibox_sdcard.h
   \brief Header file for makibox_sdcard.c
   
 */
#ifndef MAKIBOX_SDCARD_H
#define MAKIBOX_SDCARD_H

// Roland Riegel SD Card library includes
#include "../sdcard/fat.h"
#include "../sdcard/fat_config.h"
#include "../sdcard/partition.h"
#include "../sdcard/sd_raw.h"
#include "../sdcard/sd_raw_config.h"

#define LS_FILENAME_ONLY		0x00
#define LS_SIZE					0x01
#define LS_SHOW_DIRS			0x02

#if SDSUPPORT > 0
// SD Card Variables
extern struct fat_fs_struct* sdcard_fs;
extern struct fat_file_struct* sdcard_fd;
extern struct fat_dir_struct* sdcard_dir_desc;

// Function Prototypes
void sdcard_initialise(void);
uint8_t print_disk_info(const struct fat_fs_struct* fs);
void sdcard_list_root(unsigned char flags);
void sdcard_release(void);
uint8_t sdcard_openFile(const char* filename);
intptr_t sdcard_file_read(struct fat_file_struct* fd, uint8_t* buffer, uintptr_t buffer_len);
void sdcard_closeFile(struct fat_file_struct* fd);
unsigned long sdcard_get_filesize(struct fat_file_struct* fd);
unsigned long sdcard_get_current_file_position(struct fat_file_struct* fd);
uint8_t sdcard_create_file(const char* filename);
intptr_t sdcard_write_file(struct fat_file_struct* fd, const uint8_t* buffer, uintptr_t buffer_len);
uint8_t sdcard_file_goto_eof(struct fat_file_struct* fd);
uint8_t sdcard_file_goto_sof(struct fat_file_struct* fd);
uint8_t sdcard_delete_file(const char* filename);
	
#endif // SDSUPPORT

#endif // MAKIBOX_SDCARD_H

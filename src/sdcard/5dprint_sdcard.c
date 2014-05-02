/*
  5DPrint Firmware
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
   \file 5dprint_sdcard.c
   \brief 5DPrint SD card handler
   
 */

#include "../config.h"

#if SDSUPPORT > 0

#include <string.h>

#include "5dprint_sdcard.h"
#include "usb.h"
#include "language.h"


static struct partition_struct* sdcard_partition;
struct fat_dir_struct* sdcard_dir_desc;
struct fat_file_struct* sdcard_fd;

uint8_t find_file_in_dir(struct fat_fs_struct* fs, struct fat_dir_struct* dd, const char* name, struct fat_dir_entry_struct* dir_entry);
struct fat_file_struct* open_file_in_dir(struct fat_fs_struct* fs, struct fat_dir_struct* dd, const char* name);



void sdcard_initialise(void)
{
    if( !sd_raw_init() )
	{
            serial_send(TXT_SD_CARD_INIT_FAILED_CRLF);
            //break;
	}
    else
	{		
            serial_send(TXT_SD_CARD_INITIALISED_CRLF);
	}

    /* open first partition */
    if(!sdcard_partition)
	{
            sdcard_partition = partition_open(sd_raw_read,
                                              sd_raw_read_interval,
#if SD_RAW_WRITE_SUPPORT
                                              sd_raw_write,
                                              sd_raw_write_interval,
#else
                                              0,
                                              0,
#endif
                                              0
                                              );
	}

    if(!sdcard_partition)
	{
            /* If the partition did not open, assume the storage device
             * is a "superfloppy", i.e. has no MBR.
             */
            sdcard_partition = partition_open(sd_raw_read,
                                              sd_raw_read_interval,
#if SD_RAW_WRITE_SUPPORT
                                              sd_raw_write,
                                              sd_raw_write_interval,
#else
                                              0,
                                              0,
#endif
                                              -1
                                              );
            if(!sdcard_partition)
		{
                    serial_send(TXT_FAILED_TO_OPEN_PARTITION_CRLF);
                    return;
		}
	}

    /* open file system */
    if (!sdcard_fs)
	{
            sdcard_fs = fat_open(sdcard_partition);
	}
	
    if (!sdcard_fs)
	{
            serial_send(TXT_OPENING_FILESYSTEM_FAILED_CRLF);
            return;
	}
}


uint8_t print_disk_info(const struct fat_fs_struct* fs)
{
    //configure_ss_pin_as_output();
    //sd_raw_init(); // initialise to ensure SPI is set in master mode
	
    if(!fs)
        return 0;

    struct sd_raw_info disk_info;
    if(!sd_raw_get_info(&disk_info))
        return 0;

    serial_send(TXT_MANUFACTURER_HEX_CRLF, disk_info.manufacturer);
    serial_send(TXT_OEM_STR_CRLF, (char*) disk_info.oem);
    serial_send(TXT_PRODUCT_STR_CRLF, (char*) disk_info.product);
    serial_send(TXT_REVISION_HEX_CRLF, disk_info.revision);
    serial_send(TXT_SERIAL_HEX_CRLF, (unsigned int)(disk_info.serial));
    serial_send(TXT_DATE_INT_INT_CRLF, disk_info.manufacturing_month, disk_info.manufacturing_year);
    serial_send(TXT_SIZE_INT_MB_CRLF, (int)(disk_info.capacity / 1024.0 / 1024.0));
    serial_send(TXT_COPY_INT_CRLF, disk_info.flag_copy);
    serial_send(TXT_WRITE_PROTECT_INT_INT_CRLF, disk_info.flag_write_protect_temp, disk_info.flag_write_protect);
    serial_send(TXT_FORMAT_INT_CRLF, disk_info.format);
    serial_send(TXT_FREE_SPACE_LU_LU_BYTES_CRLF, (unsigned long)(fat_get_fs_free(fs)), (unsigned long)(fat_get_fs_size(fs)));

    return 1;
}


void sdcard_release(void)
{
    //sd_raw_init(); // initialise to ensure SPI is set in master mode
	
    /* close any open file */
    if (sdcard_fd)
	{
            fat_close_file(sdcard_fd);
	}
	
    /* close directory */
    if (sdcard_dir_desc)
	{
            fat_close_dir(sdcard_dir_desc);
		
            sdcard_dir_desc = 0;
	}
	
    if (sdcard_fs)
	{
            // close file system
            fat_close(sdcard_fs);
            sdcard_fs = 0;
	}
	
    if (sdcard_partition)
	{
            // close partition
            partition_close(sdcard_partition);
            sdcard_partition = 0;
	}
	
    serial_send(TXT_SD_CARD_RELEASED_CRLF);
}


void sdcard_list_root(unsigned char flags)
{
    struct fat_dir_entry_struct dir_entry;
    struct fat_dir_entry_struct directory;
	
    //sd_raw_init(); // initialise to ensure SPI is set in master mode
	
    if (sdcard_fs)
	{
            if (!sdcard_dir_desc)
		{
                    fat_get_dir_entry_of_path(sdcard_fs, "/", &directory);
                    sdcard_dir_desc = fat_open_dir(sdcard_fs, &directory);
		}
		
            if (!sdcard_dir_desc)
		{
                    serial_send(TXT_OPENING_DIRECTORY_FAILED_CRLF);
		}
		
            /* print directory listing */
            while(fat_read_dir(sdcard_dir_desc, &dir_entry))
		{
                    uint8_t spaces = sizeof(dir_entry.long_name) - strlen(dir_entry.long_name) + 4;
			
                    if (flags & LS_SHOW_DIRS)
			{
                            serial_send(TXT_STR, dir_entry.long_name);
                            serial_send(TXT_STR, (dir_entry.attributes & FAT_ATTRIB_DIR ? "/" : ""));
			}
                    else
			{
                            if ( !(dir_entry.attributes & FAT_ATTRIB_DIR) )
				{
                                    serial_send(TXT_STR, dir_entry.long_name);
				}
			}
			
                    if (flags & LS_SIZE)
			{
                            if ( !(dir_entry.attributes & FAT_ATTRIB_DIR) )
				{
                                    while(spaces--)
					{
                                            serial_send(TXT_SPACE);
					}
				
                                    serial_send(TXT_LU, dir_entry.file_size);
				}
			}
			
                    if ( !(flags & LS_SHOW_DIRS)
                         &&  !(dir_entry.attributes & FAT_ATTRIB_DIR) )
			{
                            serial_send(TXT_CRLF);
			}
                    else if (flags & LS_SHOW_DIRS)
			{
                            serial_send(TXT_CRLF);
			}
		}
		
            fat_close_dir(sdcard_dir_desc);
            sdcard_dir_desc = 0;
	}
}


uint8_t sdcard_openFile(const char* filename)
{
    struct fat_dir_entry_struct directory;
	
    if (!sdcard_fs)
	{
            serial_send(TXT_SD_CARD_FILE_SYSTEM_NOT_INITIALISED_CRLF);
            return 0;
	}
	
    if(!sdcard_dir_desc)
	{
            fat_get_dir_entry_of_path(sdcard_fs, "/", &directory);
            sdcard_dir_desc = fat_open_dir(sdcard_fs, &directory);
	}
	
    if(!sdcard_dir_desc)
	{
            serial_send(TXT_OPENING_DIRECTORY_FAILED_CRLF);
            return 0;
	}
	
    if (!sdcard_fd)
	{
            sdcard_fd = open_file_in_dir(sdcard_fs, sdcard_dir_desc, filename);
	}
	
    if (!sdcard_fd)
	{
            return 0;
	}
    else
	{
            return 1;
	}
}


uint8_t find_file_in_dir(struct fat_fs_struct* fs, struct fat_dir_struct* dd, const char* name, struct fat_dir_entry_struct* dir_entry)
{
    while(fat_read_dir(dd, dir_entry))
        {
            if(strcmp(dir_entry->long_name, name) == 0)
                {
                    fat_reset_dir(dd);
                    return 1;
                }
        }

    return 0;
}


struct fat_file_struct* open_file_in_dir(struct fat_fs_struct* fs, struct fat_dir_struct* dd, const char* name)
{
    struct fat_dir_entry_struct file_entry;
    if(!find_file_in_dir(fs, dd, name, &file_entry))
        return 0;

    return fat_open_file(fs, &file_entry);
}


void sdcard_closeFile(struct fat_file_struct* fd)
{
    fat_close_file(fd);
}


intptr_t sdcard_file_read(struct fat_file_struct* fd, uint8_t* buffer, uintptr_t buffer_len)
{
    return fat_read_file(fd, buffer, buffer_len);
}


unsigned long sdcard_get_filesize(struct fat_file_struct* fd)
{
    return (fd->dir_entry.file_size);
}


unsigned long sdcard_get_current_file_position(struct fat_file_struct* fd)
{
    return (fd->pos);
}


uint8_t sdcard_create_file(const char* filename)
{
    struct fat_dir_entry_struct directory;
    struct fat_dir_entry_struct file_entry;
	
    if (!sdcard_fs)
	{
            serial_send(TXT_SD_CARD_FILE_SYSTEM_NOT_INITIALISED_CRLF);
            return 0;
	}
	
    if(!sdcard_dir_desc)
	{
            fat_get_dir_entry_of_path(sdcard_fs, "/", &directory);
            sdcard_dir_desc = fat_open_dir(sdcard_fs, &directory);
	}
	
    if(!sdcard_dir_desc)
	{
            serial_send(TXT_OPENING_DIRECTORY_FAILED_CRLF);
            return 0;
	}
	
    return ( fat_create_file(sdcard_dir_desc, filename, &file_entry) );
}



uint8_t sdcard_delete_file(const char* filename)
{
    struct fat_dir_entry_struct directory;
    struct fat_dir_entry_struct file_entry;
	
    if (!sdcard_fs)
	{
            serial_send(TXT_SD_CARD_FILE_SYSTEM_NOT_INITIALISED_CRLF);
            return 0;
	}
	
    if(!sdcard_dir_desc)
	{
            fat_get_dir_entry_of_path(sdcard_fs, "/", &directory);
            sdcard_dir_desc = fat_open_dir(sdcard_fs, &directory);
	}
	
    if(!sdcard_dir_desc)
	{
            serial_send(TXT_OPENING_DIRECTORY_FAILED_CRLF);
            return 0;
	}
	
    if( find_file_in_dir(sdcard_fs, sdcard_dir_desc, filename, &file_entry) )
	{
            if(fat_delete_file(sdcard_fs, &file_entry))
		{
                    serial_send(TXT_DELETED_FILE_STR_CRLF, filename);
                    return 1;
		}
            else
		{
                    serial_send(TXT_COULD_NOT_DELETE_STR_FILE_DELETE_FAILED_CRLF, filename);
                    return 0;
		}
	}
	
    serial_send("-- Delete failed.\r\n");
    return 0;
}


/*
 * Return:	The number of bytes written (0 or something less than buffer_len 
 *			on disk full) or -1 on failure.
 */
intptr_t sdcard_write_file(struct fat_file_struct* fd, const uint8_t* buffer, uintptr_t buffer_len)
{
    return ( fat_write_file(fd, (uint8_t*) buffer, buffer_len) );
}

/*
 * Move file position to end of file
 * Return:	0 = failure
 *			1 = success
 */
uint8_t sdcard_file_goto_eof(struct fat_file_struct* fd)
{
    return ( fat_seek_file(fd, 0, FAT_SEEK_END) );
}

/*
 * Move file position to start of file
 * Return:	0 = failure
 *			1 = success
 */
uint8_t sdcard_file_goto_sof(struct fat_file_struct* fd)
{
    return ( fat_seek_file(fd, 0, FAT_SEEK_SET) );
}

#endif

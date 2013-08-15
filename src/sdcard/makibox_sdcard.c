

#include "../config.h"

#ifdef SDSUPPORT

#include <string.h>

// Roland Riegel SD Card library includes
#include "../sdcard/fat.h"
#include "../sdcard/fat_config.h"
#include "../sdcard/partition.h"
#include "../sdcard/sd_raw.h"
#include "../sdcard/sd_raw_config.h"

#include "../sdcard/makibox_sdcard.h"
#include "../usb.h"


static struct partition_struct* sdcard_partition;
static struct fat_dir_struct* sdcard_dir_desc;


void sdcard_initialise(void)
{
	if( !sd_raw_init() )
	{
		serial_send("-- *** SD Card Initialisation Failed.\r\n");
		//break;
	}
	else
	{		
		serial_send("-- SD Card Initialised.\r\n");
	}

	/* open first partition */
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
			serial_send("-- *** Failed to open partition.\r\n");
			return;
		}
	}

	/* open file system */
	sdcard_fs = fat_open(sdcard_partition);
	if(!sdcard_fs)
	{
		serial_send("-- *** Opening filesystem failed\r\n");
		return;
	}

}


uint8_t print_disk_info(const struct fat_fs_struct* fs)
{
	//configure_ss_pin_as_output();
	sd_raw_init(); // initialise to ensure SPI is set in master mode
	
    if(!sdcard_fs)
        return 0;

    struct sd_raw_info disk_info;
    if(!sd_raw_get_info(&disk_info))
        return 0;

    serial_send("-- Manufacturer: 0x%04X\r\n", disk_info.manufacturer);
    serial_send("-- OEM: %s\r\n", (char*) disk_info.oem);
    serial_send("-- Product: %s\r\n", (char*) disk_info.product);
    serial_send("-- Revision:  0x%04X\r\n", disk_info.revision);
    serial_send("-- Serial: 0x%X\r\n", (unsigned int)(disk_info.serial));
    serial_send("-- Date: %02d / %02d\r\n", disk_info.manufacturing_month, disk_info.manufacturing_year);
    serial_send("-- Size: %dMB\r\n", (int)(disk_info.capacity / 1024.0 / 1024.0));
    serial_send("-- Copy: %d\r\n", disk_info.flag_copy);
    serial_send("-- Write Protect: %d / %d\r\n", disk_info.flag_write_protect_temp, disk_info.flag_write_protect);
    serial_send("-- Format: %d\r\n", disk_info.format);
    serial_send("-- Free Space: %lu / %lu Bytes\r\n", (unsigned long)(fat_get_fs_free(fs)), (unsigned long)(fat_get_fs_size(fs)));

    return 1;
}


void sdcard_release(void)
{
	sd_raw_init(); // initialise to ensure SPI is set in master mode
	
	/* close directory */
	if (sdcard_dir_desc != 0)
	{
		fat_close_dir(sdcard_dir_desc);
	}
	
	if (sdcard_fs != 0)
	{
		// close file system
		fat_close(sdcard_fs);

		// close partition
		partition_close(sdcard_partition);
	}
	
	serial_send("-- SD Card Released.\r\n");
}


void sdcard_list_root(void)
{
	struct fat_dir_entry_struct dir_entry;
	struct fat_dir_entry_struct directory;
	
	sd_raw_init(); // initialise to ensure SPI is set in master mode
	
	if (sdcard_fs != NULL)
	{
		fat_get_dir_entry_of_path(sdcard_fs, "/", &directory);

		sdcard_dir_desc = fat_open_dir(sdcard_fs, &directory);
		
		if(!sdcard_dir_desc)
		{
			serial_send("-- Opening directory failed.\r\n");
		}
		
		/* print directory listing */
		while(fat_read_dir(sdcard_dir_desc, &dir_entry))
		{
			uint8_t spaces = sizeof(dir_entry.long_name) - strlen(dir_entry.long_name) + 4;

			serial_send("-- %s", dir_entry.long_name);
			serial_send("%s", (dir_entry.attributes & FAT_ATTRIB_DIR ? "/" : " "));
			
			while(spaces--)
			{
				serial_send(" ");
			}
			
			serial_send("%lu\r\n", dir_entry.file_size);
		}
		
		fat_close_dir(sdcard_dir_desc);
	}
}
#endif
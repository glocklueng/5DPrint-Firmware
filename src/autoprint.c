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
/**
   \file autoprint.c
   \brief Autoprinting when SD card is plugged in
*/

#include "config.h"
#include "sdcard/makibox_sdcard.h"
#include "makibox.h"
#include "usb.h"
#include "language.h"
#include <string.h>
#include <ctype.h>

#if AUTOPRINT > 0

// Definitions
#define MAX_CMD_SIZE 95

// Local variables
const char autoprint_filename1[] = "autoprint.g";
const char autoprint_filename2[] = "autoprint.gcode";
char autoprint_filename[64] = ""; // size same as long_name
static char sdcard_cmdbuf[MAX_CMD_SIZE + 1];
static unsigned char sdcard_bufpos = 0;
static unsigned char sdcard_ignore_comments = 0;

// Local functions prototypes
void autoPrintSuccess();
void autoPrintError();
void startSDprint();
void finishSDprint();
uint8_t autoPrintFileCheck();
uint8_t autoPrintCommandCheck(const char *cmdstr);
uint8_t autoPrintFindFile();
uint8_t autoPrintFileNameCheck(const char *long_name);

// Extern function

/**
   \fn void autoprint()
   \brief Main autoprint function routine to perform SD print automatically
   Manages the SDAutoPrintStatus based on following criteria
   - SD card availability
   - SD Printing status

   SDAutoPrintStatus:
   - 0: Not printed, 
   - 1: Prinitng, 
   - 2:  Finish Print, 
   - 3: Print finished and waiting for new SD print,
       Error reported, waiting for new SD card
   */
void autoprint(){
    static unsigned char SDAutoPrintStatus = 0;
    //serial_send ("%d", SDAutoPrintStatus);
    switch (SDAutoPrintStatus){
    case 0:
        // Check whether are SD card and valid file
        if ( sd_raw_available() ){
            sdcard_initialise();
            if (autoPrintFindFile()){
                if (autoPrintFileCheck()){	  
                    autoPrintSuccess();
                    SDAutoPrintStatus = 1;
                }
                else{
                    autoPrintError();
                    SDAutoPrintStatus = 3;
                    // Disable motors
                    process_command("M18");
                }
            }
            else{
                autoPrintError();
                SDAutoPrintStatus = 3;
                // Disable motors
                process_command("M18");
            }
        }
        break;
    case 1:
        // Start print
        startSDprint();
        SDAutoPrintStatus = 2;
        break;
    case 2:
        // Finish print and turn off stuff
        if (!sdcard_print){
            finishSDprint();
            SDAutoPrintStatus = 3;
        }
        break;
    case 3:
        // Finish a print and waiting for card being swapped
        // Or error reported, waiting for new SD card
        if (!sd_raw_available()) SDAutoPrintStatus = 0;
        break;    
    default:
        SDAutoPrintStatus = 0;
        break;
    }
}


// Local functions

/**
   \fn void startSDprint()
   \brief Reset file position and start print

*/
void startSDprint(){
    sdcard_fd->pos = 0;   // Reset file position
    process_command("M24");  // Start print
}


/**
   \fn void finishSDprint()
   \brief Release SD card and reset the printer

*/
void finishSDprint(){
    process_command("G91");
    process_command("G1 Z10"); // Lower Z to make it easier to remove the print
    process_command("G28 X0 Y0"); // Home X and Y
    process_command("G90");   
    sdcard_release();
    process_command("M18");   // Disable motors
    process_command("M104 S0");  // Disable hot end
    process_command("M140 S0");   // Disable hot bed
}


/**
   \fn void autoPrintSuccess()
   \brief Success card read notification using stepper motors

*/
void autoPrintSuccess(){
#ifdef MAKIBOX_5DPD8
    process_command("M300 F100 P1000");
#endif
#ifdef PRINTRBOARD_REVB
    process_command("G91");
    process_command("G1 F200");
    process_command("G1 X0.4");
    for (int i=0; i<30; i++){
        process_command("G1 X0.1");
        process_command("G1 X-0.1");
    }
    process_command("G90");
#endif
}


/**
   \fn void autoPrintError()
   \brief Error in card read notification using stepper motors

*/
void autoPrintError(){
#ifdef MAKIBOX_5DPD8
    process_command("M300 F4000 P1000");
#endif
#ifdef PRINTRBOARD_REVB
    process_command("G91");
    process_command("G1 F1000");
    process_command("G1 X0.4");
    for (int i=0; i<50; i++){
        process_command("G1 X0.1");
        process_command("G1 X-0.1");
    }
    process_command("G90");
#endif
}


/**
   \fn uint8_t autoPrintFileCheck()
   \brief Attempts to open the autoprint file and checks for G28 for validation of file

*/
uint8_t autoPrintFileCheck(){
    if (!sdcard_openFile(autoprint_filename)) {
        serial_send(TXT_STR, "Failed to open the right file for autoprint\n");
        serial_send(TXT_STR, autoprint_filename);
        serial_send(TXT_CRLF);
        return 0;
    }
    uint8_t sd_ch[32];
    int16_t bytes_read = -1;
    uint8_t i;
    bytes_read = sdcard_file_read(sdcard_fd, sd_ch, sizeof(sd_ch));
    while (bytes_read != 0){
        for (i = 0; i < bytes_read; i++){
            if ((char)sd_ch[i] == ';') sdcard_ignore_comments = 1;
            if ( !((char)sd_ch[i] < 0 || (char)sd_ch[i] > 255) ){
                if ((char)sd_ch[i] == '\n' || (char)sd_ch[i] == '\r'){
                    // Newline marks end of this command;  terminate
                    // string and process it.
                    sdcard_cmdbuf[sdcard_bufpos] = '\0';
                    if (sdcard_bufpos > 0) {
                        if (autoPrintCommandCheck(sdcard_cmdbuf)) return 1;
                    }
                    sdcard_ignore_comments = 0;
                    sdcard_bufpos = 0;
                    sdcard_cmdbuf[sdcard_bufpos] = '\0';
                }
                else{
                    if (sdcard_ignore_comments < 1) sdcard_cmdbuf[sdcard_bufpos++] = (char)sd_ch[i];
                    if (sdcard_bufpos > MAX_CMD_SIZE - 1)	sdcard_bufpos--;
                }
            }
        } // for
        bytes_read = sdcard_file_read(sdcard_fd, sd_ch, sizeof(sd_ch));
    } // while

    sdcard_closeFile(sdcard_fd);
    return 0;
}

/**
   \fn uint8_t autoPrintCommandCheck(const char *cmdstr)
   \brief Checkes whether cmdstr is G28

*/
uint8_t autoPrintCommandCheck(const char *cmdstr){
    char ch;
    int16_t pos = -1;
    const char homeCmd[] = "G28";
    uint8_t homeI = 0;

    while ((ch = cmdstr[++pos]) != '\0'){
        if (ch == homeCmd[homeI]){
            if (++homeI == 3) return 1;
        }
    }
    return 0;
}

/**
   \fn uint8_t autoPrintFindFile()
   \brief List direcotries and find any files that satisfies specific criterias

*/
uint8_t autoPrintFindFile(){
    struct fat_dir_entry_struct dir_entry;
    struct fat_dir_entry_struct directory;
    if (sdcard_fs){
        if (!sdcard_dir_desc){
            fat_get_dir_entry_of_path(sdcard_fs, "/", &directory);
            sdcard_dir_desc = fat_open_dir(sdcard_fs, &directory);
        }
        if (!sdcard_dir_desc) return 0;    
        /* print directory listing */
        while(fat_read_dir(sdcard_dir_desc, &dir_entry)){
            //serial_send(TXT_STR, dir_entry.long_name);
            //serial_send(TXT_STR, (dir_entry.attributes & FAT_ATTRIB_DIR ? "/" : ""));
            //serial_send(TXT_CRLF);
            if ( autoPrintFileNameCheck(dir_entry.long_name) ) {
                fat_close_dir(sdcard_dir_desc);
                sdcard_dir_desc = 0;
                return 1;
            }
        }    
        fat_close_dir(sdcard_dir_desc);
        sdcard_dir_desc = 0;
    }
    return 0;
}


/**
   \fn uint8_t autoPrintFileNameCheck(const char *long_name)
   \brief Checks whether long_name satisfies the fielanem criteria

   File name needs extension .g or .gcode
   File name is case insensitive as long it is autoprint
   Example file names: 
   autoprint.g
   AuToPrInT.GcOdE
*/
uint8_t autoPrintFileNameCheck(const char *long_name){
    char ch;
    int16_t pos = -1;
    char tmp_filename[64] = "";
    strcpy(tmp_filename, long_name);
    while ((ch = long_name[++pos]) != '\0') tmp_filename[pos] = tolower(ch);
    if (strcmp(autoprint_filename1, tmp_filename) || strcmp(autoprint_filename2, tmp_filename) ){
        strcpy(autoprint_filename, long_name);
        //serial_send(TXT_STR, tmp_filename);
        //serial_send(TXT_STR, autoprint_filename);
        return 1;
    }
    return 0;
}

#endif

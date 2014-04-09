 # 5DPrint Firmware
 # Based on Sprinter (master branch, 1 Sep 2012).
 # Designed for Printrboard (Rev B) and 5DPrint D8 Driver Board.
 # ---
 # Copyright (c) 2012-2013 by Makible Limited.
 # 
 # This file is part of the 5DPrint Firmware.
 # 
 # 5DPrint Firmware is free software: you can redistribute it and/or modify
 # it under the terms of the GNU General Public License as published by
 # the Free Software Foundation, either version 3 of the License, or
 # (at your option) any later version.
 # 
 # The 5DPrint Firmware is distributed in the hope that it will be useful,
 # but WITHOUT ANY WARRANTY; without even the implied warranty of
 # MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 # GNU General Public License for more details.
 # 
 # You should have received a copy of the GNU General Public License
 # along with the 5DPrint Firmware.  If not, see <http://www.gnu.org/licenses/>.
 #
 # Firmware is compatible with the following hardware (default build is for #1)
 # 1. 5DPrint D8 Driver Board
 # 2. Printrboard Rev. B
 #
 # To build for PrinterBoard, use the following command:
 # 		make HARDWARE=PRINTRBOARD_REVB

MCU=at90usb1286
F_CPU=16000000


5DPRINT_OBJS   = arc_func.o heater.o main.o 5dprint.o pins_teensy.o \
                 planner.o stepper.o store_eeprom.o usb.o \
				 sdcard/byteordering.o sdcard/fat.o sdcard/partition.o \
				 sdcard/sd_raw.o sdcard/5dprint_sdcard.o \
				 i2c/TWI_Master.o i2c/Master_I2C_Comms.o \
				 tone.o autoprint.o gpio.o

CC=avr-gcc
OBJCOPY=avr-objcopy
HARDWARE=_5DPD8

CPPFLAGS       = -mmcu=$(MCU) -DF_CPU=$(F_CPU)L -Os \
                 -ffunction-sections -fdata-sections -g \
                 -Wall -Wformat=2 -Werror \
		 -D$(HARDWARE)
CFLAGS         = -std=gnu99
LDFLAGS        = -mmcu=$(MCU) -Wl,-Map=5dprint.map,--gc-sections -Os


%.o: %.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@


5DPRINT_OBJS:=$(addprefix src/, $(5DPRINT_OBJS))


all: 5dprint.hex

clean:
	rm -f $(5DPRINT_OBJS)
	rm -f 5dprint.elf
	rm -f 5dprint.hex

5dprint: $(5DPRINT_OBJS)

5dprint.elf: $(5DPRINT_OBJS)
	$(CC) $(LDFLAGS) -o $@ $^ -lm -lc
#	@(printf "Program size:\n")
#	@(printf "  .text  %8d bytes\n" 0x$$(readelf -S $@ | grep '\.text' | cut -c 58-63))
#	@(printf "  .data  %8d bytes\n" 0x$$(readelf -S $@ | grep '\.data' | cut -c 58-63))
#	@(printf "  .bss   %8d bytes\n" 0x$$(readelf -S $@ | grep '\.bss' | cut -c 58-63))

	avr-size 5dprint.elf 

5dprint.hex: 5dprint.elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

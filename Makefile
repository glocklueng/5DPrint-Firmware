 # Makibox A6 Firmware
 # Based on Sprinter (master branch, 1 Sep 2012).
 # Designed for Printrboard (Rev B).
 # ---
 # Copyright (c) 2012-2013 by Makible Limited.
 # 
 # This file is part of the Makibox A6 Firmware.
 # 
 # Makibox A6 Firmware is free software: you can redistribute it and/or modify
 # it under the terms of the GNU General Public License as published by
 # the Free Software Foundation, either version 3 of the License, or
 # (at your option) any later version.
 # 
 # The Makibox A6 Firmware is distributed in the hope that it will be useful,
 # but WITHOUT ANY WARRANTY; without even the implied warranty of
 # MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 # GNU General Public License for more details.
 # 
 # You should have received a copy of the GNU General Public License
 # along with the Makibox A6 Firmware.  If not, see <http://www.gnu.org/licenses/>.

MCU=at90usb1286
F_CPU=16000000


MAKIBOX_OBJS   = arc_func.o heater.o main.o makibox.o pins_teensy.o \
                 planner.o stepper.o store_eeprom.o usb.o \
				 sdcard/byteordering.o sdcard/fat.o sdcard/partition.o \
				 sdcard/sd_raw.o sdcard/makibox_sdcard.o


CC=avr-gcc
OBJCOPY=avr-objcopy


CPPFLAGS       = -mmcu=$(MCU) -DF_CPU=$(F_CPU)L -Os \
                 -ffunction-sections -fdata-sections -g \
                 -Wall -Wformat=2 -Werror
CFLAGS         = -std=gnu99
LDFLAGS        = -mmcu=$(MCU) -Wl,-Map=makibox.map,--gc-sections -Os


%.o: %.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@


MAKIBOX_OBJS:=$(addprefix src/, $(MAKIBOX_OBJS))


all: makibox.hex

clean:
	rm -f $(MAKIBOX_OBJS)
	rm -f makibox.elf
	rm -f makibox.hex

makibox: $(MAKIBOX_OBJS)

makibox.elf: $(MAKIBOX_OBJS)
	$(CC) $(LDFLAGS) -o $@ $^ -lm -lc
#	@(printf "Program size:\n")
#	@(printf "  .text  %8d bytes\n" 0x$$(readelf -S $@ | grep '\.text' | cut -c 58-63))
#	@(printf "  .data  %8d bytes\n" 0x$$(readelf -S $@ | grep '\.data' | cut -c 58-63))
#	@(printf "  .bss   %8d bytes\n" 0x$$(readelf -S $@ | grep '\.bss' | cut -c 58-63))

	avr-size makibox.elf 

makibox.hex: makibox.elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

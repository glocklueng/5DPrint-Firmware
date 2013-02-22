

MCU=at90usb1286
F_CPU=16000000


MAKIBOX_OBJS   = arc_func.o heater.o main.o makibox.o pins_teensy.o \
                 planner.o stepper.o store_eeprom.o usb.o


CC=avr-gcc
OBJCOPY=avr-objcopy


CPPFLAGS       = -mmcu=$(MCU) -DF_CPU=$(F_CPU)L -Os \
                 -ffunction-sections -fdata-sections -g \
                 -Wall -Wformat=2 -Werror
CFLAGS         = -std=gnu99
LDFLAGS        = -mmcu=$(MCU) -Wl,--gc-sections -Os


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

makibox.hex: makibox.elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

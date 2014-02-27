/* 
   5D Print Firmware
   Based on Sprinter (master branch, 1 Sep 2012).
   Designed for Printrboard (Rev B) and 5D Print D8 Dirver Board.
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
   ---
   * Pin functions for the Teensy and Teensy++
   * http://www.pjrc.com/teensy/
   * Copyright (c) 2008-2010 PJRC.COM, LLC
   * 
   * Permission is hereby granted, free of charge, to any person obtaining a copy
   * of this software and associated documentation files (the "Software"), to deal
   * in the Software without restriction, including without limitation the rights
   * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   * copies of the Software, and to permit persons to whom the Software is
   * furnished to do so, subject to the following conditions:
   * 
   * The above copyright notice and this permission notice shall be included in
   * all copies or substantial portions of the Software.
   * 
   * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
   * THE SOFTWARE.
   */
/**
   \file pins_teensy.c
   \brief Old file inherited from arduino ancestors
   
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include "pgmspace.h"


#include "pins_teensy.h"


#ifdef ID
#undef ID  // ID bit in USBSTA conflicts with user's code
#endif


const static uint8_t A0 = CORE_ANALOG0_PIN;
const static uint8_t A1 = CORE_ANALOG1_PIN;
const static uint8_t A2 = CORE_ANALOG2_PIN;
const static uint8_t A3 = CORE_ANALOG3_PIN;
const static uint8_t A4 = CORE_ANALOG4_PIN;
const static uint8_t A5 = CORE_ANALOG5_PIN;
const static uint8_t A6 = CORE_ANALOG6_PIN;
const static uint8_t A7 = CORE_ANALOG7_PIN;
const static uint8_t SS   = CORE_SS0_PIN;
const static uint8_t MOSI = CORE_MOSI0_PIN;
const static uint8_t MISO = CORE_MISO0_PIN;
const static uint8_t SCK  = CORE_SCLK0_PIN;
const static uint8_t LED_BUILTIN = CORE_LED0_PIN;
#if defined(CORE_SDA0_PIN)
const static uint8_t SDA  = CORE_SDA0_PIN;
#endif
#if defined(CORE_SCL0_PIN)
const static uint8_t SCL  = CORE_SCL0_PIN;
#endif

#define NUM_DIGITAL_PINS                CORE_NUM_TOTAL_PINS
#define NUM_ANALOG_INPUTS               CORE_NUM_ANALOG


#define digitalPinToPort(P) (P)
#define portInputRegister(P) ((volatile uint8_t *)((int)pgm_read_byte(digital_pin_table_PGM+(P)*2+1)))
#define portModeRegister(P) (portInputRegister(P) + 1)
#define portOutputRegister(P) (portInputRegister(P) + 2)
#define digitalPinToBitMask(P) (pgm_read_byte(digital_pin_table_PGM+(P)*2))
extern const uint8_t digital_pin_table_PGM[] PROGMEM;

#define analogInputToDigitalPin(ch)	((ch) <= 7 ? (ch) - 38 : -1)
#define digitalPinHasPWM(p)		(((p) >= 14 && (p) <= 16) || ((p) >= 24 && (p) <= 27) || (p) == 0 || (p) == 1)
#define digitalPinToPortReg(p)                                          \
    (((p) >= 0 && (p) <= 7) ? &PORTD : (((p) >= 10 && (p) <= 17) ? &PORTC : \
                                        (((p) >= 20 && (p) <= 27) ? &PORTB : (((p) >= 28 && (p) <= 35) ? &PORTA : \
                                                                              (((p) >= 38 && (p) <= 45) ? &PORTF : &PORTE)))))
#define digitalPinToBit(p)                                              \
    (((p) <= 7) ? (p) : (((p) <= 9) ? (p) - 8 : (((p) <= 17) ? (p) - 10 : \
                                                 (((p) <= 19) ? (p) - 12 : (((p) <= 27) ? (p) - 20 : (((p) <= 35) ? (p) - 28 : \
                                                                                                      (((p) <= 37) ? (p) - 32 : (((p) <= 45) ? (p) - 38 : 2))))))))
#define digitalPinToPCICR(p)	(((p) >= 20 && (p) <= 27) ? &PCICR : NULL)
#define digitalPinToPCICRbit(p)	(0)
#define digitalPinToPCIFR(p)	(((p) >= 20 && (p) <= 27) ? &PCIFR : NULL)
#define digitalPinToPCIFRbit(p)	(0)
#define digitalPinToPCMSK(p)	(((p) >= 20 && (p) <= 27) ? &PCMSK0 : NULL)
#define digitalPinToPCMSKbit(p)	(((p) - 20) & 7)


#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif


uint8_t w_analog_reference = 0x40;

int analogRead(uint8_t pin)
{
    uint8_t low, high;

    if (pin >= PIN_F0 && pin <= PIN_F7) pin -= PIN_F0;
    if (pin < 8) {
        DIDR0 |= (1 << pin);
        //DDRF &= ~(1 << pin);
        //PORTF &= ~(1 << pin);
    }
    ADMUX = w_analog_reference | (pin & 0x1F);
    ADCSRA |= (1<<ADSC);
    while (ADCSRA & (1<<ADSC)) ;
    low = ADCL;
    high = ADCH;
    return (high << 8) | low;
}

#define PIN_REG_AND_MASK_LOOKUP(pin, reg, mask)                 \
    asm volatile(                                               \
                 "lsl %2"		"\n\t"                  \
                 "add %A3, %2"		"\n\t"                  \
                 "adc %B3, __zero_reg__"	"\n\n"          \
                 "lpm %1, Z+"		"\n\t"                  \
                 "lpm %A0, Z"		"\n\t"                  \
                 "ldi %B0, 0"		"\n"                    \
                 : "=z" (reg), "=r" (mask), "+r" (pin)          \
                 : "z" (digital_pin_table_PGM), "2" (pin))


static const uint8_t didr_table_PGM[] PROGMEM = {
    (int)&DIDR0, ~0x01,
    (int)&DIDR0, ~0x02,
    (int)&DIDR0, ~0x04,
    (int)&DIDR0, ~0x08,
    (int)&DIDR0, ~0x10,
    (int)&DIDR0, ~0x20,
    (int)&DIDR0, ~0x40,
    (int)&DIDR0, 0x7F  // ~0x80
};

#define PIN_DIDR_AND_MASK_LOOKUP(pin, didreg, didmask)	\
    asm volatile(					\
                 "lsl	%3"			"\n\t"	\
                 "add	%A2, %3"		"\n\t"	\
                 "adc	%B2, __zero_reg__"	"\n\n"	\
                 "lpm	%A0, Z+"		"\n\t"	\
                 "ldi	%B0, 0"			"\n\t"	\
                 "lpm	%1, Z+"			"\n\t"	\
                 : "=x" (didreg), "=r" (didmask)        \
                 : "z" (didr_table_PGM), "r" (pin))


const uint8_t digital_pin_table_PGM[] PROGMEM = {
    CORE_PIN0_BITMASK,	(int)&CORE_PIN0_PINREG,
    CORE_PIN1_BITMASK,	(int)&CORE_PIN1_PINREG,
    CORE_PIN2_BITMASK,	(int)&CORE_PIN2_PINREG,
    CORE_PIN3_BITMASK,	(int)&CORE_PIN3_PINREG,
    CORE_PIN4_BITMASK,	(int)&CORE_PIN4_PINREG,
    CORE_PIN5_BITMASK,	(int)&CORE_PIN5_PINREG,
    CORE_PIN6_BITMASK,	(int)&CORE_PIN6_PINREG,
    CORE_PIN7_BITMASK,	(int)&CORE_PIN7_PINREG,
    CORE_PIN8_BITMASK,	(int)&CORE_PIN8_PINREG,
    CORE_PIN9_BITMASK,	(int)&CORE_PIN9_PINREG,
    CORE_PIN10_BITMASK,	(int)&CORE_PIN10_PINREG,
    CORE_PIN11_BITMASK,	(int)&CORE_PIN11_PINREG,
    CORE_PIN12_BITMASK,	(int)&CORE_PIN12_PINREG,
    CORE_PIN13_BITMASK,	(int)&CORE_PIN13_PINREG,
    CORE_PIN14_BITMASK,	(int)&CORE_PIN14_PINREG,
    CORE_PIN15_BITMASK,	(int)&CORE_PIN15_PINREG,
    CORE_PIN16_BITMASK,	(int)&CORE_PIN16_PINREG,
    CORE_PIN17_BITMASK,	(int)&CORE_PIN17_PINREG,
    CORE_PIN18_BITMASK,	(int)&CORE_PIN18_PINREG,
    CORE_PIN19_BITMASK,	(int)&CORE_PIN19_PINREG,
    CORE_PIN20_BITMASK,	(int)&CORE_PIN20_PINREG,
#if CORE_NUM_TOTAL_PINS > 21
    CORE_PIN21_BITMASK,	(int)&CORE_PIN21_PINREG,
    CORE_PIN22_BITMASK,	(int)&CORE_PIN22_PINREG,
    CORE_PIN23_BITMASK,	(int)&CORE_PIN23_PINREG,
    CORE_PIN24_BITMASK,	(int)&CORE_PIN24_PINREG,
#endif
#if CORE_NUM_TOTAL_PINS > 25
    CORE_PIN25_BITMASK,	(int)&CORE_PIN25_PINREG,
    CORE_PIN26_BITMASK,	(int)&CORE_PIN26_PINREG,
    CORE_PIN27_BITMASK,	(int)&CORE_PIN27_PINREG,
    CORE_PIN28_BITMASK,	(int)&CORE_PIN28_PINREG,
    CORE_PIN29_BITMASK,	(int)&CORE_PIN29_PINREG,
    CORE_PIN30_BITMASK,	(int)&CORE_PIN30_PINREG,
    CORE_PIN31_BITMASK,	(int)&CORE_PIN31_PINREG,
    CORE_PIN32_BITMASK,	(int)&CORE_PIN32_PINREG,
    CORE_PIN33_BITMASK,	(int)&CORE_PIN33_PINREG,
    CORE_PIN34_BITMASK,	(int)&CORE_PIN34_PINREG,
    CORE_PIN35_BITMASK,	(int)&CORE_PIN35_PINREG,
    CORE_PIN36_BITMASK,	(int)&CORE_PIN36_PINREG,
    CORE_PIN37_BITMASK,	(int)&CORE_PIN37_PINREG,
    CORE_PIN38_BITMASK,	(int)&CORE_PIN38_PINREG,
    CORE_PIN39_BITMASK,	(int)&CORE_PIN39_PINREG,
    CORE_PIN40_BITMASK,	(int)&CORE_PIN40_PINREG,
    CORE_PIN41_BITMASK,	(int)&CORE_PIN41_PINREG,
    CORE_PIN42_BITMASK,	(int)&CORE_PIN42_PINREG,
    CORE_PIN43_BITMASK,	(int)&CORE_PIN43_PINREG,
    CORE_PIN44_BITMASK,	(int)&CORE_PIN44_PINREG,
    CORE_PIN45_BITMASK,	(int)&CORE_PIN45_PINREG
#endif
};

static void disable_peripherals(void) __attribute__((noinline));
static void disable_peripherals(void)
{
    EIMSK = 0; PCICR = 0; SPCR = 0; ACSR = 0; EECR = 0; ADCSRA = 0;
    TIMSK0 = 0; TIMSK1 = 0; TIMSK2 = 0; TIMSK3 = 0; UCSR1B = 0; TWCR = 0;
    DDRA = 0; DDRB = 0; DDRC = 0; DDRD = 0; DDRE = 0; DDRF = 0;
    PORTA = 0; PORTB = 0; PORTC = 0; PORTD = 0; PORTE = 0; PORTF = 0;
}


void _reboot_Teensyduino_(void)
{
    cli();
    delayMicroseconds(5000);
    UDCON = 1;
    USBCON = (1<<FRZCLK);
    delayMicroseconds(15000);
    disable_peripherals();
    asm volatile("jmp 0");
    //__builtin_unreachable();  // available in gcc 4.5
    while (1) ;
}

void _restart_Teensyduino_(void)
{
    cli();
    disable_peripherals();  // but leave USB intact
    delayMicroseconds(15000);
    asm volatile("jmp 0");
    //__builtin_unreachable();  // available in gcc 4.5
    while (1) ;
}



#if F_CPU == 16000000L
#define TIMER0_MILLIS_INC  	1
#define TIMER0_FRACT_INC	3
#define TIMER0_MICROS_INC  	4
#elif F_CPU == 8000000L
#define TIMER0_MILLIS_INC  	2
#define TIMER0_FRACT_INC	6
#define TIMER0_MICROS_INC  	8
#elif F_CPU == 4000000L
#define TIMER0_MILLIS_INC  	4
#define TIMER0_FRACT_INC	12
#define TIMER0_MICROS_INC  	16
#elif F_CPU == 2000000L
#define TIMER0_MILLIS_INC  	8
#define TIMER0_FRACT_INC	24
#define TIMER0_MICROS_INC  	32
#elif F_CPU == 1000000L
#define TIMER0_MILLIS_INC  	16
#define TIMER0_FRACT_INC	48
#define TIMER0_MICROS_INC  	64
#endif

volatile unsigned long timer0_micros_count = 0;
volatile unsigned long timer0_millis_count = 0;
volatile unsigned char timer0_fract_count = 0;

void TIMER0_OVF_vect() __attribute__((naked));
void TIMER0_OVF_vect()
{
    asm volatile(
                 "push	r24"				"\n\t"
                 "in	r24, __SREG__"			"\n\t"
                 "push	r24"				"\n\t"

                 "lds	r24, timer0_fract_count"	"\n\t"
                 "subi	r24, 256 - %0"			"\n\t"
                 "cpi	r24, 125"			"\n\t"
                 "brsh	L_%=_fract_roll"		"\n\t"

                 "L_%=_fract_noroll:"				"\n\t"
                 "sts	timer0_fract_count, r24"	"\n\t"
                 "lds	r24, timer0_millis_count"	"\n\t"
                 "subi	r24, 256 - %1"			"\n\t"
                 "sts	timer0_millis_count, r24"	"\n\t"
                 "brcs	L_%=_ovcount"			"\n\t"

                 "L_%=_millis_inc_sext:"
                 "lds	r24, timer0_millis_count+1"	"\n\t"
                 "sbci	r24, 255"			"\n\t"
                 "sts	timer0_millis_count+1, r24"	"\n\t"
                 "brcs	L_%=_ovcount"			"\n\t"
                 "lds	r24, timer0_millis_count+2"	"\n\t"
                 "sbci	r24, 255"			"\n\t"
                 "sts	timer0_millis_count+2, r24"	"\n\t"
                 "brcs	L_%=_ovcount"			"\n\t"
                 "lds	r24, timer0_millis_count+3"	"\n\t"
                 "sbci	r24, 255"			"\n\t"
                 "sts	timer0_millis_count+3, r24"	"\n\t"
                 "rjmp	L_%=_ovcount"			"\n\t"

                 "L_%=_fract_roll:"				"\n\t"
                 "subi	r24, 125"			"\n\t"
                 "sts	timer0_fract_count, r24"	"\n\t"
                 "lds	r24, timer0_millis_count"	"\n\t"
                 "subi	r24, 256 - %1 - 1"		"\n\t"
                 "sts	timer0_millis_count, r24"	"\n\t"
                 "brcc	L_%=_millis_inc_sext"		"\n\t"

                 "L_%=_ovcount:"
                 "lds	r24, timer0_micros_count"	"\n\t"
                 "subi	r24, 256 - %2"			"\n\t"
                 "sts	timer0_micros_count, r24"	"\n\t"
                 "brcs	L_%=_end"			"\n\t"
                 "lds	r24, timer0_micros_count+1"	"\n\t"
                 "sbci	r24, 255"			"\n\t"
                 "sts	timer0_micros_count+1, r24"	"\n\t"
                 "brcs	L_%=_end"			"\n\t"
                 "lds	r24, timer0_micros_count+2"	"\n\t"
                 "sbci	r24, 255"			"\n\t"
                 "sts	timer0_micros_count+2, r24"	"\n\t"

                 "L_%=_end:"
                 "pop	r24"				"\n\t"
                 "out	__SREG__, r24"			"\n\t"
                 "pop	r24"				"\n\t"
                 "reti"
                 : 
                 : "M" (TIMER0_FRACT_INC), "M" (TIMER0_MILLIS_INC),
                   "M" (TIMER0_MICROS_INC)
                 );
}


void delay(uint32_t ms)
{
    for (uint32_t i=0; i<ms; i++) delayMicroseconds(1000);
    /*
    uint16_t start = (uint16_t)micros();

    while (ms > 0) {
        if (((uint16_t)micros() - start) >= 1000) {
            ms--;
            start += 1000;
        }
    }
#if 0
    // This doesn't save a lot of power on Teensy, which
    // lacks the power saving flash memory of some newer
    // chips, and also usually consumes lots of power for
    // the USB port.  There is also some strange (probably
    // hardware) bug involving the A/D mux for the first
    // conversion after the processor wakes from idle mode.
    uint32_t start;
    if (!(SREG & 0x80)) {
        // if interrupts are disabled, busy loop
        while (ms--) delayMicroseconds(1000);
        return;
    }
    // if interrupt are enabled, use low power idle mode
    cli();
    start = timer0_millis_count;
    do {
        _SLEEP_CONTROL_REG = SLEEP_MODE_IDLE | _SLEEP_ENABLE_MASK;
        sei();
        sleep_cpu();
        _SLEEP_CONTROL_REG = SLEEP_MODE_IDLE;
        cli();
    } while (timer0_millis_count - start <= ms);
    sei();
#endif
    */
}


uint32_t _micros(void)
{
    register uint32_t out asm("r22");
    asm volatile(
                 "in	__tmp_reg__, __SREG__"		"\n\t"
                 "cli"					"\n\t"
                 "in	%A0, %2"			"\n\t"
                 "in	__zero_reg__, %3"		"\n\t"
                 "lds	%B0, timer0_micros_count"	"\n\t"
                 "lds	%C0, timer0_micros_count+1"	"\n\t"
                 "lds	%D0, timer0_micros_count+2"	"\n\t"
                 "out	__SREG__, __tmp_reg__"		"\n\t"
                 "sbrs	__zero_reg__, %4"		"\n\t"
                 "rjmp	L_%=_skip"			"\n\t"
                 "cpi	%A0, 255"			"\n\t"
                 "breq	L_%=_skip"			"\n\t"
                 "subi	%B0, 256 - %1"			"\n\t"
                 "sbci	%C0, 255"			"\n\t"
                 "sbci	%D0, 255"			"\n\t"
                 "L_%=_skip:"
                 "clr	__zero_reg__"			"\n\t"
                 "clr	__tmp_reg__"			"\n\t"
#if F_CPU == 16000000L || F_CPU == 8000000L || F_CPU == 4000000L
                 "lsl	%A0"				"\n\t"
                 "rol	__tmp_reg__"			"\n\t"
                 "lsl	%A0"				"\n\t"
                 "rol	__tmp_reg__"			"\n\t"
#if F_CPU == 8000000L || F_CPU == 4000000L
                 "lsl	%A0"				"\n\t"
                 "rol	__tmp_reg__"			"\n\t"
#endif
#if F_CPU == 4000000L
                 "lsl	%A0"				"\n\t"
                 "rol	__tmp_reg__"			"\n\t"
#endif
                 "or	%B0, __tmp_reg__"		"\n\t"
#endif
#if F_CPU == 1000000L || F_CPU == 2000000L
                 "lsr	%A0"				"\n\t"
                 "ror	__tmp_reg__"			"\n\t"
                 "lsr	%A0"				"\n\t"
                 "ror	__tmp_reg__"			"\n\t"
#if F_CPU == 2000000L
                 "lsr	%A0"				"\n\t"
                 "ror	__tmp_reg__"			"\n\t"
#endif
                 "or	%B0, %A0"			"\n\t"
                 "mov	%A0, __tmp_reg__"		"\n\t"
#endif
                 : "=d" (out)
                 : "M" (TIMER0_MICROS_INC),
                   "I" (_SFR_IO_ADDR(TCNT0)),
                   "I" (_SFR_IO_ADDR(TIFR0)),
                   "I" (TOV0)
                 : "r0"
                 );
    return out;
}




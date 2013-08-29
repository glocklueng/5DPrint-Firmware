/* USB Serial for Teensy USB Development Board
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2008 PJRC.COM, LLC
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


#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "pins_teensy.h"
#include "config.h"
#include "pgmspace.h"
#include "usb.h"



/**************************************************************************
 *
 *  Configurable Options
 *
 **************************************************************************/

// You can change these to give your code its own name.  On Windows,
// these are only used before an INF file (driver install) is loaded.
#define STR_MANUFACTURER        L"Makible"
#define STR_PRODUCT             L"Makibox A6"

// All USB serial devices are supposed to have a serial number
// (according to Microsoft).  On windows, a new COM port is created
// for every unique serial/vendor/product number combination.  If
// you program 2 identical boards with 2 different serial numbers
// and they are assigned COM7 and COM8, each will always get the
// same COM port number because Windows remembers serial numbers.
//
// On Mac OS-X, a device file is created automatically which
// incorperates the serial number, eg, /dev/cu-usbmodem12341
//
// Linux by default ignores the serial number, and creates device
// files named /dev/ttyACM0, /dev/ttyACM1... in the order connected.
// Udev rules (in /etc/udev/rules.d) can define persistent device
// names linked to this serial number, as well as permissions, owner
// and group settings.
#define STR_SERIAL_NUMBER       L"001"

// Mac OS-X and Linux automatically load the correct drivers.  On
// Windows, even though the driver is supplied by Microsoft, an
// INF file is needed to load the driver.  These numbers need to
// match the INF file.
//#define VENDOR_ID               0x16C0 //0x1D50
//#define PRODUCT_ID              0x0483 //0x604C
#define VENDOR_ID               0x1D50
#define PRODUCT_ID              0x604C

// When you write data, it goes into a USB endpoint buffer, which
// is transmitted to the PC when it becomes full, or after a timeout
// with no more writes.  Even if you write in exactly packet-size
// increments, this timeout is used to send a "zero length packet"
// that tells the PC no more data is expected and it should pass
// any buffered data to the application that may be waiting.  If
// you want data sent immediately, call usb_serial_flush_output().
#define TRANSMIT_FLUSH_TIMEOUT  3   /* in milliseconds */

// If the PC is connected but not "listening", this is the length
// of time before usb_serial_getchar() returns with an error.  This
// is roughly equivilant to a real UART simply transmitting the
// bits on a wire where nobody is listening, except you get an error
// code which you can ignore for serial-like discard of data, or
// use to know your data wasn't sent.
#define TRANSMIT_TIMEOUT        15   /* in milliseconds */


/**************************************************************************
 *
 *  Endpoint Buffer Configuration
 *
 **************************************************************************/

// These buffer sizes are best for most applications, but perhaps if you
// want more buffering on some endpoint at the expense of others, this
// is where you can make such changes.  The AT90USB162 has only 176 bytes
// of DPRAM (USB buffers) and only endpoints 3 & 4 can double buffer.

#define ENDPOINT0_SIZE          32
#define CDC_ACM_ENDPOINT        2
#define CDC_ACM_SIZE            8
#define CDC_ACM_BUFFER          EP_SINGLE_BUFFER
#define CDC_RX_ENDPOINT         3
#define CDC_RX_BUFFER           EP_DOUBLE_BUFFER
#define CDC_TX_ENDPOINT         4
#define CDC_TX_BUFFER           EP_DOUBLE_BUFFER

#if defined(__AVR_AT90USB162__)
#define CDC_RX_SIZE             32
#define CDC_TX_SIZE             32
#elif defined(__AVR_ATmega32U4__) || defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB1286__)
#define CDC_RX_SIZE             64
#define CDC_TX_SIZE             64
#endif


#define MAX_ENDPOINT            6

#define LSB(n) (n & 255)
#define MSB(n) ((n >> 8) & 255)


// constants corresponding to the various serial parameters
#define USB_SERIAL_DTR                  0x01
#define USB_SERIAL_RTS                  0x02
#define USB_SERIAL_1_STOP               0
#define USB_SERIAL_1_5_STOP             1
#define USB_SERIAL_2_STOP               2
#define USB_SERIAL_PARITY_NONE          0
#define USB_SERIAL_PARITY_ODD           1
#define USB_SERIAL_PARITY_EVEN          2
#define USB_SERIAL_PARITY_MARK          3
#define USB_SERIAL_PARITY_SPACE         4
#define USB_SERIAL_DCD                  0x01
#define USB_SERIAL_DSR                  0x02
#define USB_SERIAL_BREAK                0x04
#define USB_SERIAL_RI                   0x08
#define USB_SERIAL_FRAME_ERR            0x10
#define USB_SERIAL_PARITY_ERR           0x20
#define USB_SERIAL_OVERRUN_ERR          0x40

#define EP_TYPE_CONTROL                 0x00
#define EP_TYPE_BULK_IN                 0x81
#define EP_TYPE_BULK_OUT                0x80
#define EP_TYPE_INTERRUPT_IN            0xC1
#define EP_TYPE_INTERRUPT_OUT           0xC0
#define EP_TYPE_ISOCHRONOUS_IN          0x41
#define EP_TYPE_ISOCHRONOUS_OUT         0x40
#define EP_SINGLE_BUFFER                0x02
#define EP_DOUBLE_BUFFER                0x06
#define EP_SIZE(s)      ((s) == 64 ? 0x30 :     \
                        ((s) == 32 ? 0x20 :     \
                        ((s) == 16 ? 0x10 :     \
                                     0x00)))

#if defined(__AVR_AT90USB162__)
#define HW_CONFIG() 
#define PLL_CONFIG() (PLLCSR = ((1<<PLLE)|(1<<PLLP0)))
#define USB_CONFIG() (USBCON = (1<<USBE))
#define USB_FREEZE() (USBCON = ((1<<USBE)|(1<<FRZCLK)))
#elif defined(__AVR_ATmega32U4__)
#define HW_CONFIG() (UHWCON = 0x01)
#define PLL_CONFIG() (PLLCSR = 0x12)
#define USB_CONFIG() (USBCON = ((1<<USBE)|(1<<OTGPADE)))
#define USB_FREEZE() (USBCON = ((1<<USBE)|(1<<FRZCLK)))
#elif defined(__AVR_AT90USB646__)
#define HW_CONFIG() (UHWCON = 0x81)
#define PLL_CONFIG() (PLLCSR = 0x1A)
#define USB_CONFIG() (USBCON = ((1<<USBE)|(1<<OTGPADE)))
#define USB_FREEZE() (USBCON = ((1<<USBE)|(1<<FRZCLK)))
#elif defined(__AVR_AT90USB1286__)
#define HW_CONFIG() (UHWCON = 0x81)
#define PLL_CONFIG() (PLLCSR = 0x16)
#define USB_CONFIG() (USBCON = ((1<<USBE)|(1<<OTGPADE)))
#define USB_FREEZE() (USBCON = ((1<<USBE)|(1<<FRZCLK)))
#endif

// standard control endpoint request types
#define GET_STATUS                      0
#define CLEAR_FEATURE                   1
#define SET_FEATURE                     3
#define SET_ADDRESS                     5
#define GET_DESCRIPTOR                  6
#define GET_CONFIGURATION               8
#define SET_CONFIGURATION               9
#define GET_INTERFACE                   10
#define SET_INTERFACE                   11
// CDC (communication class device)
#define CDC_SET_LINE_CODING             0x20
#define CDC_GET_LINE_CODING             0x21
#define CDC_SET_CONTROL_LINE_STATE      0x22
#define CDC_SEND_BREAK                  0x23


#define pgm_read_byte_postinc(val, addr) \
        asm ("lpm  %0, Z+\n" : "=r" (val), "+z" (addr) : )
#define pgm_read_word_postinc(val, addr) \
        asm ("lpm  %A0, Z+\n\tlpm  %B0, Z+\n" : "=r" (val), "+z" (addr) : )

#define read_word_lsbfirst(val, reg) \
        asm volatile( \
                "lds  %A0, %1\n\tlds  %B0, %1\n" \
                : "=r" (val) : "M" ((int)(&reg)) )
#define read_word_msbfirst(val, reg) \
        asm volatile( \
                "lds  %B0, %1\n\tlds  %A0, %1\n" \
                : "=r" (val) : "M" ((int)(&reg)) )
#define read_dword_lsbfirst(val, reg) \
        asm volatile( \
                "lds  %A0, %1\n\tlds  %B0, %1\n\t" \
                "lds  %C0, %1\n\tlds  %D0, %1\n" \
                : "=r" (val) : "M" ((int)(&reg)) )
#define read_dword_msbfirst(val, reg) \
        asm volatile( \
                "lds  %D0, %1\n\tlds  %C0, %1\n\t" \
                "lds  %B0, %1\n\tlds  %A0, %1\n" \
                : "=r" (val) : "M" ((int)(&reg)) )

#define write_word_lsbfirst(val, reg) \
        asm volatile( \
                "sts  %1, %A0\n\tsts  %1, %B0\n" \
                : : "r" (val) , "M" ((int)(&reg)) )
#define write_word_msbfirst(val, reg) \
        asm volatile( \
                "sts  %1, %B0\n\tsts  %1, %A0\n" \
                : : "r" (val) , "M" ((int)(&reg)) )
#define write_dword_lsbfirst(val, reg) \
        asm volatile( \
                "sts  %1, %A0\n\tsts  %1, %B0\n\t" \
                "sts  %1, %C0\n\tsts  %1, %D0\n" \
                : : "r" (val) , "M" ((int)(&reg)) )
#define write_dword_msbfirst(val, reg) \
        asm volatile( \
                "sts  %1, %D0\n\tsts  %1, %C0\n\t" \
                "sts  %1, %B0\n\tsts  %1, %A0\n" \
                : : "r" (val) , "M" ((int)(&reg)) )

#define USBSTATE __attribute__ ((section (".noinit")))


/**************************************************************************
 *
 *  Endpoint Buffer Configuration
 *
 **************************************************************************/


static const uint8_t endpoint_config_table[] PROGMEM = {
	0,
	1, EP_TYPE_INTERRUPT_IN,  EP_SIZE(CDC_ACM_SIZE) | CDC_ACM_BUFFER,
	1, EP_TYPE_BULK_OUT,      EP_SIZE(CDC_RX_SIZE) | CDC_RX_BUFFER,
	1, EP_TYPE_BULK_IN,       EP_SIZE(CDC_TX_SIZE) | CDC_TX_BUFFER
};


/**************************************************************************
 *
 *  Descriptor Data
 *
 **************************************************************************/

// Descriptors are the data that your computer reads when it auto-detects
// this USB device (called "enumeration" in USB lingo).  The most commonly
// changed items are editable at the top of this file.  Changing things
// in here should only be done by those who've read chapter 9 of the USB
// spec and relevant portions of any USB class specifications!

static const uint8_t device_descriptor[] PROGMEM = {
	18,					// bLength
	1,					// bDescriptorType
	0x00, 0x02,				// bcdUSB
	2,					// bDeviceClass
	0,					// bDeviceSubClass
	0,					// bDeviceProtocol
	ENDPOINT0_SIZE,				// bMaxPacketSize0
	LSB(VENDOR_ID), MSB(VENDOR_ID),		// idVendor
	LSB(PRODUCT_ID), MSB(PRODUCT_ID),	// idProduct
	0x00, 0x01,				// bcdDevice
	1,					// iManufacturer
	2,					// iProduct
	3,					// iSerialNumber
	1					// bNumConfigurations
};

#define CONFIG1_DESC_SIZE (9+9+5+5+4+5+7+9+7+7)
static const uint8_t config1_descriptor[CONFIG1_DESC_SIZE] PROGMEM = {
	// configuration descriptor, USB spec 9.6.3, page 264-266, Table 9-10
	9, 					// bLength;
	2,					// bDescriptorType;
	LSB(CONFIG1_DESC_SIZE),			// wTotalLength
	MSB(CONFIG1_DESC_SIZE),
	2,					// bNumInterfaces
	1,					// bConfigurationValue
	0,					// iConfiguration
	0xC0,					// bmAttributes
	50,					// bMaxPower
	// interface descriptor, USB spec 9.6.5, page 267-269, Table 9-12
	9,					// bLength
	4,					// bDescriptorType
	0,					// bInterfaceNumber
	0,					// bAlternateSetting
	1,					// bNumEndpoints
	0x02,					// bInterfaceClass
	0x02,					// bInterfaceSubClass
	0x01,					// bInterfaceProtocol
	0,					// iInterface
	// CDC Header Functional Descriptor, CDC Spec 5.2.3.1, Table 26
	5,					// bFunctionLength
	0x24,					// bDescriptorType
	0x00,					// bDescriptorSubtype
	0x10, 0x01,				// bcdCDC
	// Call Management Functional Descriptor, CDC Spec 5.2.3.2, Table 27
	5,					// bFunctionLength
	0x24,					// bDescriptorType
	0x01,					// bDescriptorSubtype
	0x00,					// bmCapabilities
	1,					// bDataInterface
	// Abstract Control Management Functional Descriptor, CDC Spec 5.2.3.3, Table 28
	4,					// bFunctionLength
	0x24,					// bDescriptorType
	0x02,					// bDescriptorSubtype
	0x06,					// bmCapabilities
	// Union Functional Descriptor, CDC Spec 5.2.3.8, Table 33
	5,					// bFunctionLength
	0x24,					// bDescriptorType
	0x06,					// bDescriptorSubtype
	0,					// bMasterInterface
	1,					// bSlaveInterface0
	// endpoint descriptor, USB spec 9.6.6, page 269-271, Table 9-13
	7,					// bLength
	5,					// bDescriptorType
	CDC_ACM_ENDPOINT | 0x80,		// bEndpointAddress
	0x03,					// bmAttributes (0x03=intr)
	CDC_ACM_SIZE, 0,			// wMaxPacketSize
	64,					// bInterval
	// interface descriptor, USB spec 9.6.5, page 267-269, Table 9-12
	9,					// bLength
	4,					// bDescriptorType
	1,					// bInterfaceNumber
	0,					// bAlternateSetting
	2,					// bNumEndpoints
	0x0A,					// bInterfaceClass
	0x00,					// bInterfaceSubClass
	0x00,					// bInterfaceProtocol
	0,					// iInterface
	// endpoint descriptor, USB spec 9.6.6, page 269-271, Table 9-13
	7,					// bLength
	5,					// bDescriptorType
	CDC_RX_ENDPOINT,			// bEndpointAddress
	0x02,					// bmAttributes (0x02=bulk)
	CDC_RX_SIZE, 0,				// wMaxPacketSize
	0,					// bInterval
	// endpoint descriptor, USB spec 9.6.6, page 269-271, Table 9-13
	7,					// bLength
	5,					// bDescriptorType
	CDC_TX_ENDPOINT | 0x80,			// bEndpointAddress
	0x02,					// bmAttributes (0x02=bulk)
	CDC_TX_SIZE, 0,				// wMaxPacketSize
	0					// bInterval
};

// If you're desperate for a little extra code memory, these strings
// can be completely removed if iManufacturer, iProduct, iSerialNumber
// in the device desciptor are changed to zeros.
struct usb_string_descriptor_struct {
	uint8_t bLength;
	uint8_t bDescriptorType;
	int16_t wString[];
};
static const struct usb_string_descriptor_struct string0 PROGMEM = {
	4,
	3,
	{0x0409}
};
static const struct usb_string_descriptor_struct string1 PROGMEM = {
	sizeof(STR_MANUFACTURER),
	3,
	STR_MANUFACTURER
};
static const struct usb_string_descriptor_struct string2 PROGMEM = {
	sizeof(STR_PRODUCT),
	3,
	STR_PRODUCT
};
static const struct usb_string_descriptor_struct string3 PROGMEM = {
	sizeof(STR_SERIAL_NUMBER),
	3,
	STR_SERIAL_NUMBER
};

// This table defines which descriptor data is sent for each specific
// request from the host (in wValue and wIndex).
struct descriptor_list_struct {
	uint16_t	wValue;
	uint16_t	wIndex;
	const uint8_t	*addr;
	uint8_t		length;
};
static const struct descriptor_list_struct descriptor_list[] PROGMEM = {
	{0x0100, 0x0000, device_descriptor, sizeof(device_descriptor)},
	{0x0200, 0x0000, config1_descriptor, sizeof(config1_descriptor)},
	{0x0300, 0x0000, (const uint8_t *)&string0, 4},
	{0x0301, 0x0409, (const uint8_t *)&string1, sizeof(STR_MANUFACTURER)},
	{0x0302, 0x0409, (const uint8_t *)&string2, sizeof(STR_PRODUCT)},
	{0x0303, 0x0409, (const uint8_t *)&string3, sizeof(STR_SERIAL_NUMBER)}
};
#define NUM_DESC_LIST (sizeof(descriptor_list)/sizeof(struct descriptor_list_struct))


/**************************************************************************
 *
 *  Variables - these are the only non-stack RAM usage
 *
 **************************************************************************/

// zero when we are not configured, non-zero when enumerated
volatile uint8_t usb_configuration USBSTATE;
volatile uint8_t usb_suspended USBSTATE;

// the time remaining before we transmit any partially full
// packet, or send a zero length packet.
volatile uint8_t transmit_flush_timer=0;
volatile uint8_t reboot_timer=0;
uint8_t transmit_previous_timeout=0;

// serial port settings (baud rate, control signals, etc) set
// by the PC.  These are ignored, but kept in RAM because the
// CDC spec requires a read that returns the current settings.
volatile uint8_t cdc_line_coding[7]={0x00, 0xE1, 0x00, 0x00, 0x00, 0x00, 0x08};
volatile uint8_t cdc_line_rtsdtr=0;

// "peek buffer" for usb_serial
int16_t peek_buf;


/**************************************************************************
 *
 *  Public Functions - these are the API intended for the user
 *
 **************************************************************************/


// initialize USB serial
void usb_init(void)
{
	uint8_t u;

	u = USBCON;
	if ((u & (1<<USBE)) && !(u & (1<<FRZCLK))) return;
	HW_CONFIG();
        USB_FREEZE();				// enable USB
        PLL_CONFIG();				// config PLL
        while (!(PLLCSR & (1<<PLOCK))) ;	// wait for PLL lock
        USB_CONFIG();				// start USB clock
        UDCON = 0;				// enable attach resistor
	usb_configuration = 0;
	usb_suspended = 0;
	cdc_line_rtsdtr = 0;
	UDINT = 0;
        UDIEN = (1<<EORSTE)|(1<<SOFE)|(1<<SUSPE);
}


void usb_shutdown(void)
{
	UDIEN = 0;	// disable interrupts
	UDCON = 1;	// disconnect attach resistor
	USBCON = 0;	// shut off USB periperal
	PLLCSR = 0;	// shut off PLL
	usb_configuration = 0;
	usb_suspended = 1;
}

void usb_serial_begin(void)
{
	// make sure USB is initialized
	peek_buf = -1;
	usb_init();
	uint16_t begin_wait = (uint16_t)millis();
	while (1) {
		// wait for the host to finish enumeration
		if (usb_configuration) {
			delay(200);  // a little time for host to load a driver
			return;
		}
		// or for suspend mode (powered without USB)
		if (usb_suspended) {
			uint16_t begin_suspend = (uint16_t)millis();
			while (usb_suspended) {
				// must remain suspended for a while, because
				// normal USB enumeration causes brief suspend
				// states, typically under 0.1 second
				if ((uint16_t)millis() - begin_suspend > 250) {
					return;
				}
			}
		}
		// ... or a timout (powered by a USB power adaptor that
		// wiggles the data lines to keep a USB device charging)
		if ((uint16_t)millis() - begin_wait > 2500) return;
	}
}

void usb_serial_end(void)
{
	usb_shutdown();
	delay(25);
}

// number of bytes available in the receive buffer
int usb_serial_available(void)
{
	uint8_t n=0, i, intr_state;

	intr_state = SREG;
	cli();
	
	if (usb_configuration) {
		UENUM = CDC_RX_ENDPOINT;
		n = UEBCLX;
			
		if (!n) {
			i = UEINTX;
			if (i & (1<<RXOUTI) && !(i & (1<<RWAL))) UEINTX = 0x6B;
		}
	}
	
	SREG = intr_state;
		
	if (peek_buf >= 0 && n < 255) {
		n++;
	}

    return n;
}

int usb_serial_peek(void)
{
	if (peek_buf < 0) peek_buf = usb_serial_read();
	return peek_buf;
}

// get the next character, or -1 if nothing received
int usb_serial_read(void)
{
    uint8_t c, intr_state;

	if (peek_buf >= 0) {
		c = peek_buf;
		peek_buf = -1;
		return c;
	}
        // interrupts are disabled so these functions can be
        // used from the main program or interrupt context,
        // even both in the same program!
        intr_state = SREG;
        cli();
        if (!usb_configuration) {
                SREG = intr_state;
                return -1;
        }
        UENUM = CDC_RX_ENDPOINT;
	retry:
	c = UEINTX;
        if (!(c & (1<<RWAL))) {
                // no data in buffer
		if (c & (1<<RXOUTI)) {
			UEINTX = 0x6B;
			goto retry;
		}
                SREG = intr_state;
                return -1;
        }
        // take one byte out of the buffer
        c = UEDATX;
        // if this drained the buffer, release it
        if (!(UEINTX & (1<<RWAL))) UEINTX = 0x6B;
        SREG = intr_state;
        return c;
}

// discard any buffered input
void usb_serial_discard(void)
{
        uint8_t intr_state;

        if (usb_configuration) {
                intr_state = SREG;
                cli();
                UENUM = CDC_RX_ENDPOINT;
                while ((UEINTX & (1<<RWAL))) {
                        UEINTX = 0x6B;
                }
                SREG = intr_state;
        }
	peek_buf = -1;
}

#define setWriteError()
size_t usb_serial_write(const uint8_t *buffer, uint16_t size)
{
	uint8_t timeout, intr_state, write_size;
	size_t count=0;

	intr_state = SREG;
	// if we're not online (enumerated and configured), error
	if (!usb_configuration) {
		setWriteError();
		goto end;
	}
	// interrupts are disabled so these functions can be
	// used from the main program or interrupt context,
	// even both in the same program!
	//intr_state = SREG;
	cli();
	UENUM = CDC_TX_ENDPOINT;
	// if we gave up due to timeout before, don't wait again
	if (transmit_previous_timeout) {
		if (!(UEINTX & (1<<RWAL))) {
			SREG = intr_state;
			setWriteError();
			goto end;
		}
		transmit_previous_timeout = 0;
	}
	// each iteration of this loop transmits a packet
	while (size) {
		// wait for the FIFO to be ready to accept data
		timeout = UDFNUML + TRANSMIT_TIMEOUT;
		while (1) {
			// are we ready to transmit?
			if (UEINTX & (1<<RWAL)) break;
			SREG = intr_state;
			// have we waited too long?  This happens if the user
			// is not running an application that is listening
			if (UDFNUML == timeout) {
				transmit_previous_timeout = 1;
				setWriteError();
				goto end;
			}
			// has the USB gone offline?
			if (!usb_configuration) {
				setWriteError();
				goto end;
			}
			// get ready to try checking again
			intr_state = SREG;
			cli();
			UENUM = CDC_TX_ENDPOINT;
		}

		// compute how many bytes will fit into the next packet
		write_size = CDC_TX_SIZE - UEBCLX;
		if (write_size > size) write_size = size;
		size -= write_size;
		count += write_size;

#define ASM_COPY1(src, dest, tmp) "ld " tmp ", " src "\n\t" "st " dest ", " tmp "\n\t"
#define ASM_COPY2(src, dest, tmp) ASM_COPY1(src, dest, tmp) ASM_COPY1(src, dest, tmp)
#define ASM_COPY4(src, dest, tmp) ASM_COPY2(src, dest, tmp) ASM_COPY2(src, dest, tmp)
#define ASM_COPY8(src, dest, tmp) ASM_COPY4(src, dest, tmp) ASM_COPY4(src, dest, tmp)

#if 1
		// write the packet
		do {
			uint8_t tmp;
			asm volatile(
			"L%=begin:"					"\n\t"
				"ldi	r30, %4"			"\n\t"
				"sub	r30, %3"			"\n\t"
				"cpi	r30, %4"			"\n\t"
				"brsh	L%=err"				"\n\t"
				"lsl	r30"				"\n\t"
				"clr	r31"				"\n\t"
				"subi	r30, lo8(-(pm(L%=table)))"	"\n\t"
				"sbci	r31, hi8(-(pm(L%=table)))"	"\n\t"
				"ijmp"					"\n\t"
			"L%=err:"					"\n\t"
				"rjmp	L%=end"				"\n\t"
			"L%=table:"					"\n\t"
				#if (CDC_TX_SIZE == 64)
				ASM_COPY8("Y+", "X", "%1")
				ASM_COPY8("Y+", "X", "%1")
				ASM_COPY8("Y+", "X", "%1")
				ASM_COPY8("Y+", "X", "%1")
				#endif
				#if (CDC_TX_SIZE >= 32)
				ASM_COPY8("Y+", "X", "%1")
				ASM_COPY8("Y+", "X", "%1")
				#endif
				#if (CDC_TX_SIZE >= 16)
				ASM_COPY8("Y+", "X", "%1")
				#endif
				ASM_COPY8("Y+", "X", "%1")
			"L%=end:"					"\n\t"
				: "+y" (buffer), "=r" (tmp)
				: "x" (&UEDATX), "r" (write_size), "M" (CDC_TX_SIZE)
				: "r30", "r31"
			);
		} while (0);
#endif
		// if this completed a packet, transmit it now!
		if (!(UEINTX & (1<<RWAL))) UEINTX = 0x3A;
		transmit_flush_timer = TRANSMIT_FLUSH_TIMEOUT;
	}
	//SREG = intr_state;
end:
	SREG = intr_state;
	return count;
}

void usb_serial_printf(const char *fmt, ...)
{
    char buf[128];
    int size;
    va_list args;

    va_start(args, fmt);
    size = vsnprintf(buf, 128, fmt, args);
    va_end(args);
    if (size < 128)
    {
        // TODO:  error handling when exceeding the 128-byte buffer
        usb_serial_write((uint8_t *)buf, size);
    }
}


void usb_serial_printf_P(PGM_P fmt, ...)
{
    char buf[128];
    int size;
    va_list args;

    va_start(args, fmt);
    size = vsnprintf_P(buf, 128, fmt, args);
    va_end(args);
    if (size < 128)
    {
        // TODO:  error handling when exceeding the 128-byte buffer
        usb_serial_write((uint8_t *)buf, size);
    }
}

// immediately transmit any buffered output.
// This doesn't actually transmit the data - that is impossible!
// USB devices only transmit when the host allows, so the best
// we can do is release the FIFO buffer for when the host wants it
void usb_serial_flush(void)
{
        uint8_t intr_state;

        intr_state = SREG;
        cli();
        if (usb_configuration && transmit_flush_timer) {
                UENUM = CDC_TX_ENDPOINT;
                UEINTX = 0x3A;
                transmit_flush_timer = 0;
        }
        SREG = intr_state;
}

uint32_t usb_serial_baud(void)
{
    return ((uint32_t)cdc_line_coding[0] << 0) |
           ((uint32_t)cdc_line_coding[1] << 8) |
           ((uint32_t)cdc_line_coding[2] << 16) |
           ((uint32_t)cdc_line_coding[3] << 24);
}

uint8_t usb_serial_stopbits(void)
{
	return cdc_line_coding[4];
}

uint8_t usb_serial_paritytype(void)
{
	return cdc_line_coding[5];
}

uint8_t usb_serial_numbits(void)
{
	return cdc_line_coding[6];
}

uint8_t usb_serial_dtr(void)
{
	return (cdc_line_rtsdtr & USB_SERIAL_DTR) ? 1 : 0;
}

uint8_t usb_serial_rts(void)
{
	return (cdc_line_rtsdtr & USB_SERIAL_RTS) ? 1 : 0;
}


/**************************************************************************
 *
 *  Private Functions - not intended for general user consumption....
 *
 **************************************************************************/


// USB Device Interrupt - handle all device-level events
// the transmit buffer flushing is triggered by the start of frame
//
ISR(USB_GEN_vect)
{
	uint8_t intbits, t;
	
	#if (DEBUG > -1)
		PreemptionFlag |= 0x8000;
	#endif

        intbits = UDINT;
        UDINT = 0;
        if (intbits & (1<<EORSTI)) {
		// USB Reset
		UENUM = 0;
		UECONX = 1;
		UECFG0X = EP_TYPE_CONTROL;
		UECFG1X = EP_SIZE(ENDPOINT0_SIZE) | EP_SINGLE_BUFFER;
		UEIENX = (1<<RXSTPE);
		usb_configuration = 0;
		cdc_line_rtsdtr = 0;
	}
	if (intbits & (1<<SOFI)) {
		// Start Of Frame
		if (usb_configuration) {
			t = transmit_flush_timer;
			if (t) {
				transmit_flush_timer = --t;
				if (!t) {
					UENUM = CDC_TX_ENDPOINT;
					UEINTX = 0x3A;
				}
			}
			t = reboot_timer;
			if (t) {
				reboot_timer = --t;
				if (!t) _reboot_Teensyduino_();
			}
		}
	}
	// in active state
	if (intbits & (1<<SUSPI)) {
		// USB Suspend (inactivity for 3ms)
		UDIEN = (1<<WAKEUPE);
		usb_configuration = 0;
		usb_suspended = 1;
		#if (F_CPU >= 8000000L)
		// WAKEUPI does not work with USB clock freeze 
		// when CPU is running less than 8 MHz.
		// Is this a hardware bug?
		USB_FREEZE();			// shut off USB
		PLLCSR = 0;			// shut off PLL
		#endif
		// to properly meet the USB spec, current must
		// reduce to less than 2.5 mA, which means using
		// powerdown mode, but that breaks the Arduino
		// user's paradigm....
	}
	if (usb_suspended && (intbits & (1<<WAKEUPI))) {
		// USB Resume (pretty much any activity)
		#if (F_CPU >= 8000000L)
		PLL_CONFIG();
		while (!(PLLCSR & (1<<PLOCK))) ;
		USB_CONFIG();
		#endif
		UDIEN = (1<<EORSTE)|(1<<SOFE)|(1<<SUSPE);
		usb_suspended = 0;
		return;
	}
}


// Misc functions to wait for ready and send/receive packets
static inline void usb_wait_in_ready(void)
{
	while (!(UEINTX & (1<<TXINI))) ;
}
static inline void usb_send_in(void)
{
	UEINTX = ~(1<<TXINI);
}
static inline void usb_wait_receive_out(void)
{
	while (!(UEINTX & (1<<RXOUTI))) ;
}
static inline void usb_ack_out(void)
{
	UEINTX = ~(1<<RXOUTI);
}



// USB Endpoint Interrupt - endpoint 0 is handled here.  The
// other endpoints are manipulated by the user-callable
// functions, and the start-of-frame interrupt.
//
ISR(USB_COM_vect)
{
        uint8_t intbits;
	const uint8_t *list;
        const uint8_t *cfg;
	uint8_t i, n, len, en;
	volatile uint8_t *p;
	uint8_t bmRequestType;
	uint8_t bRequest;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
	uint16_t desc_val;
	const uint8_t *desc_addr;
	uint8_t	desc_length;

	#if (DEBUG > -1)
		PreemptionFlag |= 0x4000;
	#endif
	
	UENUM = 0;
	intbits = UEINTX;
	if (intbits & (1<<RXSTPI)) {
		bmRequestType = UEDATX;
		bRequest = UEDATX;
		//wValue = UEDATX;
		//wValue |= (UEDATX << 8);
		read_word_lsbfirst(wValue, UEDATX);
		//wIndex = UEDATX;
		//wIndex |= (UEDATX << 8);
		read_word_lsbfirst(wIndex, UEDATX);
		//wLength = UEDATX;
		//wLength |= (UEDATX << 8);
		read_word_lsbfirst(wLength, UEDATX);
                UEINTX = ~((1<<RXSTPI) | (1<<RXOUTI) | (1<<TXINI));
                if (bRequest == GET_DESCRIPTOR) {
			list = (const uint8_t *)descriptor_list;
			for (i=0; ; i++) {
				if (i >= NUM_DESC_LIST) {
					UECONX = (1<<STALLRQ)|(1<<EPEN);  //stall
					return;
				}
				//desc_val = pgm_read_word(list);
				//list += 2;
				pgm_read_word_postinc(desc_val, list);
				if (desc_val != wValue) {
					list += sizeof(struct descriptor_list_struct)-2;
					continue;
				}
				//desc_val = pgm_read_word(list);
				//list += 2;
				pgm_read_word_postinc(desc_val, list);
				if (desc_val != wIndex) {
					list += sizeof(struct descriptor_list_struct)-4;
					continue;
				}
				//desc_addr = (const uint8_t *)pgm_read_word(list);
				//list += 2;
				pgm_read_word_postinc(desc_addr, list);
				desc_length = pgm_read_byte(list);
				break;
			}
			len = (wLength < 256) ? wLength : 255;
			if (len > desc_length) len = desc_length;
			list = desc_addr;
			do {
				// wait for host ready for IN packet
				do {
					i = UEINTX;
				} while (!(i & ((1<<TXINI)|(1<<RXOUTI))));
				if (i & (1<<RXOUTI)) return;	// abort
				// send IN packet
				n = len < ENDPOINT0_SIZE ? len : ENDPOINT0_SIZE;
				for (i = n; i; i--) {
					//UEDATX = pgm_read_byte(desc_addr++);
					//pgm_read_byte_postinc(UEDATX, desc_addr);
					pgm_read_byte_postinc(UEDATX, list);
				}
				len -= n;
				usb_send_in();
			} while (len || n == ENDPOINT0_SIZE);
			return;
                }
		if (bRequest == SET_ADDRESS) {
			usb_send_in();
			usb_wait_in_ready();
			UDADDR = wValue | (1<<ADDEN);
			return;
		}
		if (bRequest == SET_CONFIGURATION && bmRequestType == 0) {
			usb_configuration = wValue;
			cdc_line_rtsdtr = 0;
			transmit_flush_timer = 0;
			usb_send_in();
			cfg = endpoint_config_table;
			for (i=1; i<5; i++) {
				UENUM = i;
				//en = pgm_read_byte(cfg++);
				pgm_read_byte_postinc(en, cfg);
				UECONX = en;
				if (en) {
					//UECFG0X = pgm_read_byte(cfg++);
					//UECFG1X = pgm_read_byte(cfg++);
					pgm_read_byte_postinc(UECFG0X, cfg);
					pgm_read_byte_postinc(UECFG1X, cfg);
				}
			}
        		UERST = 0x1E;
        		UERST = 0;
			return;
		}
		if (bRequest == GET_CONFIGURATION && bmRequestType == 0x80) {
			usb_wait_in_ready();
			UEDATX = usb_configuration;
			usb_send_in();
			return;
		}
		if (bRequest == CDC_GET_LINE_CODING /* 0x21 */ && bmRequestType == 0xA1) {
			usb_wait_in_ready();
			p = cdc_line_coding;
			for (i=0; i<7; i++) {
				UEDATX = *p++;
			}
			usb_send_in();
			return;
		}
		if (bRequest == CDC_SET_LINE_CODING /* 0x20 */ && bmRequestType == 0x21) {
			usb_wait_receive_out();
			p = cdc_line_coding;
			for (i=0; i<7; i++) {
				*p++ = UEDATX;
			}
			usb_ack_out();
			usb_send_in();
			return;
		}
		if (bRequest == CDC_SET_CONTROL_LINE_STATE /* 0x22 */ && bmRequestType == 0x21) {
			cdc_line_rtsdtr = wValue;
			usb_wait_in_ready();
			usb_send_in();
			return;
		}
		if (bRequest == CDC_SEND_BREAK /* 0x23 */ && bmRequestType == 0x21) {
			usb_wait_in_ready();
			usb_send_in();
			return;
		}
		if (bRequest == GET_STATUS) {
			usb_wait_in_ready();
			i = 0;
			if (bmRequestType == 0x82) {
				UENUM = wIndex;
				if (UECONX & (1<<STALLRQ)) i = 1;
				UENUM = 0;
			}
			UEDATX = i;
			UEDATX = 0;
			usb_send_in();
			return;
		}
		if ((bRequest == CLEAR_FEATURE || bRequest == SET_FEATURE)
		  && bmRequestType == 0x02 && wValue == 0) {
			i = wIndex & 0x7F;
			if (i >= 1 && i <= MAX_ENDPOINT) {
				usb_send_in();
				UENUM = i;
				if (bRequest == SET_FEATURE) {
					UECONX = (1<<STALLRQ)|(1<<EPEN);
				} else {
					UECONX = (1<<STALLRQC)|(1<<RSTDT)|(1<<EPEN);
					UERST = (1 << i);
					UERST = 0;
				}
				return;
			}
		}
        }
	UECONX = (1<<STALLRQ) | (1<<EPEN);	// stall
}



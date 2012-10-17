#include <stddef.h>
#include <stdint.h>


void usb_init();
void usb_shutdown();
void usb_serial_begin();
void usb_serial_end();
int usb_serial_available();
int usb_serial_peek();
int usb_serial_read();
void usb_serial_discard();
size_t usb_serial_write(const uint8_t *buffer, uint16_t size);
void usb_serial_flush();
uint32_t usb_serial_baud();
uint8_t usb_serial_stopbits();
uint8_t usb_serial_paritytype();
uint8_t usb_serial_numbits();
uint8_t usb_serial_dtr();
uint8_t usb_serial_rts();

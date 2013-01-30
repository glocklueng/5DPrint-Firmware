// A bug in avr-gcc causes spurious warnings when printing a float value:
//   warning: format ‘%f’ expects type ‘double’, but argument 2 has type ‘float’
// (This is because the '%f' format is actually defined to take a double.)
//
// On the AVR architecture, float and double are identical.  This seems to be
// confusing the compiler (see gcc bug #46372).  However, because they are
// identical, we can simply #define away all our worries:

/* makibox.h
*
* History:
* =======
* + 	02 NOV 2012		Author: JTK Wong (XTRONTEC Limited)
*		Moved function prototype for analogWrite_check() to makibox.h
*		so that it can be called from heater.c
*/

#define float double

#define NUM_AXIS 4
#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2
#define E_AXIS 3

extern unsigned char reset_flags;

void manage_inactivity(unsigned char debug);

void enable_x();
void enable_y();
void enable_z();
void enable_e();
void disable_x();
void disable_y();
void disable_z();
void disable_e();

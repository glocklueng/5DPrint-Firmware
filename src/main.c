
#include <avr/interrupt.h>
#include "pins_teensy.h"


void setup();
void loop();


#if F_CPU != 16000000L
#error "Prescaler values are only valid for F_CPU=16MHZ"
#endif
#define ADC_PRESCALER 0x07
#define CPU_PRESCALER 0x00
#define ADC_PRESCALE_ADJUST (-1)
#define DEFAULT_ADCSRB 0x80  // ADHSM  (high speed)


void board_init(void)
{
	cli();
	CLKPR = 0x80;
	CLKPR = CPU_PRESCALER;
	// timer 0, fast pwm mode
	TCCR0A = (1<<WGM01) | (1<<WGM00);
	TCCR0B = (1<<CS01) | (1<<CS00);		// div 64 prescaler
	TIMSK0 |= (1<<TOIE0);
	// timer 1, 8 bit phase correct pwm
	TCCR1A = (1<<WGM10);
	TCCR1B = (1<<CS11);			// div 8 prescaler
	// timer 2, 8 bit phase correct pwm
	TCCR2A = (1<<WGM20);
	TCCR2B = (1<<CS21);			// div 8 prescaler
	// timer 3, 8 bit phase correct pwm
	TCCR3A = (1<<WGM30);
	TCCR3B = (1<<CS31);			// div 8 prescaler
	// ADC
	ADCSRA = (1<<ADEN) | (ADC_PRESCALER + ADC_PRESCALE_ADJUST);
	ADCSRB = DEFAULT_ADCSRB;
	DIDR0 = 0;
	sei();
}


int main(void)
{
	board_init();

	setup();
    
	while (1) {
		loop();
	}
}

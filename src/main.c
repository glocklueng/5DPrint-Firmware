
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include "pins_teensy.h"
#include "config.h"


void setup();
void loop();
void CPU_Util_Calc(void);

#if F_CPU != 16000000L
#error "Prescaler values are only valid for F_CPU=16MHZ"
#endif
#define ADC_PRESCALER 0x07
#define CPU_PRESCALER 0x00
#define ADC_PRESCALE_ADJUST (-1)
#define DEFAULT_ADCSRB 0x80  // ADHSM  (high speed)

#define CPU_UTIL_CHECK_PERIOD	25 // ms

uint32_t bckgnd_loop_count = 0;
uint32_t previous_bckgnd_task_start_time = 0;
uint32_t bckgnd_task_time = 0;
unsigned char cpu_loading = 0, peak_cpu_load = 0, average_cpu_load = 0;
uint32_t previous_millis_cpu_util = 0;
uint16_t PreemptionFlag = 0;
unsigned char reset_flags;

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
	// timer 2 -> not used;
	// timer 3 -> initialised later in setup();
	// ADC
	ADCSRA = (1<<ADEN) | (ADC_PRESCALER + ADC_PRESCALE_ADJUST);
	ADCSRB = DEFAULT_ADCSRB;
	DIDR0 = 0;
	
	// Save the reset flags and then clear MCUSR register
	reset_flags = MCUSR;
	MCUSR = 0;
	
	// Disable the watchdog timer
	wdt_disable();
	
	sei();
}


int main(void)
{
	unsigned char first_loop = 1;
	board_init();

	setup();
    
	while (1) {
		if (DEBUG > -1)
		{
			CPU_Util_Calc();
			
			if (first_loop > 0)
			{
				peak_cpu_load = 0;
				average_cpu_load = 0;
				first_loop = 0;
			}
		}

		loop();
	}
}


/***************************************************
* CPU_Util_Calc(void)
*
* Calculates a rough estimate of CPU loading.
*
* Keeps a running average of how long it takes to 
* run the background / idle task without interruption
* by ISR's or other heavy work loads. The PreemptionFlag
* is set when running ISR's or heavy work loads to
* indicate this.
*
* This functions estimates the amount of time spent
* running the background / idle loop over a period of
* CPU_UTIL_CHECK_PERIOD. The CPU loading is then taken
* to be [100% - (percentage idle time)].
****************************************************/
void CPU_Util_Calc(void)
{
	unsigned char interrupted = 1;
	uint32_t bckgnd_task_start_time;
	uint32_t total_cpu_time, idle_time, work_time;
	
	bckgnd_loop_count++;
	
	cli();	// Disbale interrupts
	
	bckgnd_task_start_time = micros();
	
	if (PreemptionFlag == 0)
	{
		interrupted = 0;
	}
	PreemptionFlag = 0;
	
	sei();	//Enable interrupts
	
	// Update running average of bckgnd_task_time only if it has not been
	// interrupted.
	if ( !interrupted )
	{
		bckgnd_task_time += bckgnd_task_start_time - previous_bckgnd_task_start_time;
		bckgnd_task_time = (uint32_t)( (float)(bckgnd_task_time) / 2.0 );
	}
	
	// calc every CPU_UTIL_CHECK_PERIOD
	if( (millis() - previous_millis_cpu_util) >= CPU_UTIL_CHECK_PERIOD )
	{
		idle_time = (bckgnd_loop_count) * bckgnd_task_time;
		
		if ( idle_time > (CPU_UTIL_CHECK_PERIOD * 1000) )
		{
			idle_time = CPU_UTIL_CHECK_PERIOD * 1000;
		}
		work_time = (CPU_UTIL_CHECK_PERIOD * 1000) - idle_time;	// us
		total_cpu_time = CPU_UTIL_CHECK_PERIOD * 1000; 	// us
		cpu_loading = (unsigned char)( ((float)(work_time) / (float)(total_cpu_time)) * 100 ); // %
		
		if (cpu_loading > peak_cpu_load)
		{
			peak_cpu_load = cpu_loading;
		}
		
		average_cpu_load = (unsigned char)( (average_cpu_load + cpu_loading) / 2.0 );
		
		bckgnd_loop_count = 0;
		previous_millis_cpu_util = millis();
	}
	
	previous_bckgnd_task_start_time = bckgnd_task_start_time;
}

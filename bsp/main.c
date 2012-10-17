#include "core_pins.h"


void setup();
void loop();


int main(void)
{
	_init_Teensyduino_internal_();

	setup();
    
	while (1) {
		loop();
	}
}

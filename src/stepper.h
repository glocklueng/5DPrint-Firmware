#ifndef STEPPER_H
#define STEPPER_H


extern uint32_t timer1_compa_isr_exe_micros;
extern uint32_t timer1_compa_isr_exe_micros_min;
extern uint32_t timer1_compa_isr_exe_micros_max;


extern unsigned short virtual_steps_x;
extern unsigned short virtual_steps_y;
extern unsigned short virtual_steps_z;

extern uint8_t is_homing;
extern uint8_t x_homed, y_homed, z_homed;

typedef struct {
	unsigned long x;
	unsigned long y;
	unsigned long z;
	long e;
} st_position_t;

void st_init();
void st_wake_up();
void st_sleep();
void st_synchronize();
void st_set_current_position(st_position_t new_position);
st_position_t st_get_current_position(void);

#endif
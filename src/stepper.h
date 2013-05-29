#ifndef STEPPER_H
#define STEPPER_H

#define ENABLE_STEPPER_DRIVER_INTERRUPT()  TIMSK1 |= (1<<OCIE1A)
#define DISABLE_STEPPER_DRIVER_INTERRUPT() TIMSK1 &= ~(1<<OCIE1A)

#if (DEBUG > -1)
	extern uint32_t timer1_compa_isr_exe_micros;
	extern uint32_t timer1_compa_isr_exe_micros_min;
	extern uint32_t timer1_compa_isr_exe_micros_max;
#endif

extern unsigned short virtual_steps_x;
extern unsigned short virtual_steps_y;
extern unsigned short virtual_steps_z;

extern uint8_t is_homing;
extern uint8_t x_homed, y_homed, z_homed;

extern uint8_t pause_print_req;
extern uint8_t print_paused;

typedef struct {
	long x;
	long y;
	long z;
	long e;
} st_position_t;

void st_init();
void st_wake_up();
void st_sleep();
void st_synchronize();
void st_set_current_position(st_position_t new_position);
st_position_t st_get_current_position(void);
void get_current_printer_state(void);
void set_print_paused_buffer(void);
void clear_plan_buffer(void);
void resume_normal_print_buffer(void);

#endif
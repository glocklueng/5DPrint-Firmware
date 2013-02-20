#ifndef STEPPER_H
#define STEPPER_H

extern uint32_t timer1_compa_isr_exe_micros;
extern uint32_t timer1_compa_isr_exe_micros_min;
extern uint32_t timer1_compa_isr_exe_micros_max;

static unsigned short virtual_steps_x = 0;
static unsigned short virtual_steps_y = 0;
static unsigned short virtual_steps_z = 0;

void st_init();
void st_wake_up();
void st_synchronize();
void st_set_position(const long *x, const long *y, const long *z, const long *e);

#endif
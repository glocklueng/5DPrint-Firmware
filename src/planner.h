

struct block;
typedef struct block block_t;


void plan_init();
void st_init();
void tp_init();
void plan_buffer_line(float x, float y, float z, float e, float feed_rate);
void plan_set_position(float x, float y, float z, float e);
void st_wake_up();
void st_synchronize();
void st_set_position(const long *x, const long *y, const long *z, const long *e);


/*
extern block_t block_buffer[];
extern volatile unsigned char block_buffer_head;
extern volatile unsigned char block_buffer_tail;
*/

extern float axis_steps_per_unit[4];
extern float max_feedrate[4];
extern float homing_feedrate[3];
extern uint8_t axis_relative_modes[4];
extern float move_acceleration;
extern float retract_acceleration;
extern float max_xy_jerk;
extern float max_z_jerk;
extern float max_e_jerk;
extern volatile int extrudemultiply;
extern float mintravelfeedrate;
extern float minimumfeedrate;
extern uint8_t is_homing;
extern unsigned long axis_steps_per_sqr_second[NUM_AXIS];

void enable_endstops(uint8_t check);

uint8_t blocks_queued();
uint8_t blocks_available();

void check_axes_activity();

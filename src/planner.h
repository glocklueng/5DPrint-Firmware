#ifndef PLANNER_H
#define PLANNER_H

#include "config.h"

// This struct is used when buffering the setup for each linear movement
// "nominal" values are as specified in the source g-code and may never
// actually be reached if acceleration management is active.
struct block {
  // Fields used by the bresenham algorithm for tracing the line
  long steps_x;
  long steps_y;
  long steps_z;
  long steps_e;  // Step count along each axis

  unsigned long step_event_count;  // The number of step events required to complete this block
  long accelerate_until;           // The index of the step event on which to stop acceleration
  long decelerate_after;           // The index of the step event on which to start decelerating
  long acceleration_rate;          // The acceleration rate used for acceleration calculation
  unsigned char direction_bits;    // The direction bit set for this block (refers to *_DIRECTION_BIT in config.h)

  // Fields used by the motion planner to manage acceleration
//  float speed_x, speed_y, speed_z, speed_e;        // Nominal mm/minute for each axis
  float nominal_speed;                               // The nominal speed for this block in mm/min  
  float entry_speed;                                 // Entry speed at previous-current junction in mm/min
  float max_entry_speed;                             // Maximum allowable junction entry speed in mm/min
  float millimeters;                                 // The total travel of this block in mm
  float acceleration;                                // acceleration mm/sec^2
  unsigned char recalculate_flag;                    // Planner flag to recalculate trapezoids on entry junction
  unsigned char nominal_length_flag;                 // Planner flag for nominal speed always reached


  // Settings for the trapezoid generator
  long nominal_rate;                        // The nominal step rate for this block in step_events/sec 
  long initial_rate;                        // The jerk-adjusted step rate at start of block  
  long final_rate;                          // The minimal rate at exit
  long acceleration_st;                     // acceleration steps/sec^2
  volatile char busy;
};

typedef struct block block_t;


typedef struct {
	float paused_pos_x;
	float paused_pos_y;
	float paused_pos_z;
	float paused_pos_e;
	int hotend_target_temp;
	int hotend_target_temp_raw;
	int target_bed_temp_raw;
	unsigned char block_buffer_head;
	unsigned char block_buffer_tail;
	float current_position_x;
	float current_position_y;
	float current_position_z;
	float current_position_e;
} paused_data_t;

// This is used adjust the circular plan buffer when the print has been paused
extern unsigned char volatile block_buffer_size;
extern unsigned char volatile block_buffer_mask;

void plan_init();
void plan_buffer_line(float x, float y, float z, float e, float feed_rate);
void plan_set_position(float x, float y, float z, float e);
block_t *plan_get_current_block();
void plan_discard_current_block();
void check_axes_activity();


extern block_t block_buffer[BLOCK_BUFFER_SIZE];
extern block_t resume_buffer[PRINT_PAUSED_BLOCK_BUF_SIZE];
extern volatile unsigned char block_buffer_head;
extern volatile unsigned char block_buffer_tail;


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
extern unsigned long axis_steps_per_sqr_second[NUM_AXIS];

extern paused_data_t paused_data;

void enable_endstops(uint8_t check);

uint8_t blocks_queued();
uint8_t blocks_available();

#endif
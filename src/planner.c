/*
 Makibox A6 firmware, based on Sprinter (master branch, 1 Sep 2012).
 Designed for Printrboard (Rev B).
 
 ---

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 

* History:
* =======
*
* +		28 NOV 2012		Author: JTK Wong (XTRONTEC Limited)
*		ISR(TIMER1_COMPA_vect) nop instruction added to ensure that pulse width 
*		generated meets the	minimum requirement for the Allegro A4982 stepper 
*		motor driver input.
*		The existing code relies on delays in executing commands between setting
*		the step high and stepping it low again to meet the A4982's minimum pulse 
*		width requirement. Some motor drive issues have been seen during 
*		development testing which may be related to this.
*		
*		*** NOTE ***
*		THIS IS A QUICK AND DIRTY HACK TO GET THINGS WORKING A
*		BIT BETTER. RECOMMEND THAT THIS ISR IS RE-VISITED AND REFACTORED TO
*		TO WORK PROPERLY AND GENERATE THE CORRECT PULSE WIDTHS FOR THE A4982
*		DRIVER IC.
*
* +		29 NOV 2012		Author: JTK Wong 	XTRONTEC Limited
*											www.xtrontec.com
*		Added some very rough indicators of ISR execution time for 
*		ISR(TIMER1_COMPA_vect).
*
* +		30 NOV 2012		Author: JTK Wong 	XTRONTEC Limited
*											www.xtrontec.com
*		ISR(TIMER1_COMPA_vect) added another nop loop after the stepper pulse
*		has been pulled low. This is to ensure that the minimum low pulse width
*		is met for the A4982.
*
* +		17 Dec 2012		Author: JTK Wong 	XTRONTEC Limited
*											www.xtrontec.com
*		Added casting and literals for division calculations in order to guard
*		against integer division problems.
*/


#include <avr/interrupt.h>
#include "pgmspace.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <util/crc16.h>

#include "config.h"
#include "makibox.h"
#include "board_io.h"
#include "pins.h"
#include "pins_teensy.h"
#include "speed_lookuptable.h"
#include "usb.h"
#include "planner.h"

#ifdef USE_ARC_FUNCTION
  #include "arc_func.h"
#endif

#ifndef CRITICAL_SECTION_START
#define CRITICAL_SECTION_START  unsigned char _sreg = SREG; cli()
#define CRITICAL_SECTION_END    SREG = _sreg
#endif //CRITICAL_SECTION_START

#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))


static unsigned short virtual_steps_x = 0;
static unsigned short virtual_steps_y = 0;
static unsigned short virtual_steps_z = 0;
float axis_steps_per_unit[4] = _AXIS_STEP_PER_UNIT;

float max_feedrate[4] = _MAX_FEEDRATE;
float homing_feedrate[3] = _HOMING_FEEDRATE;
uint8_t axis_relative_modes[4] = _AXIS_RELATIVE_MODES;
float move_acceleration = _ACCELERATION;         // Normal acceleration mm/s^2
float retract_acceleration = _RETRACT_ACCELERATION; // Normal acceleration mm/s^2
float max_xy_jerk = _MAX_XY_JERK;
float max_z_jerk = _MAX_Z_JERK;
float max_e_jerk = _MAX_E_JERK;
volatile int extrudemultiply = 100;
float mintravelfeedrate = DEFAULT_MINTRAVELFEEDRATE;
float minimumfeedrate = DEFAULT_MINIMUMFEEDRATE;
unsigned long axis_steps_per_sqr_second[NUM_AXIS];

uint32_t timer1_compa_isr_exe_micros = 0;
uint32_t timer1_compa_isr_exe_micros_min = 0xFFFFFFFF;
uint32_t timer1_compa_isr_exe_micros_max = 0;

uint8_t is_homing = 0;

// Planner with Interrupt for Stepper

// This struct is used when buffering the setup for each linear movement
// "nominal" values are as specified in the source g-code and may never
// actually be reached if acceleration management is active.
struct block {
  // Fields used by the bresenham algorithm for tracing the line
  long steps_x, steps_y, steps_z, steps_e;  // Step count along each axis

  unsigned long step_event_count;                    // The number of step events required to complete this block
  long accelerate_until;           // The index of the step event on which to stop acceleration
  long decelerate_after;           // The index of the step event on which to start decelerating
  long acceleration_rate;          // The acceleration rate used for acceleration calculation
  unsigned char direction_bits;             // The direction bit set for this block (refers to *_DIRECTION_BIT in config.h)

  // Fields used by the motion planner to manage acceleration
//  float speed_x, speed_y, speed_z, speed_e;          // Nominal mm/minute for each axis
  float nominal_speed;                               // The nominal speed for this block in mm/min  
  float entry_speed;                                 // Entry speed at previous-current junction in mm/min
  float max_entry_speed;                             // Maximum allowable junction entry speed in mm/min
  float millimeters;                                 // The total travel of this block in mm
  float acceleration;                                // acceleration mm/sec^2
  unsigned char recalculate_flag;                    // Planner flag to recalculate trapezoids on entry junction
  unsigned char nominal_length_flag;                 // Planner flag for nominal speed always reached


  // Settings for the trapezoid generator
  long nominal_rate;                                 // The nominal step rate for this block in step_events/sec 
  long initial_rate;                        // The jerk-adjusted step rate at start of block  
  long final_rate;                          // The minimal rate at exit
  long acceleration_st;                              // acceleration steps/sec^2
  volatile char busy;
};

/*  
 Reasoning behind the mathematics in this module (in the key of 'Mathematica'):
 
 s == speed, a == acceleration, t == time, d == distance
 
 Basic definitions:
 
 Speed[s_, a_, t_] := s + (a*t) 
 Travel[s_, a_, t_] := Integrate[Speed[s, a, t], t]
 
 Distance to reach a specific speed with a constant acceleration:
 
 Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, d, t]
 d -> (m^2 - s^2)/(2 a) --> estimate_acceleration_distance()
 
 Speed after a given distance of travel with constant acceleration:
 
 Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, m, t]
 m -> Sqrt[2 a d + s^2]    
 
 DestinationSpeed[s_, a_, d_] := Sqrt[2 a d + s^2]
 
 When to start braking (di) to reach a specified destionation speed (s2) after accelerating
 from initial speed s1 without ever stopping at a plateau:
 
 Solve[{DestinationSpeed[s1, a, di] == DestinationSpeed[s2, a, d - di]}, di]
 di -> (2 a d - s1^2 + s2^2)/(4 a) --> intersection_distance()
 
 IntersectionDistance[s1_, s2_, a_, d_] := (2 a d - s1^2 + s2^2)/(4 a)
 */


block_t block_buffer[BLOCK_BUFFER_SIZE];            // A ring buffer for motion instructions
volatile unsigned char block_buffer_head;           // Index of the next block to be pushed
volatile unsigned char block_buffer_tail;           // Index of the block to process now

//===========================================================================
//=============================private variables ============================
//===========================================================================

// Returns the index of the next block in the ring buffer
// NOTE: Removed modulo (%) operator, which uses an expensive divide and multiplication.
static int8_t next_block_index(int8_t block_index) {
  block_index++;
  if (block_index == BLOCK_BUFFER_SIZE) { block_index = 0; }
  return(block_index);
}


// Returns the index of the previous block in the ring buffer
static int8_t prev_block_index(int8_t block_index) {
  if (block_index == 0) { block_index = BLOCK_BUFFER_SIZE; }
  block_index--;
  return(block_index);
}

// The current position of the tool in absolute steps
static long position[4];   
static float previous_speed[4]; // Speed of previous path line segment
static float previous_nominal_speed; // Nominal speed of previous path line segment
static unsigned char G92_reset_previous_speed = 0;


// Calculates the distance (not time) it takes to accelerate from initial_rate to target_rate using the 
// given acceleration:
float estimate_acceleration_distance(float initial_rate, float target_rate, float acceleration)
{
  if (acceleration!=0) {
  return((target_rate*target_rate-initial_rate*initial_rate)/
         (2.0*acceleration));
  }
  else {
    return 0.0;  // acceleration was 0, set acceleration distance to 0
  }
}

// This function gives you the point at which you must start braking (at the rate of -acceleration) if 
// you started at speed initial_rate and accelerated until this point and want to end at the final_rate after
// a total travel of distance. This can be used to compute the intersection point between acceleration and
// deceleration in the cases where the trapezoid has no plateau (i.e. never reaches maximum speed)

float intersection_distance(float initial_rate, float final_rate, float acceleration, float distance) 
{
 if (acceleration!=0) {
  return((2.0*acceleration*distance-initial_rate*initial_rate+final_rate*final_rate)/
         (4.0*acceleration) );
  }
  else {
    return 0.0;  // acceleration was 0, set intersection distance to 0
  }
}

// Calculates trapezoid parameters so that the entry- and exit-speed is compensated by the provided factors.

void calculate_trapezoid_for_block(block_t *block, float entry_factor, float exit_factor) {
  unsigned long initial_rate = ceil(block->nominal_rate*entry_factor); // (step/min)
  unsigned long final_rate = ceil(block->nominal_rate*exit_factor); // (step/min)

  // Limit minimal step rate (Otherwise the timer will overflow.)
  if(initial_rate <120) {initial_rate=120; }
  if(final_rate < 120) {final_rate=120;  }
  
  long acceleration = block->acceleration_st;
  int32_t accelerate_steps =
    ceil(estimate_acceleration_distance(block->initial_rate, block->nominal_rate, acceleration));
  int32_t decelerate_steps =
    floor(estimate_acceleration_distance(block->nominal_rate, block->final_rate, -acceleration));
    
  // Calculate the size of Plateau of Nominal Rate.
  int32_t plateau_steps = block->step_event_count-accelerate_steps-decelerate_steps;
  
  // Is the Plateau of Nominal Rate smaller than nothing? That means no cruising, and we will
  // have to use intersection_distance() to calculate when to abort acceleration and start breaking
  // in order to reach the final_rate exactly at the end of this block.
  if (plateau_steps < 0) {
    accelerate_steps = ceil(
      intersection_distance(block->initial_rate, block->final_rate, acceleration, block->step_event_count));
    accelerate_steps = MAX(accelerate_steps,0); // Check limits due to numerical round-off
    accelerate_steps = MIN(accelerate_steps,block->step_event_count);
    plateau_steps = 0;
  }

 // block->accelerate_until = accelerate_steps;
 // block->decelerate_after = accelerate_steps+plateau_steps;
  CRITICAL_SECTION_START;  // Fill variables used by the stepper in a critical section
  if(!block->busy) { // Don't update variables if block is busy.
    block->accelerate_until = accelerate_steps;
    block->decelerate_after = accelerate_steps+plateau_steps;
    block->initial_rate = initial_rate;
    block->final_rate = final_rate;
  }
  CRITICAL_SECTION_END;
}                    

// Calculates the maximum allowable speed at this point when you must be able to reach target_velocity using the 
// acceleration within the allotted distance.
float max_allowable_speed(float acceleration, float target_velocity, float distance) {
  return  sqrt(target_velocity*target_velocity-2*acceleration*distance);
}

// "Junction jerk" in this context is the immediate change in speed at the junction of two blocks.
// This method will calculate the junction jerk as the euclidean distance between the nominal 
// velocities of the respective blocks.
//inline float junction_jerk(block_t *before, block_t *after) {
//  return sqrt(
//    pow((before->speed_x-after->speed_x), 2)+pow((before->speed_y-after->speed_y), 2));
//}



// The kernel called by planner_recalculate() when scanning the plan from last to first entry.
void planner_reverse_pass_kernel(block_t *previous, block_t *current, block_t *next) {
  if(!current) { return; }
  
    if (next) {
    // If entry speed is already at the maximum entry speed, no need to recheck. Block is cruising.
    // If not, block in state of acceleration or deceleration. Reset entry speed to maximum and
    // check for maximum allowable speed reductions to ensure maximum possible planned speed.
    if (current->entry_speed != current->max_entry_speed) {
    
      // If nominal length true, max junction speed is guaranteed to be reached. Only compute
      // for max allowable speed if block is decelerating and nominal length is false.
      if ((!current->nominal_length_flag) && (current->max_entry_speed > next->entry_speed)) {
        current->entry_speed = MIN( current->max_entry_speed,
          max_allowable_speed(-current->acceleration,next->entry_speed,current->millimeters));
      } else {
        current->entry_speed = current->max_entry_speed;
      }
      current->recalculate_flag = 1;
    
    }
  } // Skip last block. Already initialized and set for recalculation.
}

// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This 
// implements the reverse pass.
void planner_reverse_pass() {
  uint8_t block_index = block_buffer_head;
  
  //Make a local copy of block_buffer_tail, because the interrupt can alter it
  CRITICAL_SECTION_START;
  unsigned char tail = block_buffer_tail;
  CRITICAL_SECTION_END;
  
  if(((block_buffer_head-tail + BLOCK_BUFFER_SIZE) & BLOCK_BUFFER_MASK) > 3) 
  {
    block_index = (block_buffer_head - 3) & BLOCK_BUFFER_MASK; 
    block_t *block[3] = { NULL, NULL, NULL };
    while(block_index != tail) { 
      block_index = prev_block_index(block_index); 
      block[2]= block[1];
      block[1]= block[0];
      block[0] = &block_buffer[block_index];
      planner_reverse_pass_kernel(block[0], block[1], block[2]);
    }
  }
}


// The kernel called by planner_recalculate() when scanning the plan from first to last entry.
void planner_forward_pass_kernel(block_t *previous, block_t *current, block_t *next) {
  if(!previous) { return; }
  
  // If the previous block is an acceleration block, but it is not long enough to complete the
  // full speed change within the block, we need to adjust the entry speed accordingly. Entry
  // speeds have already been reset, maximized, and reverse planned by reverse planner.
  // If nominal length is true, max junction speed is guaranteed to be reached. No need to recheck.
  if (!previous->nominal_length_flag) {
    if (previous->entry_speed < current->entry_speed) {
      double entry_speed = MIN( current->entry_speed,
        max_allowable_speed(-previous->acceleration,previous->entry_speed,previous->millimeters) );

      // Check for junction speed change
      if (current->entry_speed != entry_speed) {
        current->entry_speed = entry_speed;
        current->recalculate_flag = 1;
      }
    }
  }
}

// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This 
// implements the forward pass.
void planner_forward_pass() {
  uint8_t block_index = block_buffer_tail;
  block_t *block[3] = { NULL, NULL, NULL };

  while(block_index != block_buffer_head) {
    block[0] = block[1];
    block[1] = block[2];
    block[2] = &block_buffer[block_index];
    planner_forward_pass_kernel(block[0],block[1],block[2]);
    block_index = next_block_index(block_index);
  }
  planner_forward_pass_kernel(block[1], block[2], NULL);
}

// Recalculates the trapezoid speed profiles for all blocks in the plan according to the 
// entry_factor for each junction. Must be called by planner_recalculate() after 
// updating the blocks.
void planner_recalculate_trapezoids() {
  int8_t block_index = block_buffer_tail;
  block_t *current;
  block_t *next = NULL;
  
  while(block_index != block_buffer_head) {
    current = next;
    next = &block_buffer[block_index];
    if (current) {
      // Recalculate if current block entry or exit junction speed has changed.
      if (current->recalculate_flag || next->recalculate_flag) {
        // NOTE: Entry and exit factors always > 0 by all previous logic operations.
        calculate_trapezoid_for_block(current, current->entry_speed/(float)(current->nominal_speed),
          next->entry_speed/(float)(current->nominal_speed));
        current->recalculate_flag = 0; // Reset current only to ensure next trapezoid is computed
      }
    }
    block_index = next_block_index( block_index );
  }
  // Last/newest block in buffer. Exit speed is set with MINIMUM_PLANNER_SPEED. Always recalculated.
  if(next != NULL) {
    calculate_trapezoid_for_block(next, next->entry_speed/(float)(next->nominal_speed),
      MINIMUM_PLANNER_SPEED/(float)(next->nominal_speed));
    next->recalculate_flag = 0;
  }
}

// Recalculates the motion plan according to the following algorithm:
//
//   1. Go over every block in reverse order and calculate a junction speed reduction (i.e. block_t.entry_factor) 
//      so that:
//     a. The junction jerk is within the set limit
//     b. No speed reduction within one block requires faster deceleration than the one, true constant 
//        acceleration.
//   2. Go over every block in chronological order and dial down junction speed reduction values if 
//     a. The speed increase within one block would require faster accelleration than the one, true 
//        constant acceleration.
//
// When these stages are complete all blocks have an entry_factor that will allow all speed changes to 
// be performed using only the one, true constant acceleration, and where no junction jerk is jerkier than 
// the set limit. Finally it will:
//
//   3. Recalculate trapezoids for all blocks.

void planner_recalculate() {   
  planner_reverse_pass();
  planner_forward_pass();
  planner_recalculate_trapezoids();
}

void plan_init() {
  block_buffer_head = 0;
  block_buffer_tail = 0;
  memset(position, 0, sizeof(position)); // clear position
  previous_speed[0] = 0.0;
  previous_speed[1] = 0.0;
  previous_speed[2] = 0.0;
  previous_speed[3] = 0.0;
  previous_nominal_speed = 0.0;
}



void plan_discard_current_block() {
  if (block_buffer_head != block_buffer_tail) {
    block_buffer_tail = (block_buffer_tail + 1) & BLOCK_BUFFER_MASK;  
  }
}

block_t *plan_get_current_block() {
  if (block_buffer_head == block_buffer_tail) { 
    return(NULL); 
  }
  block_t *block = &block_buffer[block_buffer_tail];
  block->busy = 1;
  return(block);
}

uint8_t blocks_queued() 
{
  uint8_t head;	    // where we insert new blocks
  uint8_t tail;	    // where we remove old blocks

  CRITICAL_SECTION_START;
  head = block_buffer_head;
  tail = block_buffer_tail;
  CRITICAL_SECTION_END;

  if (head > tail) {
    return head - tail;
  } else if (head < tail) {
    return head + (BLOCK_BUFFER_SIZE - tail);
  } else {
    return 0;
  }
}

uint8_t blocks_available()
{
  return BLOCK_BUFFER_SIZE - blocks_queued();
}

float junction_deviation = 0.1;
float max_E_feedrate_calc = MAX_RETRACT_FEEDRATE;
uint8_t retract_feedrate_aktiv = 0;

// Add a new linear movement to the buffer. steps_x, _y and _z is the absolute position in 
// mm. Microseconds specify how many microseconds the move should take to perform. To aid acceleration
// calculation the caller must also provide the physical length of the line in millimeters.
void plan_buffer_line(float x, float y, float z, float e, float feed_rate)
{
  // Calculate the buffer head after we push this byte
  int next_buffer_head = next_block_index(block_buffer_head);

  // If the buffer is full: good! That means we are well ahead of the robot. 
  // Rest here until there is room in the buffer.
  while(block_buffer_tail == next_buffer_head) { 
    manage_inactivity(1); 
    #if (MINIMUM_FAN_START_SPEED > 0)
      manage_fan_start_speed();
    #endif 
  }

  // The target position of the tool in absolute steps
  // Calculate target position in absolute steps
  //this should be done after the wait, because otherwise a M92 code within the gcode disrupts this calculation somehow
  long target[4];
  target[X_AXIS] = lround(x*axis_steps_per_unit[X_AXIS]);
  target[Y_AXIS] = lround(y*axis_steps_per_unit[Y_AXIS]);
  target[Z_AXIS] = lround(z*axis_steps_per_unit[Z_AXIS]);     
  target[E_AXIS] = lround(e*axis_steps_per_unit[E_AXIS]);
  
  // Prepare to set up new block
  block_t *block = &block_buffer[block_buffer_head];
  
  // Mark block as not busy (Not executed by the stepper interrupt)
  block->busy = 0;

  // Number of steps for each axis
  block->steps_x = labs(target[X_AXIS]-position[X_AXIS]);
  block->steps_y = labs(target[Y_AXIS]-position[Y_AXIS]);
  block->steps_z = labs(target[Z_AXIS]-position[Z_AXIS]);
  block->steps_e = labs(target[E_AXIS]-position[E_AXIS]);
  block->steps_e *= extrudemultiply;
  block->steps_e /= 100.0;
  block->step_event_count = MAX(block->steps_x, MAX(block->steps_y, MAX(block->steps_z, block->steps_e)));

  // Bail if this is a zero-length block
  if (block->step_event_count <= DROP_SEGMENTS) { return; };

  // Compute direction bits for this block 
  block->direction_bits = 0;
  if (target[X_AXIS] < position[X_AXIS]) { block->direction_bits |= (1<<X_AXIS); }
  if (target[Y_AXIS] < position[Y_AXIS]) { block->direction_bits |= (1<<Y_AXIS); }
  if (target[Z_AXIS] < position[Z_AXIS]) { block->direction_bits |= (1<<Z_AXIS); }
  if (target[E_AXIS] < position[E_AXIS]) 
  { 
    block->direction_bits |= (1<<E_AXIS); 
    //High Feedrate for retract
    max_E_feedrate_calc = MAX_RETRACT_FEEDRATE;
    retract_feedrate_aktiv = 1;
  }
  else
  {
     if(retract_feedrate_aktiv)
     {
       if(block->steps_e > 0)
         retract_feedrate_aktiv = 0;
     }
     else
     {
       max_E_feedrate_calc = max_feedrate[E_AXIS]; 
     }
  }
  

 #ifdef DELAY_ENABLE
  if(block->steps_x != 0)
  {
    enable_x();
    delayMicroseconds(DELAY_ENABLE);
  }
  if(block->steps_y != 0)
  {
    enable_y();
    delayMicroseconds(DELAY_ENABLE);
  }
  if(block->steps_z != 0)
  {
    enable_z();
    delayMicroseconds(DELAY_ENABLE);
  }
  if(block->steps_e != 0)
  {
    enable_e();
    delayMicroseconds(DELAY_ENABLE);
  }
 #else
  //enable active axes
  if(block->steps_x != 0) enable_x();
  if(block->steps_y != 0) enable_y();
  if(block->steps_z != 0) enable_z();
  if(block->steps_e != 0) enable_e();
 #endif 
 
  if (block->steps_e == 0) {
        if(feed_rate<mintravelfeedrate) feed_rate=mintravelfeedrate;
  }
  else {
    	if(feed_rate<minimumfeedrate) feed_rate=minimumfeedrate;
  } 

  // slow down when the buffer starts to empty, rather than wait at the corner for a buffer refill
  int moves_queued=(block_buffer_head-block_buffer_tail + BLOCK_BUFFER_SIZE) & BLOCK_BUFFER_MASK;
#ifdef SLOWDOWN  
  if(moves_queued < (BLOCK_BUFFER_SIZE * 0.5) && moves_queued > 1) feed_rate = feed_rate*moves_queued / (float)(BLOCK_BUFFER_SIZE * 0.5); 
#endif

  float delta_mm[4];
  delta_mm[X_AXIS] = (target[X_AXIS]-position[X_AXIS])/(float)(axis_steps_per_unit[X_AXIS]);
  delta_mm[Y_AXIS] = (target[Y_AXIS]-position[Y_AXIS])/(float)(axis_steps_per_unit[Y_AXIS]);
  delta_mm[Z_AXIS] = (target[Z_AXIS]-position[Z_AXIS])/(float)(axis_steps_per_unit[Z_AXIS]);
  //delta_mm[E_AXIS] = (target[E_AXIS]-position[E_AXIS])/axis_steps_per_unit[E_AXIS];
  delta_mm[E_AXIS] = ((target[E_AXIS]-position[E_AXIS])/(float)(axis_steps_per_unit[E_AXIS]))*extrudemultiply/100.0;
  
  if ( block->steps_x <= DROP_SEGMENTS && block->steps_y <= DROP_SEGMENTS && block->steps_z <= DROP_SEGMENTS ) {
    block->millimeters = fabs(delta_mm[E_AXIS]);
  } else {
    block->millimeters = sqrt(square(delta_mm[X_AXIS]) + square(delta_mm[Y_AXIS]) + square(delta_mm[Z_AXIS]));
  }
  
  float inverse_millimeters = 1.0/(float)(block->millimeters);  // Inverse millimeters to remove multiple divides 
  
  // Calculate speed in mm/second for each axis. No divide by zero due to previous checks.
  float inverse_second = feed_rate * inverse_millimeters;
  
  block->nominal_speed = block->millimeters * inverse_second; // (mm/sec) Always > 0
  block->nominal_rate = ceil(block->step_event_count * inverse_second); // (step/sec) Always > 0

  
 

  
/*
  //  segment time im micro seconds
  long segment_time = lround(1000000.0/inverse_second);
  if ((blockcount>0) && (blockcount < (BLOCK_BUFFER_SIZE - 4))) {
    if (segment_time<minsegmenttime)  { // buffer is draining, add extra time.  The amount of time added increases if the buffer is still emptied more.
        segment_time=segment_time+lround(2*(minsegmenttime-segment_time)/blockcount);
    }
  }
  else {
    if (segment_time<minsegmenttime) segment_time=minsegmenttime;
  }
  //  END OF SLOW DOWN SECTION    
*/


 // Calculate and limit speed in mm/sec for each axis
  float current_speed[4];
  float speed_factor = 1.0; //factor <=1 do decrease speed
  for(int i=0; i < 3; i++) 
  {
    current_speed[i] = delta_mm[i] * inverse_second;
    if(fabs(current_speed[i]) > max_feedrate[i])
      speed_factor = fmin(speed_factor, max_feedrate[i] / fabs(current_speed[i]));
  }
  
  current_speed[E_AXIS] = delta_mm[E_AXIS] * inverse_second;
  if(fabs(current_speed[E_AXIS]) > max_E_feedrate_calc)
    speed_factor = fmin(speed_factor, max_E_feedrate_calc / fabs(current_speed[E_AXIS]));


  // Correct the speed  
  if( speed_factor < 1.0) 
  {
    for(unsigned char i=0; i < 4; i++) {
      current_speed[i] *= speed_factor;
    }
    block->nominal_speed *= speed_factor;
    block->nominal_rate *= speed_factor;
  }

  // Compute and limit the acceleration rate for the trapezoid generator.  
  float steps_per_mm = block->step_event_count/(float)(block->millimeters);
  if(block->steps_x == 0 && block->steps_y == 0 && block->steps_z == 0) {
    block->acceleration_st = ceil(retract_acceleration * steps_per_mm); // convert to: acceleration steps/sec^2
  }
  else {
    block->acceleration_st = ceil(move_acceleration * steps_per_mm); // convert to: acceleration steps/sec^2
    // Limit acceleration per axis
    if(((float)block->acceleration_st * (float)block->steps_x / (float)block->step_event_count) > axis_steps_per_sqr_second[X_AXIS])
      block->acceleration_st = axis_steps_per_sqr_second[X_AXIS];
    if(((float)block->acceleration_st * (float)block->steps_y / (float)block->step_event_count) > axis_steps_per_sqr_second[Y_AXIS])
      block->acceleration_st = axis_steps_per_sqr_second[Y_AXIS];
    if(((float)block->acceleration_st * (float)block->steps_e / (float)block->step_event_count) > axis_steps_per_sqr_second[E_AXIS])
      block->acceleration_st = axis_steps_per_sqr_second[E_AXIS];
    if(((float)block->acceleration_st * (float)block->steps_z / (float)block->step_event_count ) > axis_steps_per_sqr_second[Z_AXIS])
      block->acceleration_st = axis_steps_per_sqr_second[Z_AXIS];
  }
  block->acceleration = block->acceleration_st / (float)(steps_per_mm);
  block->acceleration_rate = (long)((float)block->acceleration_st * 8.388608);
  
#if 0  // Use old jerk for now
  // Compute path unit vector
  double unit_vec[3];

  unit_vec[X_AXIS] = delta_mm[X_AXIS]*inverse_millimeters;
  unit_vec[Y_AXIS] = delta_mm[Y_AXIS]*inverse_millimeters;
  unit_vec[Z_AXIS] = delta_mm[Z_AXIS]*inverse_millimeters;
  
  // Compute maximum allowable entry speed at junction by centripetal acceleration approximation.
  // Let a circle be tangent to both previous and current path line segments, where the junction
  // deviation is defined as the distance from the junction to the closest edge of the circle,
  // colinear with the circle center. The circular segment joining the two paths represents the
  // path of centripetal acceleration. Solve for max velocity based on max acceleration about the
  // radius of the circle, defined indirectly by junction deviation. This may be also viewed as
  // path width or max_jerk in the previous grbl version. This approach does not actually deviate
  // from path, but used as a robust way to compute cornering speeds, as it takes into account the
  // nonlinearities of both the junction angle and junction velocity.
  double vmax_junction = MINIMUM_PLANNER_SPEED; // Set default max junction speed

  // Skip first block or when previous_nominal_speed is used as a flag for homing and offset cycles.
  if ((block_buffer_head != block_buffer_tail) && (previous_nominal_speed > 0.0)) {
    // Compute cosine of angle between previous and current path. (prev_unit_vec is negative)
    // NOTE: Max junction velocity is computed without sin() or acos() by trig half angle identity.
    double cos_theta = - previous_unit_vec[X_AXIS] * unit_vec[X_AXIS]
                       - previous_unit_vec[Y_AXIS] * unit_vec[Y_AXIS]
                       - previous_unit_vec[Z_AXIS] * unit_vec[Z_AXIS] ;
                           
    // Skip and use default max junction speed for 0 degree acute junction.
    if (cos_theta < 0.95) {
      vmax_junction = min(previous_nominal_speed,block->nominal_speed);
      // Skip and avoid divide by zero for straight junctions at 180 degrees. Limit to min() of nominal speeds.
      if (cos_theta > -0.95) {
        // Compute maximum junction velocity based on maximum acceleration and junction deviation
        double sin_theta_d2 = sqrt(0.5*(1.0-cos_theta)); // Trig half angle identity. Always positive.
        vmax_junction = min(vmax_junction,
          sqrt(block->acceleration * junction_deviation * sin_theta_d2/(float)(1.0-sin_theta_d2)) );
      }
    }
  }
#endif
  // Start with a safe speed
  float vmax_junction = max_xy_jerk/2.0; 
  float vmax_junction_factor = 1.0; 

  if(fabs(current_speed[Z_AXIS]) > max_z_jerk/2.0) 
    vmax_junction = fmin(vmax_junction, max_z_jerk/2.0);

  if(fabs(current_speed[E_AXIS]) > max_e_jerk/2.0) 
    vmax_junction = fmin(vmax_junction, max_e_jerk/2.0);

  if(G92_reset_previous_speed == 1)
  {
    vmax_junction = 0.1;
    G92_reset_previous_speed = 0;  
  }

  vmax_junction = fmin(vmax_junction, block->nominal_speed);
  float safe_speed = vmax_junction;

  if ((moves_queued > 1) && (previous_nominal_speed > 0.0001)) {
    float jerk = sqrt(pow((current_speed[X_AXIS]-previous_speed[X_AXIS]), 2)+pow((current_speed[Y_AXIS]-previous_speed[Y_AXIS]), 2));
    //    if((fabs(previous_speed[X_AXIS]) > 0.0001) || (fabs(previous_speed[Y_AXIS]) > 0.0001)) {
    vmax_junction = block->nominal_speed;
    //    }
    if (jerk > max_xy_jerk) {
      vmax_junction_factor = (max_xy_jerk/(float)(jerk));
    } 
    if(fabs(current_speed[Z_AXIS] - previous_speed[Z_AXIS]) > max_z_jerk) {
      vmax_junction_factor= fmin(vmax_junction_factor, (max_z_jerk/fabs(current_speed[Z_AXIS] - previous_speed[Z_AXIS])));
    } 
    if(fabs(current_speed[E_AXIS] - previous_speed[E_AXIS]) > max_e_jerk) {
      vmax_junction_factor = fmin(vmax_junction_factor, (max_e_jerk/fabs(current_speed[E_AXIS] - previous_speed[E_AXIS])));
    } 
    vmax_junction = fmin(previous_nominal_speed, vmax_junction * vmax_junction_factor); // Limit speed to max previous speed
  }
  block->max_entry_speed = vmax_junction;

  // Initialize block entry speed. Compute based on deceleration to user-defined MINIMUM_PLANNER_SPEED.
  double v_allowable = max_allowable_speed(-block->acceleration,MINIMUM_PLANNER_SPEED,block->millimeters);
  block->entry_speed = fmin(vmax_junction, v_allowable);

  // Initialize planner efficiency flags
  // Set flag if block will always reach maximum junction speed regardless of entry/exit speeds.
  // If a block can de/ac-celerate from nominal speed to zero within the length of the block, then
  // the current block and next block junction speeds are guaranteed to always be at their maximum
  // junction speeds in deceleration and acceleration, respectively. This is due to how the current
  // block nominal speed limits both the current and next maximum junction speeds. Hence, in both
  // the reverse and forward planners, the corresponding block junction speed will always be at the
  // the maximum junction speed and may always be ignored for any speed reduction checks.
  if (block->nominal_speed <= v_allowable) { 
    block->nominal_length_flag = 1; 
  }
  else { 
    block->nominal_length_flag = 0; 
  }
  block->recalculate_flag = 1; // Always calculate trapezoid for new block

  // Update previous path unit_vector and nominal speed
  memcpy(previous_speed, current_speed, sizeof(previous_speed)); // previous_speed[] = current_speed[]
  previous_nominal_speed = block->nominal_speed;
  calculate_trapezoid_for_block(block, block->entry_speed/(float)(block->nominal_speed),
    safe_speed/(float)(block->nominal_speed));
    
  // Move buffer head
  block_buffer_head = next_buffer_head;
  
  // Update position
  memcpy(position, target, sizeof(target)); // position[] = target[]

  planner_recalculate();
  #ifdef AUTOTEMP
    getHighESpeed();
  #endif
  st_wake_up();
}

int calc_plannerpuffer_fill(void)
{
  int moves_queued=(block_buffer_head-block_buffer_tail + BLOCK_BUFFER_SIZE) & BLOCK_BUFFER_MASK;
  return(moves_queued);
}

void plan_set_position(float x, float y, float z, float e)
{
  position[X_AXIS] = lround(x*axis_steps_per_unit[X_AXIS]);
  position[Y_AXIS] = lround(y*axis_steps_per_unit[Y_AXIS]);
  position[Z_AXIS] = lround(z*axis_steps_per_unit[Z_AXIS]);     
  position[E_AXIS] = lround(e*axis_steps_per_unit[E_AXIS]);  

  virtual_steps_x = 0;
  virtual_steps_y = 0;
  virtual_steps_z = 0;

  previous_nominal_speed = 0.0; // Resets planner junction speeds. Assumes start from rest.
  previous_speed[0] = 0.0;
  previous_speed[1] = 0.0;
  previous_speed[2] = 0.0;
  previous_speed[3] = 0.0;
  
  G92_reset_previous_speed = 1;
}

#ifdef AUTOTEMP
void getHighESpeed()
{
  static float oldt=0;
  if(!autotemp_enabled)
    return;
  if((target_temp+2) < autotemp_min)  //probably temperature set to zero.
    return; //do nothing
  
  float high=0.0;
  uint8_t block_index = block_buffer_tail;
  
  while(block_index != block_buffer_head) {
    if((block_buffer[block_index].steps_x != 0) ||
       (block_buffer[block_index].steps_y != 0) ||
       (block_buffer[block_index].steps_z != 0)) {
      float se=(float(block_buffer[block_index].steps_e)/float(block_buffer[block_index].step_event_count))*block_buffer[block_index].nominal_speed;
      //se; units steps/sec;
      if(se>high)
      {
        high=se;
      }
    }
    block_index = (block_index+1) & BLOCK_BUFFER_MASK;
  }
   
  float t=autotemp_min+high*autotemp_factor;
  
  if(t<autotemp_min)
    t=autotemp_min;
  
  if(t>autotemp_max)
    t=autotemp_max;
  
  if(oldt>t)
  {
    t=AUTOTEMP_OLDWEIGHT*oldt+(1-AUTOTEMP_OLDWEIGHT)*t;
  }
  oldt=t;
  autotemp_setpoint = (int)t;

}
#endif



// Stepper

// intRes = intIn1 * intIn2 >> 16
// uses:
// r26 to store 0
// r27 to store the byte 1 of the 24 bit result
#define MultiU16X8toH16(intRes, charIn1, intIn2) \
asm volatile ( \
"clr r26 \n\t" \
"mul %A1, %B2 \n\t" \
"movw %A0, r0 \n\t" \
"mul %A1, %A2 \n\t" \
"add %A0, r1 \n\t" \
"adc %B0, r26 \n\t" \
"lsr r0 \n\t" \
"adc %A0, r26 \n\t" \
"adc %B0, r26 \n\t" \
"clr r1 \n\t" \
: \
"=&r" (intRes) \
: \
"d" (charIn1), \
"d" (intIn2) \
: \
"r26" \
)

// intRes = longIn1 * longIn2 >> 24
// uses:
// r26 to store 0
// r27 to store the byte 1 of the 48bit result
#define MultiU24X24toH16(intRes, longIn1, longIn2) \
asm volatile ( \
"clr r26 \n\t" \
"mul %A1, %B2 \n\t" \
"mov r27, r1 \n\t" \
"mul %B1, %C2 \n\t" \
"movw %A0, r0 \n\t" \
"mul %C1, %C2 \n\t" \
"add %B0, r0 \n\t" \
"mul %C1, %B2 \n\t" \
"add %A0, r0 \n\t" \
"adc %B0, r1 \n\t" \
"mul %A1, %C2 \n\t" \
"add r27, r0 \n\t" \
"adc %A0, r1 \n\t" \
"adc %B0, r26 \n\t" \
"mul %B1, %B2 \n\t" \
"add r27, r0 \n\t" \
"adc %A0, r1 \n\t" \
"adc %B0, r26 \n\t" \
"mul %C1, %A2 \n\t" \
"add r27, r0 \n\t" \
"adc %A0, r1 \n\t" \
"adc %B0, r26 \n\t" \
"mul %B1, %A2 \n\t" \
"add r27, r1 \n\t" \
"adc %A0, r26 \n\t" \
"adc %B0, r26 \n\t" \
"lsr r27 \n\t" \
"adc %A0, r26 \n\t" \
"adc %B0, r26 \n\t" \
"clr r1 \n\t" \
: \
"=&r" (intRes) \
: \
"d" (longIn1), \
"d" (longIn2) \
: \
"r26" , "r27" \
)

// Some useful constants

#define ENABLE_STEPPER_DRIVER_INTERRUPT()  TIMSK1 |= (1<<OCIE1A)
#define DISABLE_STEPPER_DRIVER_INTERRUPT() TIMSK1 &= ~(1<<OCIE1A)

uint8_t check_endstops = 1;
void enable_endstops(uint8_t check)
{
  check_endstops = check;
}

#ifdef ENDSTOPS_ONLY_FOR_HOMING
  #define CHECK_ENDSTOPS  if(check_endstops)
#else
  #define CHECK_ENDSTOPS
#endif

static block_t *current_block;  // A pointer to the block currently being traced

// Variables used by The Stepper Driver Interrupt
static unsigned char out_bits;        // The next stepping-bits to be output
static long counter_x,       // Counter variables for the bresenham line tracer
            counter_y, 
            counter_z,       
            counter_e;
static unsigned long step_events_completed; // The number of step events executed in the current block
static unsigned char busy = 0; // TRUE when SIG_OUTPUT_COMPARE1A is being serviced. Used to avoid retriggering that handler.
static long acceleration_time, deceleration_time;
static unsigned short acc_step_rate; // needed for deceleration start point
static char step_loops;
static unsigned short OCR1A_nominal;

static volatile uint8_t endstop_x_hit=0;
static volatile uint8_t endstop_y_hit=0;
static volatile uint8_t endstop_z_hit=0;

#if X_MIN_PIN > -1
  static uint8_t old_x_min_endstop=0;
#endif
#if X_MAX_PIN > -1
  static uint8_t old_x_max_endstop=0;
#endif
#if Y_MIN_PIN > -1
  static uint8_t old_y_min_endstop=0;
#endif
#if Y_MAX_PIN > -1
  static uint8_t old_y_max_endstop=0;
#endif
#if Z_MIN_PIN > -1
  static uint8_t old_z_min_endstop=0;
#endif
#if Z_MAX_PIN > -1
  static uint8_t old_z_max_endstop=0;
#endif



//         __________________________
//        /|                        |\     _________________         ^
//       / |                        | \   /|               |\        |
//      /  |                        |  \ / |               | \       s
//     /   |                        |   |  |               |  \      p
//    /    |                        |   |  |               |   \     e
//   +-----+------------------------+---+--+---------------+----+    e
//   |               BLOCK 1            |      BLOCK 2          |    d
//
//                           time ----->
// 
//  The trapezoid is the shape of the speed curve over time. It starts at block->initial_rate, accelerates 
//  first block->accelerate_until step_events_completed, then keeps going at constant speed until 
//  step_events_completed reaches block->decelerate_after after which it decelerates until the trapezoid generator is reset.
//  The slope of acceleration is calculated with the leib ramp alghorithm.

void st_wake_up() 
{
  //  TCNT1 = 0;
  if(!busy) 
  ENABLE_STEPPER_DRIVER_INTERRUPT();  
}

unsigned short calc_timer(unsigned short step_rate)
{
  unsigned short timer;
  if(step_rate > MAX_STEP_FREQUENCY) step_rate = MAX_STEP_FREQUENCY;
  
  if(step_rate > 20000) { // If steprate > 20kHz >> step 4 times
    step_rate = (step_rate >> 2)&0x3fff;
    step_loops = 4;
  }
  else if(step_rate > 10000) { // If steprate > 10kHz >> step 2 times
    step_rate = (step_rate >> 1)&0x7fff;
    step_loops = 2;
  }
  else {
    step_loops = 1;
  } 
  
  if(step_rate < (F_CPU/500000)) step_rate = (F_CPU/500000);
  step_rate -= (F_CPU/500000); // Correct for minimal speed
  
  if(step_rate >= (8*256)) // higher step rate 
  { // higher step rate 
    unsigned short table_address = (unsigned short)&speed_lookuptable_fast[(unsigned char)(step_rate>>8)][0];
    unsigned char tmp_step_rate = (step_rate & 0x00ff);
    unsigned short gain = (unsigned short)pgm_read_word_near(table_address+2);
    MultiU16X8toH16(timer, tmp_step_rate, gain);
    timer = (unsigned short)pgm_read_word_near(table_address) - timer;
  }
  else 
  { // lower step rates
    unsigned short table_address = (unsigned short)&speed_lookuptable_slow[0][0];
    table_address += ((step_rate)>>1) & 0xfffc;
    timer = (unsigned short)pgm_read_word_near(table_address);
    timer -= (((unsigned short)pgm_read_word_near(table_address+2) * (unsigned char)(step_rate & 0x0007))>>3);
  }
  if(timer < 100) { timer = 100; }//(20kHz this should never happen)
  return timer;
}

// Initializes the trapezoid generator from the current block. Called whenever a new 
// block begins.
void trapezoid_generator_reset()
{
  deceleration_time = 0;
  
  
  // step_rate to timer interval
  acc_step_rate = current_block->initial_rate;
  acceleration_time = calc_timer(acc_step_rate);
  OCR1A = acceleration_time;
  OCR1A_nominal = calc_timer(current_block->nominal_rate);
    
}

// "The Stepper Driver Interrupt" - This timer interrupt is the workhorse.  
// It pops blocks from the block_buffer and executes them by pulsing the stepper pins appropriately. 
ISR(TIMER1_COMPA_vect)
{
	uint32_t isr_start_micros;
	
	PreemptionFlag |= 0x0001;
	
	isr_start_micros = micros();
	
  // If there is no current block, attempt to pop one from the buffer
  if (current_block == NULL) {
    // Anything in the buffer?
    current_block = plan_get_current_block();
    if (current_block != NULL) {
      trapezoid_generator_reset();
      counter_x = -(current_block->step_event_count >> 1);
      counter_y = counter_x;
      counter_z = counter_x;
      counter_e = counter_x;
      step_events_completed = 0;
    } 
    else {
        OCR1A=2000; // 1kHz.
    }    
  } 

  if (current_block != NULL) {
    // Set directions TO DO This should be done once during init of trapezoid. Endstops -> interrupt
    out_bits = current_block->direction_bits;

    // Set direction and check limit switches
    if ((out_bits & (1<<X_AXIS)) != 0) {   // -direction
      WRITE(X_DIR_PIN, INVERT_X_DIR);
      CHECK_ENDSTOPS
      {
        #if X_MIN_PIN > -1
          uint8_t x_min_endstop=(READ(X_MIN_PIN) != X_ENDSTOP_INVERT);
          if(x_min_endstop && old_x_min_endstop && (current_block->steps_x > 0)) {
            if(!is_homing)
              endstop_x_hit=1;
            else  
              step_events_completed = current_block->step_event_count;
          }
          else
          {
            endstop_x_hit=0;
          }
          old_x_min_endstop = x_min_endstop;
        #else
          endstop_x_hit=0;
        #endif
      }
    }
    else { // +direction 
      WRITE(X_DIR_PIN,!INVERT_X_DIR);
      CHECK_ENDSTOPS 
      {
        #if X_MAX_PIN > -1
          uint8_t x_max_endstop=(READ(X_MAX_PIN) != X_ENDSTOP_INVERT);
          if(x_max_endstop && old_x_max_endstop && (current_block->steps_x > 0)){
            if(!is_homing)
              endstop_x_hit=1;
            else    
              step_events_completed = current_block->step_event_count;
          }
          else
          {
            endstop_x_hit=0;
          }
          old_x_max_endstop = x_max_endstop;
        #else
          endstop_x_hit=0;
        #endif
      }
    }

    if ((out_bits & (1<<Y_AXIS)) != 0) {   // -direction
      WRITE(Y_DIR_PIN,INVERT_Y_DIR);
      CHECK_ENDSTOPS
      {
        #if Y_MIN_PIN > -1
          uint8_t y_min_endstop=(READ(Y_MIN_PIN) != Y_ENDSTOP_INVERT);
          if(y_min_endstop && old_y_min_endstop && (current_block->steps_y > 0)) {
            if(!is_homing)
              endstop_y_hit=1;
            else
              step_events_completed = current_block->step_event_count;
          }
          else
          {
            endstop_y_hit=0;
          }
          old_y_min_endstop = y_min_endstop;
        #else
          endstop_y_hit=0;  
        #endif
      }
    }
    else { // +direction
      WRITE(Y_DIR_PIN,!INVERT_Y_DIR);
      CHECK_ENDSTOPS
      {
        #if Y_MAX_PIN > -1
          uint8_t y_max_endstop=(READ(Y_MAX_PIN) != Y_ENDSTOP_INVERT);
          if(y_max_endstop && old_y_max_endstop && (current_block->steps_y > 0)){
            if(!is_homing)
              endstop_y_hit=1;
            else  
              step_events_completed = current_block->step_event_count;
          }
          else
          {
            endstop_y_hit=0;
          }
          old_y_max_endstop = y_max_endstop;
        #else
          endstop_y_hit=0;  
        #endif
      }
    }

    if ((out_bits & (1<<Z_AXIS)) != 0) {   // -direction
      WRITE(Z_DIR_PIN,INVERT_Z_DIR);
      CHECK_ENDSTOPS
      {
        #if Z_MIN_PIN > -1
          uint8_t z_min_endstop=(READ(Z_MIN_PIN) != Z_ENDSTOP_INVERT);
          if(z_min_endstop && old_z_min_endstop && (current_block->steps_z > 0)) {
            if(!is_homing)  
              endstop_z_hit=1;
            else  
              step_events_completed = current_block->step_event_count;
          }
          else
          {
            endstop_z_hit=0;
          }
          old_z_min_endstop = z_min_endstop;
        #else
          endstop_z_hit=0;  
        #endif
      }
    }
    else { // +direction
      WRITE(Z_DIR_PIN,!INVERT_Z_DIR);
      CHECK_ENDSTOPS
      {
        #if Z_MAX_PIN > -1
          uint8_t z_max_endstop=(READ(Z_MAX_PIN) != Z_ENDSTOP_INVERT);
          if(z_max_endstop && old_z_max_endstop && (current_block->steps_z > 0)) {
            if(!is_homing)
              endstop_z_hit=1;
            else  
              step_events_completed = current_block->step_event_count;
          }
          else
          {
            endstop_z_hit=0;
          }
          old_z_max_endstop = z_max_endstop;
        #else
          endstop_z_hit=0;  
        #endif
      }
    }

    if ((out_bits & (1<<E_AXIS)) != 0) {  // -direction
      WRITE(E_DIR_PIN,INVERT_E_DIR);
    }
    else { // +direction
      WRITE(E_DIR_PIN,!INVERT_E_DIR);
    }
    
    for(int8_t i=0; i < step_loops; i++) { // Take multiple steps per interrupt (For high speed moves) 
      
      counter_x += current_block->steps_x;
      if (counter_x > 0) {
        if(!endstop_x_hit)
        {
          if(virtual_steps_x)
            virtual_steps_x--;
          else
            WRITE(X_STEP_PIN, HIGH);
        }
        else
          virtual_steps_x++;
          
        counter_x -= current_block->step_event_count;
      }

      counter_y += current_block->steps_y;
      if (counter_y > 0) {
        if(!endstop_y_hit)
        {
          if(virtual_steps_y)
            virtual_steps_y--;
          else
            WRITE(Y_STEP_PIN, HIGH);
        }
        else
          virtual_steps_y++;
            
        counter_y -= current_block->step_event_count;
      }

      counter_z += current_block->steps_z;
      if (counter_z > 0) {
        if(!endstop_z_hit)
        {
          if(virtual_steps_z)
            virtual_steps_z--;
          else
            WRITE(Z_STEP_PIN, HIGH);
        }
        else
          virtual_steps_z++;
          
        counter_z -= current_block->step_event_count;        
      }

      counter_e += current_block->steps_e;
      if (counter_e > 0) {
        WRITE(E_STEP_PIN, HIGH);
        counter_e -= current_block->step_event_count;
      }
	  
	  // ******************* DIRTY HACK *******************
	  // *** JTK Wong (XTRONTEC Limited); 28 November 2012;
	  // nop instruction added to ensure that pulse width generated meets the
	  // minimum requirement for the Allegro A4982 stepper motor driver input.
	  // The existing code relies on delays in executing commands between setting
	  // the step high and stepping it low again to meet the A4982's minimum pulse 
	  // width requirement. Some motor drive issues have been seen during 
	  // development testing which may be related to this.
		
	  // *** NOTE ***
	  // THIS IS A QUICK AND DIRTY HACK TO GET THINGS WORKING A
	  // BIT BETTER. RECOMMEND THAT THIS ISR IS RE-VISITED AND REFACTORED TO
	  // TO WORK PROPERLY AND GENERATE THE CORRECT PULSE WIDTHS FOR THE A4982
	  // DRIVER IC.
	  for(int8_t step_pulse_width=0; step_pulse_width < 8; step_pulse_width++)
	  {
	 	asm volatile("nop");
	  }
	  // ******************* END OF DIRTY HACK *******************
        
	  WRITE(X_STEP_PIN, LOW);
	  WRITE(Y_STEP_PIN, LOW);
	  WRITE(Z_STEP_PIN, LOW);
	  WRITE(E_STEP_PIN, LOW);

	  // ******************* DIRTY HACK *******************
	  // *** JTK Wong (XTRONTEC Limited); 30 November 2012;
	  // nop instruction added to ensure that pulse width generated meets the
	  // minimum requirement for the Allegro A4982 stepper motor driver input.
	  // The existing code relies on delays in executing commands between setting
	  // the step high and stepping it low again to meet the A4982's minimum pulse 
	  // width requirement. Some motor drive issues have been seen during 
	  // development testing which may be related to this.
		
	  // *** NOTE ***
	  // THIS IS A QUICK AND DIRTY HACK TO GET THINGS WORKING A
	  // BIT BETTER. RECOMMEND THAT THIS ISR IS RE-VISITED AND REFACTORED TO
	  // TO WORK PROPERLY AND GENERATE THE CORRECT PULSE WIDTHS FOR THE A4982
	  // DRIVER IC.
	  for(int8_t step_pulse_width=0; step_pulse_width < 16; step_pulse_width++)
	  {
	 	asm volatile("nop");
	  }
	  // ******************* END OF DIRTY HACK *******************
	  
	  
      step_events_completed += 1;  
      if(step_events_completed >= current_block->step_event_count) break;
    }
    // Calculare new timer value
    unsigned short timer;
    unsigned short step_rate;
    if (step_events_completed <= (unsigned long int)current_block->accelerate_until) {
      
      MultiU24X24toH16(acc_step_rate, acceleration_time, current_block->acceleration_rate);
      acc_step_rate += current_block->initial_rate;
      
      // upper limit
      if(acc_step_rate > current_block->nominal_rate)
        acc_step_rate = current_block->nominal_rate;

      // step_rate to timer interval
      timer = calc_timer(acc_step_rate);
      OCR1A = timer;
      acceleration_time += timer;
    } 
    else if (step_events_completed > (unsigned long int)current_block->decelerate_after) {   
      MultiU24X24toH16(step_rate, deceleration_time, current_block->acceleration_rate);
      
      if(step_rate > acc_step_rate) { // Check step_rate stays positive
        step_rate = current_block->final_rate;
      }
      else {
        step_rate = acc_step_rate - step_rate; // Decelerate from aceleration end point.
      }

      // lower limit
      if(step_rate < current_block->final_rate)
        step_rate = current_block->final_rate;

      // step_rate to timer interval
      timer = calc_timer(step_rate);
      OCR1A = timer;
      deceleration_time += timer;
    }
    else {
      OCR1A = OCR1A_nominal;
    }

    // If current block is finished, reset pointer 
    if (step_events_completed >= current_block->step_event_count) {
      current_block = NULL;
      plan_discard_current_block();
    }   
  }

	timer1_compa_isr_exe_micros = (micros() - isr_start_micros);
	
	if (timer1_compa_isr_exe_micros > timer1_compa_isr_exe_micros_max)
	{
		timer1_compa_isr_exe_micros_max = timer1_compa_isr_exe_micros;
	}
	
	if (timer1_compa_isr_exe_micros < timer1_compa_isr_exe_micros_min)
	{
		timer1_compa_isr_exe_micros_min = timer1_compa_isr_exe_micros;
	}
}


void st_init()
{
  // waveform generation = 0100 = CTC
  TCCR1B &= ~(1<<WGM13);
  TCCR1B |=  (1<<WGM12);
  TCCR1A &= ~(1<<WGM11); 
  TCCR1A &= ~(1<<WGM10);

  // output mode = 00 (disconnected)
  TCCR1A &= ~(3<<COM1A0); 
  TCCR1A &= ~(3<<COM1B0); 

  // Set the timer pre-scaler
  // Generally we use a divider of 8, resulting in a 2MHz timer
  // frequency on a 16MHz MCU. If you are going to change this, be
  // sure to regenerate speed_lookuptable.h with
  // create_speed_lookuptable.py
  TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (2<<CS10); // 2MHz timer

  OCR1A = 0x4000;
  TCNT1 = 0;
  ENABLE_STEPPER_DRIVER_INTERRUPT();

  #ifdef ENDSTOPS_ONLY_FOR_HOMING
    enable_endstops(0);
  #else
    enable_endstops(1);
  #endif
  
  sei();
}

void check_axes_activity() {
  unsigned char x_active = 0;
  unsigned char y_active = 0;
  unsigned char z_active = 0;
  unsigned char e_active = 0;
  block_t *block;

  if(block_buffer_tail != block_buffer_head) {
    uint8_t block_index = block_buffer_tail;
    while(block_index != block_buffer_head) {
      block = &block_buffer[block_index];
      if(block->steps_x != 0) x_active++;
      if(block->steps_y != 0) y_active++;
      if(block->steps_z != 0) z_active++;
      if(block->steps_e != 0) e_active++;
      block_index = (block_index+1) & BLOCK_BUFFER_MASK;
    }
  }
  if((DISABLE_X) && (x_active == 0)) disable_x();
  if((DISABLE_Y) && (y_active == 0)) disable_y();
  if((DISABLE_Z) && (z_active == 0)) disable_z();
  if((DISABLE_E) && (e_active == 0)) disable_e();
}
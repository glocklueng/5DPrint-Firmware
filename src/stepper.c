/*
  Makibox A6 Firmware
  Based on Sprinter (master branch, 1 Sep 2012).
  Designed for Printrboard (Rev B).
  ---
  Copyright (c) 2012-2013 by Makible Limited.
 
  This file is part of the Makibox A6 Firmware.
 
  Makibox A6 Firmware is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  The Makibox A6 Firmware is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with the Makibox A6 Firmware.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <avr/interrupt.h>
#include <stdlib.h>

#include "config.h"
#include "makibox.h"
#include "board_io.h"
#include "pins.h"
#include "pins_teensy.h"
#include "speed_lookuptable.h"
#include "planner.h"
#include "stepper.h"
#include "heater.h"

#if DIGIPOTS > 0
#include "i2c/Master_I2C_Comms.h"
#include "store_eeprom.h"
#endif

#ifndef CRITICAL_SECTION_START
#define CRITICAL_SECTION_START  unsigned char _sreg = SREG; cli()
#define CRITICAL_SECTION_END    SREG = _sreg
#endif //CRITICAL_SECTION_START

#if (DEBUG > -1)
uint32_t timer1_compa_isr_exe_micros = 0;
uint32_t timer1_compa_isr_exe_micros_min = 0xFFFFFFFF;
uint32_t timer1_compa_isr_exe_micros_max = 0;
#endif


// actual_steps_ are used to try and keep track of the actual positions of the
// axes and extruder drive. System must first be 'homed' so we know where the
// zero points are.
static long actual_steps_x = 0;
static long actual_steps_y = 0;
static long actual_steps_z = 0;
static long actual_steps_e = 0;

unsigned short virtual_steps_x = 0;
unsigned short virtual_steps_y = 0;
unsigned short virtual_steps_z = 0;

uint8_t is_homing = 0;

uint8_t pause_print_req = 0;
paused_data_t paused_data = {0};



// Stepper

// intRes = intIn1 * intIn2 >> 16
// uses:
// r26 to store 0
// r27 to store the byte 1 of the 24 bit result
#define MultiU16X8toH16(intRes, charIn1, intIn2)                        \
    asm volatile (                                                      \
                  "clr r26 \n\t"                                        \
                  "mul %A1, %B2 \n\t"                                   \
                  "movw %A0, r0 \n\t"                                   \
                  "mul %A1, %A2 \n\t"                                   \
                  "add %A0, r1 \n\t"                                    \
                  "adc %B0, r26 \n\t"                                   \
                  "lsr r0 \n\t"                                         \
                  "adc %A0, r26 \n\t"                                   \
                  "adc %B0, r26 \n\t"                                   \
                  "clr r1 \n\t"                                         \
                  :                                                     \
                                                                        "=&r" (intRes) \
                  :                                                     \
                                                                        "d" (charIn1), \
                                                                        "d" (intIn2) \
                  :                                                     \
                                                                        "r26" \
                                                                        )

// intRes = longIn1 * longIn2 >> 24
// uses:
// r26 to store 0
// r27 to store the byte 1 of the 48bit result
#define MultiU24X24toH16(intRes, longIn1, longIn2)                      \
    asm volatile (                                                      \
                  "clr r26 \n\t"                                        \
                  "mul %A1, %B2 \n\t"                                   \
                  "mov r27, r1 \n\t"                                    \
                  "mul %B1, %C2 \n\t"                                   \
                  "movw %A0, r0 \n\t"                                   \
                  "mul %C1, %C2 \n\t"                                   \
                  "add %B0, r0 \n\t"                                    \
                  "mul %C1, %B2 \n\t"                                   \
                  "add %A0, r0 \n\t"                                    \
                  "adc %B0, r1 \n\t"                                    \
                  "mul %A1, %C2 \n\t"                                   \
                  "add r27, r0 \n\t"                                    \
                  "adc %A0, r1 \n\t"                                    \
                  "adc %B0, r26 \n\t"                                   \
                  "mul %B1, %B2 \n\t"                                   \
                  "add r27, r0 \n\t"                                    \
                  "adc %A0, r1 \n\t"                                    \
                  "adc %B0, r26 \n\t"                                   \
                  "mul %C1, %A2 \n\t"                                   \
                  "add r27, r0 \n\t"                                    \
                  "adc %A0, r1 \n\t"                                    \
                  "adc %B0, r26 \n\t"                                   \
                  "mul %B1, %A2 \n\t"                                   \
                  "add r27, r1 \n\t"                                    \
                  "adc %A0, r26 \n\t"                                   \
                  "adc %B0, r26 \n\t"                                   \
                  "lsr r27 \n\t"                                        \
                  "adc %A0, r26 \n\t"                                   \
                  "adc %B0, r26 \n\t"                                   \
                  "clr r1 \n\t"                                         \
                  :                                                     \
                                                                        "=&r" (intRes) \
                  :                                                     \
                                                                        "d" (longIn1), \
                                                                        "d" (longIn2) \
                  :                                                     \
                                                                        "r26" , "r27" \
                                                                        )

// Some useful constants

uint8_t check_endstops = 1;
void enable_endstops(uint8_t check){
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

block_t resume_buffer[PRINT_PAUSED_BLOCK_BUF_SIZE] = {{0}};

#if DIGIPOTS > 0
unsigned short max_x_motor_current = XAXIS_DEFAULT_MAX_CURRENT;	// mA
unsigned short max_y_motor_current = YAXIS_DEFAULT_MAX_CURRENT;	// mA
unsigned short max_z_motor_current = ZAXIS_DEFAULT_MAX_CURRENT;	// mA
unsigned short max_e_motor_current = EAXIS_DEFAULT_MAX_CURRENT;	// mA
#endif

#if SET_MICROSTEP > 0
// Default microstep resolution = 16th step
unsigned short microstep_x[2]={HIGH, HIGH};
unsigned short microstep_y[2]={HIGH, HIGH};
unsigned short microstep_z[2]={HIGH, HIGH};
unsigned short microstep_e[2]={HIGH, HIGH};
#endif


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

void st_wake_up() {
    //  TCNT1 = 0;
    if(!busy) ENABLE_STEPPER_DRIVER_INTERRUPT();  
}


void st_sleep() {
    //  TCNT1 = 0;
    if(!busy) DISABLE_STEPPER_DRIVER_INTERRUPT();  
}


unsigned short calc_timer(unsigned short step_rate) {
    unsigned short timer;
    unsigned long fcpu_div_500k = (F_CPU/500000L);
  
    if(step_rate > MAX_STEP_FREQUENCY) step_rate = MAX_STEP_FREQUENCY;
  
    if(step_rate > 20000) { // If steprate > 20kHz >> step 4 times
        step_rate = (step_rate >> 2)&0x3fff;
        step_loops = 4;
    }
    else if(step_rate > 10000) { // If steprate > 10kHz >> step 2 times
        step_rate = (step_rate >> 1)&0x7fff;
        step_loops = 2;
    }
    else step_loops = 1; 
  
    if(step_rate < fcpu_div_500k) step_rate = fcpu_div_500k;
    step_rate -= fcpu_div_500k; // Correct for minimal speed
  
    //if(step_rate >= (8*256)) // higher step rate
    if(step_rate >= 2048) {// higher step rate
        unsigned short table_address = (unsigned short)&speed_lookuptable_fast[(unsigned char)(step_rate>>8)][0];
        unsigned char tmp_step_rate = (step_rate & 0x00ff);
        unsigned short gain = (unsigned short)pgm_read_word_near(table_address+2);
        MultiU16X8toH16(timer, tmp_step_rate, gain);
        timer = (unsigned short)pgm_read_word_near(table_address) - timer;
    }
    else { // lower step rates
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
void trapezoid_generator_reset() {
    deceleration_time = 0;
    
    // step_rate to timer interval
    acc_step_rate = current_block->initial_rate;
    acceleration_time = calc_timer(acc_step_rate);
    OCR1A = acceleration_time;
    OCR1A_nominal = calc_timer(current_block->nominal_rate);

    // Set microstep resolution
#if X_MS1_PIN > -1
    WRITE(X_MS1_PIN, microstep_x[0]);
#endif
#if X_MX2_PIN > -1
    WRITE(X_MS2_PIN, microstep_x[1]);
#endif
#if Y_MS1_PIN > -1
    WRITE(Y_MS1_PIN, microstep_y[0]);
#endif
#if Y_MX2_PIN > -1
    WRITE(Y_MS2_PIN, microstep_y[1]);
#endif
#if Z_MS1_PIN > -1
    WRITE(Z_MS1_PIN, microstep_z[0]);
#endif
#if Z_MS2_PIN > -1
    WRITE(Z_MS2_PIN, microstep_z[1]);
#endif
#if E_MS1_PIN > -1
    WRITE(E_MS1_PIN, microstep_e[0]);
#endif
#if E_MS2_PIN > -1
    WRITE(E_MS2_PIN, microstep_e[1]);
#endif
}

// "The Stepper Driver Interrupt" - This timer interrupt is the workhorse.  
// It pops blocks from the block_buffer and executes them by pulsing the stepper pins appropriately. 
ISR(TIMER1_COMPA_vect) {
#if (DEBUG > -1)
    uint32_t isr_start_micros;		
    isr_start_micros = micros();	
    PreemptionFlag |= 0x0001;
#endif
	
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
        else OCR1A=2000; // 1kHz.
    } 

    if (current_block != NULL) {
        // Disable Motors If Not Used
	// Reduces power dissipation in stepper motors
	if (DISABLE_X) {
            if (current_block->steps_x > 0)	enable_x();
            else disable_x();
        }
        if (DISABLE_Y) {
            if (current_block->steps_y > 0) enable_y();
            else disable_y();
        }
        if (DISABLE_Z) {
            if (current_block->steps_z > 0)	enable_z();
            else disable_z();
        }
        if (DISABLE_E) {
            if (current_block->steps_e > 0) enable_e();
            else disable_e();
        }
    
	// Set directions TO DO This should be done once during init of trapezoid. Endstops -> interrupt
        out_bits = current_block->direction_bits;

        // Set direction and check limit switches
        if ((out_bits & (1<<X_AXIS)) != 0) {   // -direction
            WRITE(X_DIR_PIN, INVERT_X_DIR);
            CHECK_ENDSTOPS {
#if X_MIN_PIN > -1
                uint8_t x_min_endstop=(READ(X_MIN_PIN) != X_ENDSTOP_INVERT);
                if(x_min_endstop && old_x_min_endstop && (current_block->steps_x > 0)) {
                    if(!is_homing)  endstop_x_hit=1;
                    else step_events_completed = current_block->step_event_count;
                }
                else endstop_x_hit=0;
                old_x_min_endstop = x_min_endstop;
#else
                endstop_x_hit=0;
#endif
            }
        }
        else { // +direction 
            WRITE(X_DIR_PIN,!INVERT_X_DIR);
            CHECK_ENDSTOPS {
#if X_MAX_PIN > -1
                uint8_t x_max_endstop=(READ(X_MAX_PIN) != X_ENDSTOP_INVERT);
                if(x_max_endstop && old_x_max_endstop && (current_block->steps_x > 0)){
                    if(!is_homing) endstop_x_hit=1;
                    else step_events_completed = current_block->step_event_count;
                }
                else endstop_x_hit=0;
                       
                old_x_max_endstop = x_max_endstop;
#else
                endstop_x_hit=0;
#endif
            }
        }

        if ((out_bits & (1<<Y_AXIS)) != 0) {   // -direction
            WRITE(Y_DIR_PIN,INVERT_Y_DIR);
            CHECK_ENDSTOPS {
#if Y_MIN_PIN > -1
                uint8_t y_min_endstop=(READ(Y_MIN_PIN) != Y_ENDSTOP_INVERT);
                if(y_min_endstop && old_y_min_endstop && (current_block->steps_y > 0)) {
                    if(!is_homing) endstop_y_hit=1;
                    else step_events_completed = current_block->step_event_count;
                }
                else endstop_y_hit=0;
                old_y_min_endstop = y_min_endstop;
#else
                endstop_y_hit=0;  
#endif
            }
        }
        else { // +direction
            WRITE(Y_DIR_PIN,!INVERT_Y_DIR);
            CHECK_ENDSTOPS {
#if Y_MAX_PIN > -1
                uint8_t y_max_endstop=(READ(Y_MAX_PIN) != Y_ENDSTOP_INVERT);
                if(y_max_endstop && old_y_max_endstop && (current_block->steps_y > 0)){
                    if(!is_homing) endstop_y_hit=1;
                    else step_events_completed = current_block->step_event_count;
                }
                else endstop_y_hit=0;
                old_y_max_endstop = y_max_endstop;
#else
                endstop_y_hit=0;  
#endif
            }
        }

        if ((out_bits & (1<<Z_AXIS)) != 0) {   // -direction
            WRITE(Z_DIR_PIN,INVERT_Z_DIR);
            CHECK_ENDSTOPS {
#if Z_MIN_PIN > -1
                uint8_t z_min_endstop=(READ(Z_MIN_PIN) != Z_ENDSTOP_INVERT);
                if(z_min_endstop && old_z_min_endstop && (current_block->steps_z > 0)) {
                    if(!is_homing) endstop_z_hit=1;
                    else step_events_completed = current_block->step_event_count;
                }
                else endstop_z_hit=0;
                old_z_min_endstop = z_min_endstop;
#else
                endstop_z_hit=0;  
#endif
            }
        }
        else { // +direction
            WRITE(Z_DIR_PIN,!INVERT_Z_DIR);
            CHECK_ENDSTOPS {
#if Z_MAX_PIN > -1
                uint8_t z_max_endstop=(READ(Z_MAX_PIN) != Z_ENDSTOP_INVERT);
                if(z_max_endstop && old_z_max_endstop && (current_block->steps_z > 0)) {
                    if(!is_homing) endstop_z_hit=1;
                    else step_events_completed = current_block->step_event_count;
                }
                else endstop_z_hit=0;
                old_z_max_endstop = z_max_endstop;
#else
                endstop_z_hit=0;  
#endif
            }
        }

        if ((out_bits & (1<<E_AXIS)) != 0) WRITE(E_DIR_PIN,INVERT_E_DIR); // -direction
        else WRITE(E_DIR_PIN,!INVERT_E_DIR); // +direction
    
        for(int8_t i=0; i < step_loops; i++) { // Take multiple steps per interrupt (For high speed moves) 
            counter_x += current_block->steps_x;
            if (counter_x > 0) {
                if(!endstop_x_hit) {
                    if(virtual_steps_x) virtual_steps_x--;
                    else {
                        WRITE(X_STEP_PIN, HIGH);
                        // Keeping track of stepper positions
                        if (current_block->direction_bits & (1 << X_AXIS)) actual_steps_x--;
                        //actual_steps_x ? actual_steps_x-- : 0;
                        else actual_steps_x++;
                    }
                }
                else virtual_steps_x++;
                counter_x -= current_block->step_event_count;
            }

            counter_y += current_block->steps_y;
            if (counter_y > 0) {
                if(!endstop_y_hit) {
                    if(virtual_steps_y) virtual_steps_y--;
                    else {
                        WRITE(Y_STEP_PIN, HIGH);
                        // Keeping track of stepper positions
                        if (current_block->direction_bits & (1 << Y_AXIS)) actual_steps_y--;
                        //actual_steps_y ? actual_steps_y-- : 0;
                        else actual_steps_y++;
                    }
                }
                else virtual_steps_y++;
                counter_y -= current_block->step_event_count;
            }

            counter_z += current_block->steps_z;
            if (counter_z > 0) {
                if(!endstop_z_hit) {
                    if(virtual_steps_z) virtual_steps_z--;
                    else {
                        WRITE(Z_STEP_PIN, HIGH);
                        // Keeping track of stepper positions
                        if (current_block->direction_bits & (1 << Z_AXIS)) actual_steps_z--;
                        //actual_steps_z ? actual_steps_z-- : 0;
                        else actual_steps_z++;
                    }
                }
                else virtual_steps_z++;
                counter_z -= current_block->step_event_count;        
            }

            counter_e += current_block->steps_e;
            if (counter_e > 0) {
		WRITE(E_STEP_PIN, HIGH);		
		// Keeping track of stepper positions
		if (current_block->direction_bits & (1 << E_AXIS)) actual_steps_e--;
		else actual_steps_e++;
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
            if(acc_step_rate > current_block->nominal_rate) acc_step_rate = current_block->nominal_rate;

            // step_rate to timer interval
            timer = calc_timer(acc_step_rate);
            OCR1A = timer;
            acceleration_time += timer;
        } 
        else if (step_events_completed > (unsigned long int)current_block->decelerate_after) {   
            MultiU24X24toH16(step_rate, deceleration_time, current_block->acceleration_rate);

            // Check step_rate stays positive
            if(step_rate > acc_step_rate) step_rate = current_block->final_rate;
            else step_rate = acc_step_rate - step_rate; // Decelerate from aceleration end point.

            // lower limit
            if(step_rate < current_block->final_rate) step_rate = current_block->final_rate;

            // step_rate to timer interval
            timer = calc_timer(step_rate);
            OCR1A = timer;
            deceleration_time += timer;
        }
        else OCR1A = OCR1A_nominal;

        // If current block is finished, reset pointer 
        if (step_events_completed >= current_block->step_event_count) {
            current_block = NULL;
            plan_discard_current_block();
	  
            if (pause_print_req) {	
                get_current_printer_state();
                set_print_paused_buffer();
                // Clear the plan buffer
                clear_plan_buffer();
                // de-assert 'pause_print_req' flag
                pause_print_req = 0;
            }
        }   
    }
    else {
        if (pause_print_req) {	
            get_current_printer_state();
            set_print_paused_buffer();
            // Clear the plan buffer
            clear_plan_buffer();
            // de-assert 'pause_print_req' flag
            pause_print_req = 0;
        }
    }
  
#if (DEBUG > -1)
    timer1_compa_isr_exe_micros = (micros() - isr_start_micros);
    
    if (timer1_compa_isr_exe_micros > timer1_compa_isr_exe_micros_max)
        timer1_compa_isr_exe_micros_max = timer1_compa_isr_exe_micros;
    
    if (timer1_compa_isr_exe_micros < timer1_compa_isr_exe_micros_min)
        timer1_compa_isr_exe_micros_min = timer1_compa_isr_exe_micros;
#endif
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
    TCCR1A &= ~(3<<COM1C0);

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
  
    // Timer 1B -> Disabled
    OCR1B = 0xFFFF;
    TIMSK1 &= ~(1 << OCIE1B);
  
    // Timer 1C -> Heater Control.
    // Approx 2kHz. Interrupt Freq = Clk / (N * (1 + OCR1C)
    OCR1C = 999;
    TIMSK1 |= (1 << OCIE1C);   // enable timer 1C output compare match interrupt
  
    sei();
}


void st_set_current_position(st_position_t new_position) {
    actual_steps_x = new_position.x;
    actual_steps_y = new_position.y;
    actual_steps_z = new_position.z;
    actual_steps_e = new_position.e;
}


st_position_t st_get_current_position(void) {
    st_position_t pos;

    CRITICAL_SECTION_START;
    pos.x = actual_steps_x;
    pos.y = actual_steps_y;
    pos.z = actual_steps_z;
    pos.e = actual_steps_e;
    CRITICAL_SECTION_END;

    return pos;
}


void get_current_printer_state(void) {
    // Get current positions and target temperatures
    paused_data.paused_pos_x = actual_steps_x / (float)(axis_steps_per_unit[X_AXIS]);
    paused_data.paused_pos_y = actual_steps_y / (float)(axis_steps_per_unit[Y_AXIS]);
    paused_data.paused_pos_z = actual_steps_z / (float)(axis_steps_per_unit[Z_AXIS]);
    paused_data.paused_pos_e = actual_steps_e / (float)(axis_steps_per_unit[E_AXIS]);
	
    paused_data.hotend_target_temp = target_temp;
    paused_data.hotend_target_temp_raw = target_raw;
    paused_data.target_bed_temp_raw = target_bed_raw;
	
    // Copy first 8 buffer contents
    for (int i = 0; i < PRINT_PAUSED_BLOCK_BUF_SIZE; i++)
        resume_buffer[i] = block_buffer[i];

    // Copy current block_buffer_head and block_buffer_tail
    paused_data.block_buffer_head = block_buffer_head; 
    paused_data.block_buffer_tail = block_buffer_tail;
}


void set_print_paused_buffer(void) {
    CRITICAL_SECTION_START;
	
    block_buffer_size = PRINT_PAUSED_BLOCK_BUF_SIZE;
    block_buffer_mask = PRINT_PAUSED_BLOCK_BUF_MASK;
	
    block_buffer_tail = 0;
    block_buffer_head = 0;
	
    CRITICAL_SECTION_END;
}


void clear_plan_buffer(void) {
    st_position_t pos;
    float current_pos_in_mm[NUM_AXIS];
	
    // Reset the plan buffer
    plan_init();
	
    pos = st_get_current_position();
	
    current_pos_in_mm[X_AXIS] = pos.x / (float)(axis_steps_per_unit[X_AXIS]);
    current_pos_in_mm[Y_AXIS] = pos.y / (float)(axis_steps_per_unit[Y_AXIS]);
    current_pos_in_mm[Z_AXIS] = pos.z / (float)(axis_steps_per_unit[Z_AXIS]);
    current_pos_in_mm[E_AXIS] = pos.e / (float)(axis_steps_per_unit[E_AXIS]);
	
    plan_set_position(current_pos_in_mm[X_AXIS], current_pos_in_mm[Y_AXIS], 
                      current_pos_in_mm[Z_AXIS], current_pos_in_mm[E_AXIS]);
}


void resume_normal_print_buffer(void) {
    CRITICAL_SECTION_START;
	
    block_buffer_size = CFG_BLOCK_BUFFER_SIZE;
    block_buffer_mask = CFG_BLOCK_BUFFER_MASK;
	
    // Copy first 8 buffer contents from saved data
    for (int i = 0; i < PRINT_PAUSED_BLOCK_BUF_SIZE; i++)
        block_buffer[i] = resume_buffer[i];
	
    // Copy block_buffer_head and block_buffer_tail from saved data
    block_buffer_head = paused_data.block_buffer_head; 
    block_buffer_tail = paused_data.block_buffer_tail;
	
    CRITICAL_SECTION_END;
}


void resume_normal_buf_discard_all_buf_moves(void) {
    CRITICAL_SECTION_START;
	
    block_buffer_size = CFG_BLOCK_BUFFER_SIZE;
    block_buffer_mask = CFG_BLOCK_BUFFER_MASK;
	
    block_buffer_tail = 0;
    block_buffer_head = 0;
	
    CRITICAL_SECTION_END;
}


// Block until all buffered steps are executed
void st_synchronize()
{
    while(blocks_queued()) {
        manage_inactivity(1);
#if (MINIMUM_FAN_START_SPEED > 0)
        manage_fan_start_speed();
#endif
    }
}


// Set the current limit for Allegro A4982 stepper driver IC using the MCP4451  
// digi-pot device.
void set_stepper_motors_max_current(unsigned char Axis, unsigned short MilliAmps) {
    unsigned long WaitForI2CSendTimer = 0;
	
    switch (Axis) {
    case X_AXIS:
        I2C_digipots_set_wiper(I2C_DIGIPOT_VOL_WIPER0_ADDR, MilliAmps);
        max_x_motor_current = MilliAmps;
        break;
		
    case Y_AXIS:
        I2C_digipots_set_wiper(I2C_DIGIPOT_VOL_WIPER1_ADDR, MilliAmps);
        max_y_motor_current = MilliAmps;
        break;
		
    case Z_AXIS:
        I2C_digipots_set_wiper(I2C_DIGIPOT_VOL_WIPER2_ADDR, MilliAmps);
        max_z_motor_current = MilliAmps;
        break;
		
    case E_AXIS:
        I2C_digipots_set_wiper(I2C_DIGIPOT_VOL_WIPER3_ADDR, MilliAmps);
        max_e_motor_current = MilliAmps;
        break;
		
    default:
        break;
    }
	
    WaitForI2CSendTimer = millis();
	
    while ( I2C_Locked 
            && ( millis() - WaitForI2CSendTimer < I2C_TRANSCEIVER_BUSY_TIMEOUT ) ){
        Service_I2C_Master();	// Send I2C message to device
    }
}

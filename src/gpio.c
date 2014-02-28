/*
  5DPrint Firmware
  Based on Sprinter (master branch, 1 Sep 2012).
  Designed for Printrboard (Rev B) and 5DPrint D8 Dirver Board.
  ---
  Copyright (c) 2012-2014 by Makible Limited.
 
  This file is part of the 5DPrint Firmware.
 
  5DPrint Firmware is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  The 5DPrint Firmware is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with the 5DPrint Firmware.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "pins.h"
#include "board_io.h"
#include "gpio.h"
#include "config.h"       

#if GPIO > 0
/**
   \fn void setupGPIO()
   \brief Set up GPIO pins as output and driving them low

 */
void setupGPIO(){
    // Set all GPIO pins to output
    SET_OUTPUT(GPIO_1);
    SET_OUTPUT(GPIO_2);
    SET_OUTPUT(GPIO_3);
    SET_OUTPUT(GPIO_4);
    SET_OUTPUT(GPIO_5);
    SET_OUTPUT(GPIO_6);
    SET_OUTPUT(GPIO_7);
    SET_OUTPUT(GPIO_8);
    SET_OUTPUT(GPIO_9);
    // Set all GPIO pins to low by default
    WRITE(GPIO_1, LOW);
    WRITE(GPIO_2, LOW);
    WRITE(GPIO_3, LOW);
    WRITE(GPIO_4, LOW);
    WRITE(GPIO_5, LOW);
    WRITE(GPIO_6, LOW);
    WRITE(GPIO_7, LOW);
    WRITE(GPIO_8, LOW);
    WRITE(GPIO_9, LOW);
}

/**
   \fn void writeGPIO(uint8_t pin, uint8_t value)
   \brief Set the GPIO pin with the value

 */
void writeGPIO(uint8_t pin, uint8_t value){
    switch(pin){
    case 1:
        if (value == 0) WRITE(GPIO_1, LOW);
        else if (value == 1) WRITE(GPIO_1, HIGH);
        break;
    case 2:
        if (value == 0) WRITE(GPIO_2, LOW);
        else if (value == 1) WRITE(GPIO_2, HIGH);
        break;
    case 3:
        if (value == 0) WRITE(GPIO_3, LOW);
        else if (value == 1) WRITE(GPIO_3, HIGH);
        break;
    case 4:
        if (value == 0) WRITE(GPIO_4, LOW);
        else if (value == 1) WRITE(GPIO_4, HIGH);
        break;
    case 5:
        if (value == 0) WRITE(GPIO_5, LOW);
        else if (value == 1) WRITE(GPIO_5, HIGH);
        break;
    case 6:
        if (value == 0) WRITE(GPIO_6, LOW);
        else if (value == 1) WRITE(GPIO_6, HIGH);
        break;
    case 7:
        if (value == 0) WRITE(GPIO_7, LOW);
        else if (value == 1) WRITE(GPIO_7, HIGH);
        break;
    case 8:
        if (value == 0) WRITE(GPIO_8, LOW);
        else if (value == 1) WRITE(GPIO_8, HIGH);
        break;
    case 9:
        if (value == 0) WRITE(GPIO_9, LOW);
        else if (value == 1) WRITE(GPIO_9, HIGH);
        break;
    default:
        break;
    }
}

#endif 

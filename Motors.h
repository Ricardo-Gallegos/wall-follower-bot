// Motors.h
// Runs on TM4C123 for CECS347 Project 2

// configure the system to get its clock from the PLL
//void Motors_Init(void);


// PWM.h
// Runs on TM4C123
// Use PWM0/PB6 and PWM1/PB7 to generate pulse-width modulated outputs.
// Daniel Valvano
// March 28, 2014

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2014
  Program 6.7, section 6.3.2

 Copyright 2014 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */
// Modified by Min He, September 7, 2021
#include <stdint.h>

#define PERIOD 10000
#define STEP 1000
#define MAX_DUTY 9000

// Constant definitions based on the following hardware interface:
// System clock is 16MHz.
#define TOTAL_PERIOD 16000 //  16MHz/1000=16000
#define START_SPEED 16000*0.5

// period is 16-bit number of PWM clock cycles in one period 
// Output on PB6/M0PWM0
void PWM1G2A_Init(uint16_t period);

// change duty cycle of PB6
// duty is number of PWM clock cycles output is high  
void PWM1G2A_Duty(uint16_t duty);

// period is 16-bit number of PWM clock cycles in one period 
// Output on PB7/M0PWM1
void PWM1G2B_Init(uint16_t period);

// change duty cycle of PB7
// duty is number of PWM clock cycles output is high  
void PWM1G2B_Duty(uint16_t duty);

//////////////////////1. Declarations Section////////////////////////////////
////////// Function Prototypes //////////
// Dependency: None
// Inputs: None
// Outputs: None
// Description: 
//	Initializes the PWM module 1 signals tied to PF321 on the Tiva Launchpad 
//		to allow for changing brightness of LEDs based on vehicle speed.
//	Initializes the PWM module 0 signals tied to PB76 to be used with the 
//		L298N motor driver allowing for a variable speed of robot car.
void PWM_PB76_Init(void);

// Dependency: PWM_PB76_Init()
// Inputs: 
//	duty_L is the value corresponding to the duty cycle of the left wheel
//	duty_R is the value corresponding to the duty cycle of the right wheel
// Outputs: None 
// Description: Changes the duty cycles of PB76 by changing the CMP registers
void PWM_PB76_Duty(unsigned long duty_L, unsigned long duty_R);
/////////////////////////////////////////////////////////////////////////////

// Initialize both Motors
void Motors_Init(void);

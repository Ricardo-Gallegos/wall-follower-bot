// Motors.c
// Runs on TM4C123 for CECS347 Project 2
#include "tm4c123gh6pm.h"
#include "Motors.h"

// The #define statement SYSDIV2 in PLL.h
// configure the system to get its clock from the PLL

// PWM.c
// Runs on TM4C123
// Use PWM0/PB6 and PWM1/PB7 to generate pulse-width modulated outputs.
// Daniel Valvano
// March 28, 2014
// Modified by Min He, September 7, 2021

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
#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "Motors.h"

/////////////////////////////////////////////////////////////////////////////
// Course Number: CECS 347
// Assignment: Example project for Hardware PWM controlled Car
// Description: 
/////////////////////////////////////////////////////////////////////////////


////////// Local Global Variables //////////
/////////////////////////////////////////////////////////////////////////////

//////////////////////3. Subroutines Section/////////////////////////////////
// Dependency: None
// Inputs: None
// Outputs: None
// Description: 
// Initializes the PWM module 0 generator 0 outputs A&B tied to PB76 to be used with the 
//		L298N motor driver allowing for a variable speed of robot car.
void PWM_PB76_Init(void){
	if ((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOB)==0) {
		SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB;	// Activate B clocks
		while ((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOB)==0){};
	}
	
	GPIO_PORTB_AFSEL_R |= 0xC0;	// enable alt funct: PB76 for PWM
  GPIO_PORTB_PCTL_R &= ~0xFF000000; // PWM to be used
  GPIO_PORTB_PCTL_R |= 0x44000000; // PWM to be used
  GPIO_PORTB_DEN_R |= 0xC0;	// enable digital I/O 
	
	// Initializes PWM settings
	SYSCTL_RCGCPWM_R |= 0x01;	// activate PWM0
	SYSCTL_RCC_R &= ~0x001E0000; // Clear any previous PWM divider values
	
	// PWM0_0 output A&B Initialization for PB76
	PWM0_0_CTL_R = 0;	// re-loading down-counting mode
	PWM0_0_GENA_R |= 0xC8;	// low on LOAD, high on CMPA down
	PWM0_0_GENB_R |= 0xC08;// low on LEOAD, high on CMPB down
	PWM0_0_LOAD_R = TOTAL_PERIOD - 1;	// cycles needed to count down to 0
  PWM0_0_CMPA_R = 0;	// count value when output rises
	PWM0_0_CMPB_R = 0;	// count value when output rises
	
	PWM0_0_CTL_R |= 0x00000001;	// Enable PWM0 Generator 0 in Countdown mode
	PWM0_ENABLE_R &= ~0x00000003;	// Disable PB76:PWM0 output 0&1 on initialization
}


// Dependency: PWM_Init()
// Inputs: 
//	duty_L is the value corresponding to the duty cycle of the left wheel
//	duty_R is the value corresponding to the duty cycle of the right wheel
// Outputs: None 
// Description: Changes the duty cycles of PB76 by changing the CMP registers
void PWM_PB76_Duty(unsigned long duty_L, unsigned long duty_R){
	PWM0_0_CMPA_R = duty_L - 1;	// PB6 count value when output rises
  PWM0_0_CMPB_R = duty_R - 1;	// PB7 count value when output rises
}
/////////////////////////////////////////////////////////////////////////////


// period is 16-bit number of PWM clock cycles in one period 
// Output on PF1/M1PWM5
void PWM1G2A_Init(uint16_t period){
  SYSCTL_RCGCPWM_R |= 0x02;             // 1) activate PWM1
  SYSCTL_RCGCGPIO_R |= 0x20;            // 2) activate port F: 100000
  while((SYSCTL_RCGCGPIO_R&0x20) == 0){};
	GPIO_PORTF_AFSEL_R |= 0x02;           // enable alt funct on PF1: 0000 0010
  GPIO_PORTF_PCTL_R &= ~0x000000F0;     // configure PF1 as PWM1
  GPIO_PORTF_PCTL_R |= 0x00000050;
  GPIO_PORTF_AMSEL_R &= ~0x02;          // disable analog functionality on PF1
  GPIO_PORTF_DEN_R |= 0x02;             // enable digital I/O on PF1
  //GPIO_PORTB_DR8R_R |= 0xC0;    // enable 8 mA drive on PB6,7
  SYSCTL_RCC_R = 0x00100000 |           // 3) use PWM divider
    (SYSCTL_RCC_R & (~0x001E0000));   //    configure for /2 divider: PWM clock: 80Mhz/2=40MHz
  PWM1_2_CTL_R = 0;                     // 4) re-loading down-counting mode
  PWM1_2_GENA_R = 0xC8;                 // low on LOAD, high on CMPA down
  // PF1 goes low on LOAD
  // PF1 goes high on CMPA down
  PWM1_2_LOAD_R = period - 1;           // 5) cycles needed to count down to 0
  PWM1_2_CMPA_R = 0;             // 6) count value when output rises
  PWM1_2_CTL_R |= 0x00000001;           // 7) start PWM1
  PWM1_ENABLE_R |= 0x00000020;          // enable PF1/M1PWM5
}
// change duty cycle of PF1
// duty is number of PWM clock cycles output is high  
void PWM1G2A_Duty(uint16_t duty){
  PWM1_2_CMPA_R = duty - 1;             // 6) count value when output rises
}
// period is 16-bit number of PWM clock cycles in one period 
// Output on PF2/M1PWM6
void PWM1G2B_Init(uint16_t period){
  volatile unsigned long delay;
  SYSCTL_RCGCPWM_R |= 0x02;             // 1) activate PWM1
  SYSCTL_RCGCGPIO_R |= 0x20;            // 2) activate port F
  delay = SYSCTL_RCGCGPIO_R;            // allow time to finish activating
  GPIO_PORTF_AFSEL_R |= 0x04;           // enable alt funct on PF2
  GPIO_PORTF_PCTL_R &= ~0x00000F00;     // configure PF2 as M1PWM6
  GPIO_PORTF_PCTL_R |= 0x00000500;
  GPIO_PORTF_AMSEL_R &= ~0x04;          // disable analog functionality on PF2
  GPIO_PORTF_DEN_R |= 0x04;             // enable digital I/O on PF1
  SYSCTL_RCC_R |= SYSCTL_RCC_USEPWMDIV; // 3) use PWM divider
  SYSCTL_RCC_R &= ~SYSCTL_RCC_PWMDIV_M; //    clear PWM divider field
  SYSCTL_RCC_R += SYSCTL_RCC_PWMDIV_2;  //    configure for /2 divider
  PWM1_3_CTL_R = 0;                     // 4) re-loading down-counting mode
  PWM1_3_GENB_R = (PWM_1_GENB_ACTCMPBD_ONE|PWM_1_GENB_ACTLOAD_ZERO); // 0xC08
  // PF2 goes low on LOAD
  // PF2 goes high on CMPB down
  PWM1_3_LOAD_R = period - 1;           // 5) cycles needed to count down to 0: LAB2: PF1->M1PWM5:PWM1_2_
  PWM1_3_CMPB_R = 0;             // 6) count value when output rises: Lab 2:5%2=1->CMPB, GENB
  PWM1_3_CTL_R |= 0x00000001;           // 7) start PWM1: odd->B, even->A
  PWM1_ENABLE_R |= 0x00000040;          // enable PF2/M1PWM6
}
// change duty cycle of PF2
// duty is number of PWM clock cycles output is high  
void PWM1G2B_Duty(uint16_t duty){
  PWM1_3_CMPB_R = duty - 1;             // 6) count value when output rises
}

void Motors_Init(void){
	PWM1G2A_Init(PERIOD);         // initialize PWM0, PB6
  PWM1G2B_Init(PERIOD);         // initialize PWM0, PB7
}


// TrackExplorer.c
// Runs on TM4C123
// This is the starter file for CECS 347 Project 2 - A Track Explorer
// This project uses hardware PWM to control two DC Motors, 
// ADC to collect analog inputs from three Sharp IR sensors.
// The three Sharp analog IR distance sensors (GP2Y0A21YK0F) are used
// to allowthe robot to navigate through a track with two walls: 
// one mounted looking directly forward to avoid a head-on collision, 
// the other two looking forward to the left and to the right to detect  
// the distances between the car and the two walls. The goal is to 
// control power to each wheel so the left and right distances to the 
// walls are equal.
// If an object is detected too close to the robot, 
// the robot should be able to avoid it.
/*
    ------------------------------------------wall---------
                      /
                     /
                    / 
                   /
         -----------
         |         |
         | Robot   | ---> direction of motion and third sensor
         |         |
         -----------
                   \
                    \
                     \
                      \
    ------------------------------------------wall---------
*/
// The original project is designed by Dr. Daniel Valvano, Jonathan Valvano
// September 12, 2013
// Modifications are made by Dr. Min He.

// PE1 connected to forward facing IR distance sensor
// PE4 connected to right IR distance sensor
// PE5 connected to left IR distance sensor

#include "tm4c123gh6pm.h"
#include "Sensors.h"
#include "Motors.h"
#include "LEDSW.h"
#include "PLL.h"
#include "stdint.h"

// basic functions defined at end of startup.s
extern void DisableInterrupts(void); // Disable interrupts
extern void EnableInterrupts(void);  // Enable interrupts
extern void WaitForInterrupt(void);  // low power mode

// You use datasheet to calculate the following ADC values
// then test your sensors to adjust the values 
#define CRASH             IR15CM// if there is less than this distance ahead of the robot, it will immediately stop
#define IR15CM            2233  // ADC output for 15cm:1.8v -> (1.8/3.3)*4095=2233 
#define IR20CM            1724  // ADC output for 20cm:1.39v -> (1.39/3.3)*4095=1724
#define IR30CM            1116  // ADC output for 30cm:0.9v -> (0.9/3.3)*4095=1116
#define IR40CM            918   // ADC output for 40cm:0.74v -> (0.74/3.3)*4095=918
#define IR80CM            496   // ADC output for 80cm:0.4v -> (0.4/3.3)*4095=496
                                // with equal power to both motors (LeftH == RightH), the robot still may not drive straight
                                // due to mechanical differences in the motors, so bias the left wheel faster or slower than
                                // the constant right wheel
#define LEFTPOWER        	0.5*PERIOD   // duty cycle of left wheel 
#define RIGHTPOWER        0.5*PERIOD    // duty cycle of left wheel 

void System_Init(void);
void steering(uint16_t ahead_dist,uint16_t right_dist, uint16_t left_dist);
uint16_t left, right, ahead;
void Delay(void);
void GPIOPortF_Handler(void);
void Stop(void);

	int main(void){
	
	uint16_t dutyA, dutyB;
	uint8_t i;

  DisableInterrupts();  // disable interrupts while initializing
  System_Init();
  EnableInterrupts();   // enable after all initialization are done
	
  // TODO: Calibrate the sensors: read at least 10 times from the sensor 
	// before the car starts to move: this will allow software to filter the sensor outputs.	
	for(int i = 0; i < 10; i++){
			ReadSensorsMedianFilter(&ahead, &right, &left);
	}

	// TODO: start with moving forward, LED green 
	LED_Init();
  Car_Dir_Init();
  PWM_PB76_Init();
  PWM_PB76_Duty(START_SPEED, START_SPEED);

	//Enable clock for PORTF
  SYSCTL_RCGCGPIO_R |= 0x20;
  // Delay for clock stabilization
  while ((SYSCTL_PRGPIO_R & 0x20) == 0) {}

  // Enable PF0 and PF4 as inputs with pull-up resistors
  GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;    // Unlock PORTF
  GPIO_PORTF_CR_R |= 0x11;              // Allow changes to PF0 and PF4
  GPIO_PORTF_PUR_R |= 0x11;             // Enable pull-up resistors on PF0 and PF4
  GPIO_PORTF_DIR_R &= ~0x11;            // Set PF0 and PF4 as inputs
  GPIO_PORTF_DEN_R |= 0x11;             // Enable digital function on PF0 and PF4

	while(1){
	//choose one of the following three software filter methods
	ReadSensorsMedianFilter(&ahead, &right, &left);
	//ReadSensorsIIRFilter(&ahead, &right, &left);
	//ReadSensorsFIRFilter(&ahead, &right, &left);
	GPIOPortF_Handler(); }
	}
	
	void System_Init(void) {
  PLL_Init();           // bus clock at 80 MHz
  Sensors_Init();        // initialize ADC to sample AIN2 (PE1), AIN9 (PE4), AIN8 (PE5)
  LEDSW_Init();         // configure onboard LEDs and push buttons
  Motors_Init(); 
	}

	void GPIOPortF_Handler(void){ 
	
		// Read the value of PF0 and PF4
		uint32_t switch_value = GPIO_PORTF_DATA_R & 0x11;
		
		if ((switch_value & 0x01) == 0) {
      // PF0 is pressed
			steering(ahead,left,right);
				} 
		
		if ((switch_value & 0x10) == 0) {
      // PF4 is pressed
			Stop();
			}				
			 }

	void Stop(void){
		PWM0_ENABLE_R &= ~0x00000003; // stop both wheels
		LED = Red;
	}

void steering(uint16_t ahead_dist,uint16_t right_dist, uint16_t left_dist){
  // Suggest the following simple control as starting point:
  // 1. If any one of the senors see obstacle <20cm, stop
  // 2. If all sensors detect no obstacle within 35cm, stop
  // 3. If left sees obstacle within 30cm, turn right
  // 4. If right sees obstacle within 30cm, turn left
  // 5. If both sensors see no obstacle within 30cm, go straight  
  // Feel free to add more controlls to fine tune your robot car.
  // Make sure to take care of both wheel movements and LED display here.
	// further away = smaller value
	
	while(1) {
	ReadSensorsMedianFilter(&ahead, &right, &left);
			// Read the value of PF0 and PF4
		uint32_t switch_value = GPIO_PORTF_DATA_R & 0x11;
			// Check the state of PF0 and PF4 and take appropriate actions
  
		if (ahead > IR20CM) // reverse if center sensor is close
		{
			PWM_PB76_Duty(7500, 7500); //8000 8000
			WHEEL_DIR = BACKWARD;
			PWM0_ENABLE_R |= 0x00000003;
			LED = Red;
		}
		
				else if (left > IR20CM) // if left sensor is close to obstacle, turn left
		{
			// Backward left turn
			LED = Red;
			WHEEL_DIR = BACKWARD;
			PWM0_ENABLE_R &= ~0x00000002; // Disable right wheel
			PWM0_ENABLE_R |= 0x00000001; // Enable left wheel
		}
				
				else if (right > IR20CM) // if right sensor is close to obstacle, turn right
		{
				// Backward right turn
			LED = Red;
			WHEEL_DIR=BACKWARD;
			PWM0_ENABLE_R |= 0x00000002; // Enable right wheel
			PWM0_ENABLE_R &= ~0x00000001; // Disable left wheel
		}
		
				else if ((ahead >= IR80CM) && (left >= IR80CM) && (right >= IR80CM))
				{
					Stop();
					LED = Blue;
				}
				else  {
				//	go straight
	    LED = Green;
			WHEEL_DIR = FORWARD;
			PWM0_ENABLE_R |= 0x00000003; // enable both wheels
		}
			}
		}	

// Subroutine to wait 0.5 sec
// Inputs: None
// Outputs: None
// Notes: ...
void Delay(void){unsigned long volatile time;
  time = (727240*100/91);  // 1 sec 
  while(time){
			
		time--;
  }
	for (time=0;time<100;time=time+10) { 
	}
}


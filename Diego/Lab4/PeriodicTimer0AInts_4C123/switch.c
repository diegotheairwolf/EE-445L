// SwitchTestMain.c
// Runs on LM4F120/TM4C123
// Test the switch initialization functions by setting the LED
// color according to the status of the switches.
// Daniel and Jonathan Valvano
// September 12, 2013

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2014
   Example 2.3, Program 2.9, Figure 2.36

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

// negative logic switches connected to PF0 and PF4 on the Launchpad
// red LED connected to PF1 on the Launchpad
// blue LED connected to PF2 on the Launchpad
// green LED connected to PF3 on the Launchpad
// NOTE: The NMI (non-maskable interrupt) is on PF0.  That means that
// the Alternate Function Select, Pull-Up Resistor, Pull-Down Resistor,
// and Digital Enable are all locked for PF0 until a value of 0x4C4F434B
// is written to the Port F GPIO Lock Register.  After Port F is
// unlocked, bit 0 of the Port F GPIO Commit Register must be set to
// allow access to PF0's control registers.  On the LM4F120, the other
// bits of the Port F GPIO Commit Register are hard-wired to 1, meaning
// that the rest of Port F can always be freely re-configured at any
// time.  Requiring this procedure makes it unlikely to accidentally
// re-configure the JTAG pins as GPIO, which can lock the debugger out
// of the processor and make it permanently unable to be debugged or
// re-programmed.
#include <stdint.h>
#include "inc/tm4c123gh6pm.h"


#define PF0       (*((volatile uint32_t *)0x40025004))
#define PF1       (*((volatile uint32_t *)0x40025008))
#define PF2       (*((volatile uint32_t *)0x40025010))
#define PF3       (*((volatile uint32_t *)0x40025020))
#define PF4       (*((volatile uint32_t *)0x40025040))

void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
unsigned long button;
long sr;

//int main(void){ 
//	unsigned long in,out;
//	PortD_Init();
//	EdgeCounter_Init();
//	while(1){
//		
//		//in = (GPIO_PORTD_DATA_R&0x01); // in 0 if not pressed, 1 if pressed
//    //out = (in^0x01)<<3;   // out 8 if not pressed, 0 if switch pressed
//    //GPIO_PORTD_DATA_R = out;
//		
//		WaitForInterrupt();
//		
//		if(button == 0x02){
//			GPIO_PORTD_DATA_R = button<<2;
//		}
//		else if(button == 0x04){
//			GPIO_PORTD_DATA_R = button>>1;
//		}
//		else if(button == 0x08){
//			GPIO_PORTD_DATA_R = button^0x08>>2;
//		}
//		else if(button == 0x10){
//			GPIO_PORTD_DATA_R = button^0x10>>3;
//		}
//		
//	}
//}

void PortD_Init(void){
	unsigned long volatile delay;
  SYSCTL_RCGC2_R |= 0x08;           // Port D clock
  delay = SYSCTL_RCGC2_R;           // wait 3-5 bus cycles
  GPIO_PORTD_DIR_R |= 0x08;         // PD3 output
  GPIO_PORTD_AFSEL_R &= ~0x08;      // not alternative
  GPIO_PORTD_AMSEL_R &= ~0x08;      // no analog
  GPIO_PORTD_PCTL_R &= ~0x0000F000; // bits for PD3
  GPIO_PORTD_DEN_R |= 0x08;         // enable PD3
}


void EdgeCounter_Init(void){       
  SYSCTL_RCGC2_R |= 0x00000020; // (a) activate clock for port F
  GPIO_PORTF_DIR_R &= ~0x1E;    // (c) make PF4 in (built-in button)
  GPIO_PORTF_AFSEL_R &= ~0x1E;  //     disable alt funct on PF4
  GPIO_PORTF_DEN_R |= 0x1E;     //     enable digital I/O on PF4
  GPIO_PORTF_PCTL_R &= ~0x000FFFF0; //  configure PF4 as GPIO
  GPIO_PORTF_AMSEL_R &= ~0x1E;  //    disable analog functionality on PF4
  GPIO_PORTF_PUR_R |= 0x00;     //     enable weak pull-up on PF4
  GPIO_PORTF_IS_R &= ~0x1E;     // (d) PF4 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x1E;    //     PF4 is not both edges
  GPIO_PORTF_IEV_R |= 0x1E;    //     PF4 RISING edge event
  GPIO_PORTF_ICR_R = 0x1E;      // (e) clear flag4
  GPIO_PORTF_IM_R |= 0x1E;      // (f) arm interrupt on PF4
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000; // (g) priority 5
  NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC
//  EnableInterrupts();           // (i) Enable global Interrupt flag (I)
}

void GPIOPortF_Handler(void){
  GPIO_PORTF_ICR_R = 0x1E;      // acknowledge flag4-1
	sr = StartCritical();
	button = GPIO_PORTF_DATA_R;
	EndCritical(sr);
}

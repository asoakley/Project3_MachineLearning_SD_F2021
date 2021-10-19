/*
 * timers.c
 *
 *  Created on: Oct 16, 2021
 *      Author: austi
 */
#include "msp.h"
#include <timers.h>
#include <gpio.h>

// Globals

// bruh


// Initialize Timers
void Init_Timers(void){
    Init_TimerA2(TIMERA2_PERIOD);
}

void Init_TimerA2(uint16_t period){
   TIMER_A2->CTL = 0x0280;      // Set clock source to SMCLK (3 MHZ by default), input clock divider / 4 (0.75 MHZ) (see user guide for details)
   TIMER_A2->CCTL[0] = 0x0010;   // Capture mode, compare mode, enable CC interrupt, clear CCIFG
   TIMER_A2->CCR[0] = period - 1;   // Set capture compare register value
   TIMER_A2->EX0 = 0x0005;      // Input clock divider, divide by 6 (125 kHz)
   NVIC->IP[3] = (NVIC->IP[3] & 0xFFFFFF00)|0x00000040;     // NVIC priority 2
   NVIC->ISER[0] = 0x00001000;  // enable interrupt 12 in the NVIC
   TIMER_A2->CTL |= 0x0014;     // reset and start Timer A in up mode

}




// Timer A2

void TA2_0_IRQHandler(void) {
    TIMER_A2->CCTL[0] &= ~BIT0; // Clear flag
    P2OUT ^= (RGB_RED | RGB_BLUE | RGB_GREEN);

}


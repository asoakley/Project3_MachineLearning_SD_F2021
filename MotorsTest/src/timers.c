/*
 * timers.c
 *
 *  Created on: Oct 16, 2021
 *      Author: austi
 */
#include "msp.h"
#include <timers.h>
#include <motors.h>
#include <gpio.h>

// Globals

// bruh


// Initialize Timers
void Init_Timers(void){
    Init_TimerA2(TIMERA2_PERIOD);
    Init_TimerA0(MOTOR_PWM_PERIOD, LEFT_FORWARD_SLOW, RIGHT_FORWARD_SLOW);  // Set the initial period and duty cycle of the motors
}


void Init_TimerA0(uint16_t period, uint16_t duty_left, uint16_t duty_right)
{
    TIMER_A0->CCTL[0] = 0x0080; // CCI0 toggle
    TIMER_A0->CCR[0] = period; // Period is 2*period*8*83.33ns is 1.333*period
    TIMER_A0->EX0 = 0x0000; // divide by 1
    TIMER_A0->CCTL[3] = 0x0040; // CCR1 toggle/reset
    TIMER_A0->CCR[3] = duty_right; // CCR1 duty cycle is duty1/period
    TIMER_A0->CCTL[4] = 0x0040; // CCR2 toggle/reset
    TIMER_A0->CCR[4] = duty_left; // CCR2 duty cycle is duty2/period
    TIMER_A0->CTL = 0x02B0; // SMCLK=3MHz, divide by 4, up-down mode
}

void Init_TimerA2(uint16_t period){
   TIMER_A2->CTL = 0x0280;      // Set clock source to SMCLK (3 MHZ by default), input clock divider / 4 (0.75 MHZ) (see user guide for details)
   TIMER_A2->CCTL[0] = 0x0010;   // Capture mode, compare mode, enable CC interrupt, clear CCIFG
   TIMER_A2->CCR[0] = period - 1;   // Set capture compare register value
   TIMER_A2->EX0 = 0x0005;      // Input clock divider, divide by 6 (125 kHz)
   //Configure Interrupts
   NVIC_SetPriority(TA2_0_IRQn, 2); //sets interrupt priority for CCTL[0]
       //enable interrupts
   NVIC_EnableIRQ(TA2_0_IRQn);
   TIMER_A2->CTL |= 0x0014;     // reset and start Timer A in up mode

}




// Timer A2

void TA2_0_IRQHandler(void) {
    TIMER_A2->CCTL[0] &= ~BIT0; // Clear flag
    P2OUT ^= (RGB_RED | RGB_BLUE | RGB_GREEN);

}


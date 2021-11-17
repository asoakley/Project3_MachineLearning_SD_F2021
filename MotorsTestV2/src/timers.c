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

//=================
// Globals
//=================
    extern volatile char Motors_Ready;
    extern volatile uint16_t TimerTicks = 0;



//==========================
// Timer Initialization
//==========================

void Init_Timers(void){
    Init_TimerA2(TIMERA2_HALF);
    Init_TimerA0(MOTOR_PWM_PERIOD, LEFT_OFF, RIGHT_OFF);  // Set the initial period and duty cycle of the motors
    Init_SysTickTimer();
}


// PWM for Motors

void Init_TimerA0(uint16_t period, uint16_t duty_left, uint16_t duty_right)
{
    TIMER_A0->CCTL[0] = 0x0080; // CCI0 toggle
    TIMER_A0->CCR[0] = period; // Period is (8 / SMCLK)
    TIMER_A0->EX0 = 0x0000; // divide by 1
    TIMER_A0->CCTL[3] = 0x0040; // CCR3 toggle/reset
    TIMER_A0->CCR[3] = duty_right; // CCR3 duty cycle is duty_right/period
    TIMER_A0->CCTL[4] = 0x0040; // CCR4 toggle/reset
    TIMER_A0->CCR[4] = duty_left; // CCR4 duty cycle is duty_left/period
    TIMER_A0->CTL = 0x02F0; // SMCLK=12MHz, divide by 8, up-down mode
}


// LED Blink , Timed Motor Movement, Switch Debounce

void Init_TimerA2(uint16_t period){
    TIMER_A2->CTL = 0x0100;      // Set clock source to ACLK (32.768 KHz), input clock divider / 1

    TIMER_A2->CCR[0] = period;   // Set capture compare register value
    TIMER_A2->CCTL[0] |= CCIE;   // Capture mode, compare mode, enable CC interrupt
    TIMER_A2->CCTL[0] &= ~CCIFG; // Clear interrupt flag

    //TIMER_A2->CCTL[1] |= CCIE;   // Capture mode, compare mode, enable CC interrupt
    TIMER_A2->CCTL[1] &= ~CCIFG; // Clear interrupt flag

    //TIMER_A2->CCTL[2] |= CCIE;   // Capture mode, compare mode, enable CC interrupt
    TIMER_A2->CCTL[2] &= ~CCIFG; // Clear interrupt flag


    TIMER_A2->EX0 = 0x0000;      // Input clock divider, divide by 1
   //Configure Interrupts
    NVIC_SetPriority(TA2_0_IRQn, 2); //sets interrupt priority for CCTL[0]

    NVIC_SetPriority(TA2_N_IRQn, 2); //sets interrupt priority for CCTL[1-6]


    TIMER_A2->CTL |= 0x0014;     // reset and start Timer A in up mode

}

void Init_SysTickTimer(void){
    SysTick->LOAD = 0x00B71B00;      // .25 sec
    SysTick->CTRL = 0x00000007;     // System clock (48MHz) , interrupt enabled, timer enabled
    NVIC_SetPriority(SysTick_IRQn, 2);
}

//==================================================
// SysTick Timer Interrupt for Timed Motor Movement
//==================================================
void SysTick_Handler(void){     // Count flag automatically cleared
    if(TimerTicks > 0){
        TimerTicks--;
    }
    else {
        P1OUT ^= RED_LED;
        Motors_Ready = 1;
        MotorsSimple(LEFT_OFF, FORWARD, RIGHT_OFF, FORWARD);
        SysTick->CTRL = 0x00000006; // Disable Timer
    }
}

void SysTickCount_250ms(uint16_t ticks){
    TimerTicks = ticks;
    SysTick->LOAD = 0x00B71B00;      // .25 sec
    SysTick->CTRL = 0x00000007;     // System clock (48MHz) , interrupt enabled, timer enabled
}

// Timer A2 Interrupt

void TA2_0_IRQHandler(void) {
    TIMER_A2->CCTL[0] &= ~CCIFG; // Clear flag
    P2OUT ^= (RGB_RED | RGB_BLUE | RGB_GREEN);

}

void TA2_N_IRQHandler(void){
    switch(TIMER_A2->IV){
        case 0x00:
        break;

    // Capture Compare 1 For SW1
        case 0x02:
            TIMER_A2->CCTL[1] &= ~CCIFG;
            P1IFG &= ~SW1;
            P1IE |= SW1;
            P1OUT &= ~RED_LED;
            TIMER_A2->CCTL[1] &= ~CCIE;   // Disable Interrupt when counter expires
        break;

    // Capture Compare 2 for SW2 debounce
        case 0x04:
            TIMER_A2->CCTL[2] &= ~CCIFG;
            P1IFG &= ~SW2;
            P1IE |= SW2;
            P1OUT &= ~RED_LED;
            TIMER_A2->CCTL[2] &= ~CCIE;   // Disable Interrupt when counter expires
        break;

        // Overflow
        case 0x0E:
            break;

        default: break;
    }

}



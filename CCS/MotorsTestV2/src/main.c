#include "msp.h"
#include <gpio.h>
#include <timers.h>
#include <motors.h>
#include <Clock.h>


/**
 * main.c
 */

// Globals


void EnableInterrupts(void){
    NVIC_EnableIRQ(PORT1_IRQn);
    NVIC_EnableIRQ(PORT4_IRQn);
    NVIC_EnableIRQ(TA2_0_IRQn);
    NVIC_EnableIRQ(TA2_N_IRQn);
    NVIC_EnableIRQ(SysTick_IRQn);
}

void main(void)
  {
        Clock_Init48MHz();  // System clock at 48MHz, SMCLK at 12MHz
        Init_Ports();
        Init_Timers();
        MotorsSimple(LEFT_OFF, FORWARD, RIGHT_OFF, FORWARD);   // Initialize motors OFF

        EnableInterrupts();

        while(1){
            TriangleFSM();
            Figure8FSM();
        }

        //SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;        // Put in low power mode with interrupts enabled
}

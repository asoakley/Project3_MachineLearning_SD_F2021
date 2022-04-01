#include "msp.h"
#include <gpio.h>
#include <timers.h>
#include <motors.h>


/**
 * main.c
 */

// Globals



void main(void)
{
        Clock_Init48MHz();  // System clock at 48MHz, SMCLK at 12MHz
        Init_Ports();
        Init_Timers();
        MotorsSimple(LEFT_OFF, FORWARD, RIGHT_OFF, FORWARD);

        SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;        // Put in low power mode with interrupts enabled
}

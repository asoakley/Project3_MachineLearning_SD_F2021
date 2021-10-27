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
        Init_Ports();
        Init_Timers();
        MotorsSimple(LEFT_OFF, FORWARD, RIGHT_OFF, FORWARD);   // Initialize motors OFF

        SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;        // Put in low power mode with interrupts enabled
}

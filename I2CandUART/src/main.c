#include "msp.h"
#include <gpio.h>
#include <timers.h>
#include <motors.h>
#include <IMU.h>


/**
 * main.c
 */

// Globals


void main(void)
{
        Clock_Init48MHz();  // System clock at 48MHz, SMCLK at 12MHz
        Init_Ports();
        Init_Timers();
        Init_IMU();
        MotorsSimple(LEFT_OFF, FORWARD, RIGHT_OFF, FORWARD);   // Initialize motors OFF

        while(1){

        }

        //SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;        // Put in low power mode with interrupts enabled
}

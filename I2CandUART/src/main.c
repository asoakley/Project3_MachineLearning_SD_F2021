#include "msp.h"
#include <gpio.h>
#include <timers.h>
#include <motors.h>
#include <IMU.h>
#include <PC_comm.h>
#include <Clock.h>


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
        Init_PC_UART();
        MotorsSimple(LEFT_OFF, FORWARD, RIGHT_OFF, FORWARD);   // Initialize motors OFF

        while(1){
            Print_Accel();
        }

        //SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;        // Put in low power mode with interrupts enabled
}

#include "msp.h"
#include <gpio.h>
#include <timers.h>
#include <motors.h>


/**
 * MotorsTest
 * main.c
 */

// Globals



void main(void)
{
        Init_Ports();
        Init_Timers();
        MotorsSimple(LEFT_OFF, FORWARD, RIGHT_OFF, FORWARD);

        while(1){

            ;
        }
}

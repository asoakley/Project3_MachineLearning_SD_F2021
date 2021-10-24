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
        MotorsSimple(LEFT_SLOW, REVERSE, RIGHT_SLOW, REVERSE);

        while(1){

            ;
        }
}

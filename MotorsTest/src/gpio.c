#include <gpio.h>
#include "msp.h"

void Init_Ports(){
    Init_Port1();
    Init_Port2();
    Init_Port3();
//    Init_Port4();
  //  Init_Port5();
    //Init_Port6();
}

void Init_Port1(void){
    // P1.0 RED LED

    P1SEL0 &= ~RED_LED;
    P1SEL1 &= ~RED_LED;
    P1DIR |= RED_LED;
    P1OUT &= ~RED_LED;

    // P1.1 and P1.4 Switches

    P1SEL0 &= ~(SW1 | SW2); // SW2 Operation
    P1SEL1 &= ~(SW1 | SW2); // SW2 Operation
    P1OUT |= (SW1 | SW2); // Configure pullup resistor
    P1DIR &= ~(SW1 | SW2); // Direction = input
    P1REN |= (SW1 | SW2); // Enable pullup resistor
    P1IES |= (SW1 | SW2); // P1.3 Hi/Lo edge interrupt
    P1IFG &= ~(SW1 | SW2); // Clear all P1.3 interrupt flags
    P1IE |= (SW1 | SW2); // P1.3 interrupt enabled
    NVIC_SetPriority(PORT1_IRQn, 2); //sets interrupt priority for CCTL[0]
        //enable interrupts
    NVIC_EnableIRQ(PORT1_IRQn);

    // P1.2


    // P1.3


    // P1.5

    // P1.6

    // P1.7

}

void Init_Port2(void){
    // P2.0, P2.1, P2.2 RGB LED

    P2SEL0 &= ~(RGB_RED | RGB_BLUE | RGB_GREEN);
    P2SEL1 &= ~(RGB_RED | RGB_BLUE | RGB_GREEN);
    P2DIR |= (RGB_RED | RGB_BLUE | RGB_GREEN);
    P2OUT &= ~(RGB_RED | RGB_BLUE | RGB_GREEN);

    // P2.3

    // P2.4

    // P2.5

    // P2.6, P2.7 Right/Left PWM

    P2SEL0 |= (PWM_RIGHT | PWM_LEFT);
    P2SEL1 &= ~(PWM_RIGHT | PWM_LEFT);
    P2DIR |= (PWM_RIGHT | PWM_LEFT);

}

void Init_Port3(void){
    // P3.0

    // P3.1

    // P3.2

    // P3.3

    // P3.4

    // P3.5

    // P3.6

    // P3.7

}

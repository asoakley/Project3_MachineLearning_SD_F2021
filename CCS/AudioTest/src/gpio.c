#include <gpio.h>
#include "msp.h"

void Init_Ports(){
    Init_Port1();
    Init_Port2();
    Init_Port3();
    Init_Port4();
    Init_Port5();
    //Init_Port6();
}

void Init_Port1(void){
    // P1.0 RED LED

    P1SEL0 &= ~RED_LED;     // GPIO Function
    P1SEL1 &= ~RED_LED;     // GPIO Function
    P1DIR |= RED_LED;       // Output
    P1OUT &= ~RED_LED;      // Turn LED OFF

    // P1.1 and P1.4 Switches

    P1SEL0 &= ~(SW1 | SW2); // SW2 Operation
    P1SEL1 &= ~(SW1 | SW2); // SW2 Operation
    P1OUT |= (SW1 | SW2); // Configure pullup resistor
    P1DIR &= ~(SW1 | SW2); // Direction = input
    P1REN |= (SW1 | SW2); // Enable pullup resistor
    P1IES |= (SW1 | SW2); // P1.1 and P1.4 Hi/Lo edge interrupt
    P1IFG &= ~(SW1 | SW2); // Clear all P1.1 and P1.4 interrupt flags
    P1IE |= (SW1 | SW2); // P1.1 and P1.4 interrupt enabled

    NVIC_SetPriority(PORT1_IRQn, 1);    // set interrupt priority, lower number is higher priority
    NVIC_EnableIRQ(PORT1_IRQn);         // enable interrupt

    // P1.2


    // P1.3


    // P1.5

    // P1.6

    // P1.7

}

void Init_Port2(void){
    // P2.0, P2.1, P2.2 RGB LED

    P2SEL0 &= ~(RGB_RED | RGB_BLUE | RGB_GREEN);        // GPIO function
    P2SEL1 &= ~(RGB_RED | RGB_BLUE | RGB_GREEN);        // GPIO function
    P2DIR |= (RGB_RED | RGB_BLUE | RGB_GREEN);          // Output
    P2OUT &= ~(RGB_RED | RGB_BLUE | RGB_GREEN);         // Turn LEDs OFF

    // P2.3

    // P2.4

    // P2.5

    // P2.6, P2.7 Right/Left PWM

    P2SEL0 |= (PWM_RIGHT | PWM_LEFT);       // PWM function (Timer A0)
    P2SEL1 &= ~(PWM_RIGHT | PWM_LEFT);      // PWM function (Timer A0)
    P2DIR |= (PWM_RIGHT | PWM_LEFT);        // Output

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

void Init_Port4(void){
    // P4.0, P4.2, P4.3, P4.5, P4.6, P4.7: Bump Switches

    P4SEL0 &= ~(BUMP0 | BUMP1 | BUMP2 | BUMP3 | BUMP4 | BUMP5);     // GPIO Function
    P4SEL1 &= ~(BUMP0 | BUMP1 | BUMP2 | BUMP3 | BUMP4 | BUMP5);     // GPIO Function
    P4OUT |= (BUMP0 | BUMP1 | BUMP2 | BUMP3 | BUMP4 | BUMP5);       // Configure pullup resistor
    P4DIR &= ~(BUMP0 | BUMP1 | BUMP2 | BUMP3 | BUMP4 | BUMP5);      // Input
    P4REN |= (BUMP0 | BUMP1 | BUMP2 | BUMP3 | BUMP4 | BUMP5);       // Enable pullup resistor
    P4IES |= (BUMP0 | BUMP1 | BUMP2 | BUMP3 | BUMP4 | BUMP5);       // P4 Hi/Lo edge interrupt
    P4IFG &= ~(BUMP0 | BUMP1 | BUMP2 | BUMP3 | BUMP4 | BUMP5);      // Clear all P4 interrupt flags
    P4IE |= (BUMP0 | BUMP1 | BUMP2 | BUMP3 | BUMP4 | BUMP5);

    NVIC_SetPriority(PORT4_IRQn, 1);    // sets interrupt priority
    NVIC_EnableIRQ(PORT4_IRQn);         // enable interrupts

    // P4.1

    // P4.4



}

void Init_Port5(void){
    // P5.0

    // P5.1

    // P5.2

    // P5.3

    // P5.4 and P5.5: Left and Right Motor Direction

    P5SEL0 &= ~(LDIR | RDIR);       // GPIO function
    P5SEL1 &= ~(LDIR | RDIR);       // GPIO function
    P5DIR |= (LDIR | RDIR);         // Output
    P5OUT &= ~(LDIR | RDIR);        // Start in forward direction

    // P5.6

    // P5.7

}

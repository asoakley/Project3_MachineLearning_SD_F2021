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
    // P1.0 , P1.1 , P1.2
    // RGB LED

    P2SEL0 &= ~(RGB_RED | RGB_BLUE | RGB_GREEN);
    P2SEL1 &= ~(RGB_RED | RGB_BLUE | RGB_GREEN);
    P2DIR |= (RGB_RED | RGB_BLUE | RGB_GREEN);
    P2OUT &= ~(RGB_RED | RGB_BLUE | RGB_GREEN);


    // P1.3

    // P1.4

    // P1.5

    // P1.6

    // P1.7

}

void Init_Port2(void){
    // P2.0

    // P2.1

    // P2.2

    // P2.3

    // P2.4

    // P2.5

    // P2.6

    // P2.7

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

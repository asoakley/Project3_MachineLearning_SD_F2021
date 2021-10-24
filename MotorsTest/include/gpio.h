/*
 * ports.h
 *
 *  Created on: Oct 7, 2021
 *      Author: austi
 */

#include "msp.h"

#ifndef INCLUDE_GPIO_H_
#define INCLUDE_GPIO_H_

//-----------Macros-----------

//Port 1
#define RED_LED         BIT0
#define SW1             BIT1
#define SW2             BIT4

//Port 2
#define RGB_RED         BIT0
#define RGB_BLUE        BIT1
#define RGB_GREEN       BIT2
#define PWM_RIGHT       BIT6
#define PWM_LEFT        BIT7


//Port 3



//Port 4

//Port 5

#define LDIR            BIT4
#define RDIR            BIT5


//Port 7




//Port 8
#define YELLOW_LED_R    BIT5
#define YELLOW_LED_L    BIT0
#define RED_LED_R       BIT7
#define RED_LED_L       BIT6

//Function Declarations

void Init_Ports(void);
void Init_Port1(void);
void Init_Port2(void);
void Init_Port3(void);
void Init_Port4(void);
void Init_Port5(void);
void Init_Port6(void);



#endif /* INCLUDE_GPIO_H_ */

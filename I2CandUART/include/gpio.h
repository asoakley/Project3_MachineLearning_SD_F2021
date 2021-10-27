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

#define BUMP0           BIT0

#define BUMP1           BIT2
#define BUMP2           BIT3

#define BUMP3           BIT5
#define BUMP4           BIT6
#define BUMP5           BIT7

#define BUMPS_ALL       (BUMP0 | BUMP1 | BUMP2 | BUMP3 | BUMP4 | BUMP5)

//Port 5

#define LDIR            BIT4
#define RDIR            BIT5

//Port 6

#define UCB1SDA         BIT4
#define UCB1SCL         BIT5

//Port 7




//Port 8
#define YELLOW_LED_L    BIT0




#define YELLOW_LED_R    BIT5
#define RED_LED_L       BIT6
#define RED_LED_R       BIT7


//Function Declarations

void Init_Ports(void);
void Init_Port1(void);
void Init_Port2(void);
void Init_Port3(void);
void Init_Port4(void);
void Init_Port5(void);
void Init_Port6(void);



#endif /* INCLUDE_GPIO_H_ */

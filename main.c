#include "msp.h"


/*
 Step 1:
    1) Connect motor to MSP board
    2) Control motor via SW1 press
    3) Connect 2 motors and control

 
 */

/********************
 * Car main.c code
 ********************/
void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer
}

//chimichanga==pie

/********************
 * Remote main.c code
 ********************/
//void main(void)
//{
//    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer
//}

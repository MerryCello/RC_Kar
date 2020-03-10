#include "msp.h"


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

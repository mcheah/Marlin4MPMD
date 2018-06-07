/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/
//Includes
#include "main.h"
#include "Marlin_export.h"

/* Private variables ---------------------------------------------------------*/

int main(void)
{
    BSP_LED_Init(LED_GREEN);
    BSP_LED_Init(LED_RED);
    BSP_LED_Init(LED_BLUE);
	setup();
	for(;;)
	{
		loop();
	}
}

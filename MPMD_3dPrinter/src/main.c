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

int main(void)
{
	setup();
	for(;;)
	{
		loop();
	}
}

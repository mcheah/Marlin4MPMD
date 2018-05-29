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

#ifdef __cplusplus
extern "C" {
#endif
#include <stopwatch2.h>
#ifdef __cplusplus
}
#endif

int main(void)
{
	setup();
	StopWatch_Init();
	StopWatch_Start();
	for(;;)
	{
		loop();
	}
}

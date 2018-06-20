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
#include "ff_gen_drv.h"
#include "sd_diskio.h"
//Private defines
#ifdef STM32_USE_BOOTLOADER
#define APPLICATION_ADDRESS     (uint32_t)0x08002000
//#define APPLICATION_ADDRESS		  (uint32_t)0x08000000
//Private variables
__IO uint32_t VectorTable[48] __attribute__((section(".RAMVectorTable")));

static inline void remapVectorTable(void)
{
	for(uint8_t i = 0; i < 48; i++)
	{
	    VectorTable[i] = *(__IO uint32_t*)(APPLICATION_ADDRESS + (i<<2));
	}
	  /* Enable the SYSCFG peripheral clock*/
	__HAL_RCC_APB2_FORCE_RESET();
	  /* Remap SRAM at 0x00000000 */
	__HAL_SYSCFG_REMAPMEMORY_SRAM();
	RCC->AHBENR = 1<<2 |1<<4 | 1<<18;
	 if((WWDG->CR == 0xC0) && (WWDG->CR & 0x80))
		 WWDG->CR = 0x000000ff;
//			 HAL_WWDG_Refresh(&WwdgHandle);
	 if(IWDG->RLR)
		 IWDG->RLR = 0x0000AAAAU;
	 __HAL_RCC_APB2_RELEASE_RESET();
}
#endif

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
FATFS SDFatFs;  /* File system object for SD card logical drive */
FIL MyFile;     /* File object */
char SDPath[4]; /* SD card logical drive path */

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void testFS(void)
{
	FRESULT res;                                          /* FatFs function common result code */
	uint32_t byteswritten, bytesread;                     /* File write/read counts */
	uint8_t wtext[] = "TestingTestingTestingTesting"; /* File write buffer */
	uint8_t rtext[100];                                   /* File read buffer */
	  /*##-1- Link the micro SD disk I/O driver ##################################*/
	  if(FATFS_LinkDriver(&SD_Driver, SDPath) == 0)
	  {
	    /*##-2- Register the file system object to the FatFs module ##############*/
	    if(f_mount(&SDFatFs, (TCHAR const*)SDPath, 0) != FR_OK)
	    {
	      /* FatFs Initialization Error */
	    	BSP_MiscErrorHandler(0x6000 | 1);
	    }
	    else
	    {
	      /*##-3- Create a FAT file system (format) on the logical drive #########*/
	      /* WARNING: Formatting the uSD card will delete all content on the device */
	      if(/*f_mkfs((TCHAR const*)SDPath, 0, 0) != FR_OK*/0)
	      {
	        /* FatFs Format Error */
		    	BSP_MiscErrorHandler(0x6000 | 2);
	      }
	      else
	      {
	        /*##-4- Create and Open a new text file object with write access #####*/
	        if(f_open(&MyFile, "STM32B.TXT", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
	        {
	          /* 'STM32.TXT' file Open for write Error */
		    	BSP_MiscErrorHandler(0x6000 | 3);
	        }
	        else
	        {
	          /*##-5- Write data to the text file ################################*/
	          res = f_write(&MyFile, wtext, sizeof(wtext), (void *)&byteswritten);

	          /*##-6- Close the open text file #################################*/
	          if (f_close(&MyFile) != FR_OK )
	          {
	  	    	BSP_MiscErrorHandler(0x6000 | 4);
	          }

	          if((byteswritten == 0) || (res != FR_OK))
	          {
	            /* 'STM32.TXT' file Write or EOF Error */
	  	    	BSP_MiscErrorHandler(0x6000 | 5);
	          }
	          else
	          {
	            /*##-7- Open the text file object with read access ###############*/
	            if(f_open(&MyFile, "STM32B.TXT", FA_READ) != FR_OK)
	            {
	              /* 'STM32.TXT' file Open for read Error */
	    	    	BSP_MiscErrorHandler(0x6000 | 6);
	            }
	            else
	            {
	              /*##-8- Read data from the text file ###########################*/
	              res = f_read(&MyFile, rtext, sizeof(rtext), (UINT*)&bytesread);

	              if((bytesread == 0) || (res != FR_OK))
	              {
	                /* 'STM32.TXT' file Read or EOF Error */
	      	    	BSP_MiscErrorHandler(0x6000 | 7);
	              }
	              else
	              {
	                /*##-9- Close the open text file #############################*/
	                f_close(&MyFile);

	                /*##-10- Compare read data with the expected data ############*/
	                if((bytesread != byteswritten))
	                {
	                  /* Read data is different from the expected data */
	        	    	BSP_MiscErrorHandler(0x6000 | 8);
	                }
	                else
	                {
	                  /* Success of the demo: no error occurrence */
	                  BSP_LED_On(LED1);
	                }
	              }
	            }
	          }
	        }
	      }
	    }
	  }

	  /*##-11- Unlink the RAM disk I/O driver ####################################*/
	  FATFS_UnLinkDriver(SDPath);
	  BSP_LED_Off(LED3);
	  /* Infinite loop */
	  while (1)
	  {
		  BSP_LED_Toggle(LED3);
		  HAL_Delay(1000);
	  }
}

int main(void)
{
#ifdef STM32_USE_BOOTLOADER
	remapVectorTable();
#endif
	HAL_Init();
	SystemClock_Config();

	BSP_LED_Init(LED_GREEN);
    BSP_LED_Init(LED_RED);
    BSP_LED_Init(LED_BLUE);
    BSP_LED_On(LED_RED);
    testFS();
//	setup();

	for(;;)
	{
//		loop();
	}
}

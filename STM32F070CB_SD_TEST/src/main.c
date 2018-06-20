/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f0xx_hal.h"
#include "stm32f0xx_mpmd.h"
#include "ff_gen_drv.h"
#include "sd_diskio.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
FATFS SDFatFs;  /* File system object for SD card logical drive */
FIL MyFile;     /* File object */
char SDPath[4]; /* SD card logical drive path */

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
/* Private functions ---------------------------------------------------------*/

int main(void)
{
	  FRESULT res;                                          /* FatFs function common result code */
	  uint32_t byteswritten, bytesread;                     /* File write/read counts */
	  uint8_t wtext[] = "This is mickey's test file Triumph, BASEDGODBITCH"; /* File write buffer */
	  uint8_t rtext[100];                                   /* File read buffer */

	  /* STM32F0xx HAL library initialization:
	       - Configure the Flash prefetch
	       - Systick timer is configured by default as source of time base, but user
	         can eventually implement his proper time base source (a general purpose
	         timer for example or other time source), keeping in mind that Time base
	         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
	         handled in milliseconds basis.
	       - Low Level Initialization
	     */
	  HAL_Init();

	  /* Configure LED1 and LED3 */
	  BSP_LED_Init(LED1);
	  BSP_LED_Init(LED2);
	  BSP_LED_Init(LED3);

	  /* Configure the system clock to 48 MHz */
	  SystemClock_Config();
//	  SD_IO_Init();
//	  while(1)
//	  {
//		  HAL_GPIO_TogglePin(NUCLEO_SPIx_SCK_GPIO_PORT,NUCLEO_SPIx_SCK_PIN);
//		  delay_basic(0.1);
////		  delay_basic(0.001);
////		  for(uint32_t i=0; i<48e3/224;i++) { }
////		  BSP_LED_Toggle(LED3);
////		  HAL_Delay(200);
//	  }
	  /*##-1- Link the micro SD disk I/O driver ##################################*/
	  if(FATFS_LinkDriver(&SD_Driver, SDPath) == 0)
	  {
	    /*##-2- Register the file system object to the FatFs module ##############*/
	    if(f_mount(&SDFatFs, (TCHAR const*)SDPath, 0) != FR_OK)
	    {
	      /* FatFs Initialization Error */
	      Error_Handler();
	    }
	    else
	    {
	      /*##-3- Create a FAT file system (format) on the logical drive #########*/
	      /* WARNING: Formatting the uSD card will delete all content on the device */
	      if(/*f_mkfs((TCHAR const*)SDPath, 0, 0) != FR_OK*/0)
	      {
	        /* FatFs Format Error */
	        Error_Handler();
	      }
	      else
	      {
	        /*##-4- Create and Open a new text file object with write access #####*/
	        if(f_open(&MyFile, "STM32BASED.TXT", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
	        {
	          /* 'STM32.TXT' file Open for write Error */
	          Error_Handler();
	        }
	        else
	        {
	          /*##-5- Write data to the text file ################################*/
	          res = f_write(&MyFile, wtext, sizeof(wtext), (void *)&byteswritten);

	          /*##-6- Close the open text file #################################*/
	          if (f_close(&MyFile) != FR_OK )
	          {
	            Error_Handler();
	          }

	          if((byteswritten == 0) || (res != FR_OK))
	          {
	            /* 'STM32.TXT' file Write or EOF Error */
	            Error_Handler();
	          }
	          else
	          {
	            /*##-7- Open the text file object with read access ###############*/
	            if(f_open(&MyFile, "STM32BASED.TXT", FA_READ) != FR_OK)
	            {
	              /* 'STM32.TXT' file Open for read Error */
	              Error_Handler();
	            }
	            else
	            {
	              /*##-8- Read data from the text file ###########################*/
	              res = f_read(&MyFile, rtext, sizeof(rtext), (UINT*)&bytesread);

	              if((bytesread == 0) || (res != FR_OK))
	              {
	                /* 'STM32.TXT' file Read or EOF Error */
	                Error_Handler();
	              }
	              else
	              {
	                /*##-9- Close the open text file #############################*/
	                f_close(&MyFile);

	                /*##-10- Compare read data with the expected data ############*/
	                if((bytesread != byteswritten))
	                {
	                  /* Read data is different from the expected data */
	                  Error_Handler();
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

void SystemClock_Config(void)
{
	  RCC_ClkInitTypeDef RCC_ClkInitStruct;
	  RCC_OscInitTypeDef RCC_OscInitStruct;

	  /* No HSE Oscillator on Nucleo, Activate PLL with HSI as source */
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
	  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
	  //TODO: verify that MPMD has a 8Mhz crystal
#ifdef NUCLEO
	  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
#else
	  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
#endif
	  if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK)
	  {
	    /* Initialization Error */
	    while(1)
	    {

	    }
	  }
	  /* Select PLL as system clock source and configure the HCLK, PCLK1 clocks dividers */
	  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1);
	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1)!= HAL_OK)
	  {
	    /* Initialization Error */
	    while(1);
	  }

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED3 on */
  while(1)
  {
	  BSP_LED_Toggle(LED3);
	  HAL_Delay(100);
  }
}

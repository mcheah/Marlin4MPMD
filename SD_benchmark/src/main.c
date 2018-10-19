/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/

#ifdef __cplusplus
 extern "C" {
#endif
#include "main.h"
#include "ff_gen_drv.h"
#include "sd_diskio.h"
#ifdef __cplusplus
}
#endif
#include "tgunzip.h"
#include "stm32f0xx_3dprinter_cdc.h"
//#include "stm32f0xx.h"
//#include "stm32f0xx_nucleo.h"
char SDPath[4]; /* SD card logical drive path */
FATFS fileSystem;
DIR root;
extern SPI_HandleTypeDef hnucleo_Spi;
#ifdef __cplusplus
extern "C" {
#endif
//////////////////////////////////////C Function////////////////////////////////
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
//int __io_putchar(int ch)
//{
//  /* Place your implementation of fputc here */
//  /* e.g. write a character to the USART3 and Loop until the end of transmission */
//		HAL_UART_Transmit(&(gBspUartData.handle), (uint8_t *)&ch, 1, 0xFFFF);
//  return ch;
//}
//////////////////////////////////End C Function////////////////////////////////
#ifdef __cplusplus
}
#endif

uint8_t USBENABLED = 0;
void setup() {
	HAL_Init();
	SystemClock_Config();
	HAL_InitTick(TICK_INT_PRIORITY);
	BSP_PB_Init(BUTTON_USER,BUTTON_MODE_GPIO);
//	BSP_UartHwInit(921600);
//	BSP_UartIfStart();
	BSP_CdcHwInit(115200);
	BSP_CdcIfStart();
	BSP_SD_Init();
	USBENABLED = 1;
	if(FATFS_LinkDriver(&SD_Driver, SDPath) == 0)
	{
		FRESULT code = FR_OK;
		if ((code = f_mount(&fileSystem, (TCHAR const*)SDPath, 0)) != FR_OK) {
			printf("mount error\r\n");
		}
//		if ((code = f_opendir(&root,SDPath)) != FR_OK) { }
	}
}
typedef enum M34_state {
	M34_IDLE,
	M34_M,
	M34_M2,
	M34_M29,
	M34_RX
} M34_stat;
#define M34_TIMEOUT 1000

void usb_benchmark() {
	BSP_LED_On(LED_BLUE);
	FIL WFILE;
	unsigned char SDbuff[512];
	char filename[40]="COFFEE3.GCO";
	M34_stat M34_state = M34_IDLE;
	FRESULT res = f_open(&WFILE,filename,FA_CREATE_ALWAYS | FA_WRITE);
	if(res!=FR_OK)
		return;
//	p_card->openFile(current_command_args, false);
	//Send OK to ensure we are ready
	BSP_CdcPrintf("OK\r\n");
//    HAL_UART_Transmit(&gBspUartData.handle,"OK\r\n",4,1000);
//	SERIAL_PROTOCOLPGM(MSG_OK);
//	SERIAL_EOL;
	uint32_t last_rx = HAL_GetTick();
	uint16_t SD_idx = 0;
	const char M29_CMD[] = "M29\r\n";
	while(M34_state != M34_RX) {
//      if(BSP_UartGetNbRxAvailableBytes()>0) {
	  if(BSP_CdcGetNbRxAvailableBytes(0)>0) {
		  SD_idx += BSP_CdcCopyNextRxBytes(&SDbuff[SD_idx],512-SD_idx);
//		  SD_idx += BSP_UartCopyNextRxBytes(&SDbuff[SD_idx],512-SD_idx);
		  if(HAL_GetTick()-last_rx>M34_TIMEOUT)
			  M34_state = memcmp(SDbuff,M29_CMD,sizeof(M29_CMD)-1)==0 ? M34_RX : M34_IDLE;
		  if(M34_state==M34_IDLE)
			  last_rx = HAL_GetTick();
	//    		  char serial_char = MYSERIAL.read();
	//    		  switch(M34_state) {
	//    		  	  case M34_IDLE:
	//    		  		  if((millis()-last_rx>M34_TIMEOUT) && serial_char=='M') {
	//    		  			  SERIAL_PROTOCOLPGM("M Received\r\n");
	//    		  			  M34_state = M34_M; }
	//    		  		  else {
	////    		  			  p_card->write_buff(&serial_char,1);
	//    		  			  SDbuff[SD_idx++] = serial_char;
	//    		  			  last_rx = millis();
	//    		  		  }
	//    		  		  break;
	//    		  	  case M34_M:
	//    		  		  if((millis()-last_rx>M34_TIMEOUT) && serial_char=='2') {
	//    		  			SERIAL_PROTOCOLPGM("M2 Received\r\n");
	//    		  			  M34_state = M34_M2; }
	//    		  		  else {
	//    		  			  SERIAL_PROTOCOLPGM("M Received\r\n");
	////    		  			  p_card->write_buff(&serial_char,1);
	//    		  			  SDbuff[SD_idx++] = serial_char;
	//    		  			  last_rx = millis();
	//    		  			  M34_state = M34_IDLE;
	//    		  		  }
	//    		  		  break;
	//    		  	  case M34_M2:
	//    		  		  if((millis()-last_rx>M34_TIMEOUT) && serial_char=='9') {
	//    		  			  SERIAL_PROTOCOLPGM("M29Received\r\n");
	//    		  			  M34_state = M34_M29; }
	//    		  		  else {
	////    		  			  p_card->write_buff(&serial_char,1);
	//    		  			  SDbuff[SD_idx++] = serial_char;
	//    		  			  last_rx = millis();
	//    		  			  M34_state = M34_IDLE;
	//    		  		  }
	//    		  		  break;
	//    		  	  case M34_M29:
	//    		  		  if((millis()-last_rx>M34_TIMEOUT) && (serial_char=='\r' || serial_char=='\n')) {
	//    		  			  SERIAL_PROTOCOLPGM("M\\r\\n Received\r\n");
	//    		  			  M34_state = M34_RX; }
	//    		  		  else {
	////    		  			  p_card->write_buff(&serial_char,1);
	//    		  			  SDbuff[SD_idx++] = serial_char;
	//    		  			  last_rx = millis();
	//    		  			  M34_state = M34_IDLE;
	//    		  		  }
	//    		  		  break;
	//    		  	  case M34_RX:
	//    		  		  break;
	//    		  } //switch(M34_state)
	  } //if MYSERIAL.available()>0
	  //Flush buff to SD if timeout or we fill sector size
	  if(M34_state !=M34_RX && (SD_idx>0) && (SD_idx==sizeof(SDbuff) || HAL_GetTick()-last_rx > M34_TIMEOUT))  {
		  unsigned int count;
		  __disable_irq();
		  f_write(&WFILE,SDbuff,SD_idx,&count);
		  __enable_irq();
//		  p_card->write_buff(SDbuff,SD_idx);
		  SD_idx = 0;
//		  BSP_UartPrintf("OK%d\r\n",HAL_GetTick()%10);
//		  char writebuff[20];
//		  BSP_CdcPrintf("OK%d\r\n",HAL_GetTick()%10);
//		  sprintf(writebuff,"OK%d\r\n",HAL_GetTick()%10);
//		  HAL_UART_Transmit(&gBspUartData.handle,writebuff,5,1000);
//		  printf("OK%d\r\n",HAL_GetTick()%10);
	  }
//	  else if(HAL_GetTick()-last_rx > M34_TIMEOUT)
//	  {
//		printf("OK\r\n");
//		last_rx = HAL_GetTick();
//	  }
	} //while(M34_state!= M34_RX)
	f_close(&WFILE);
//	p_card->closefile();
//	printf("Done saving file.");
//	SERIAL_PROTOCOLLNPGM(MSG_FILE_SAVED);
	BSP_LED_Off(LED_BLUE);
}

void uart_benchmark() {
	BSP_LED_On(LED_BLUE);
	FIL WFILE;
	unsigned char SDbuff[512];
	char filename[40]="COFFEE2.GCO";
	M34_stat M34_state = M34_IDLE;
	f_open(&WFILE,filename,FA_CREATE_ALWAYS | FA_WRITE);
//	p_card->openFile(current_command_args, false);
	//Send OK to ensure we are ready
    HAL_UART_Transmit(&gBspUartData.handle,"OK\r\n",4,1000);
//	SERIAL_PROTOCOLPGM(MSG_OK);
//	SERIAL_EOL;
	uint32_t last_rx = HAL_GetTick();
	uint16_t SD_idx = 0;
	const char M29_CMD[] = "M29\r\n";
	while(M34_state != M34_RX) {
      if(BSP_UartGetNbRxAvailableBytes()>0) {
//	  if(BSP_CdcGetNbRxAvailableBytes(false)>0) {
		  SD_idx += BSP_UartCopyNextRxBytes(&SDbuff[SD_idx],512-SD_idx);
		  if(HAL_GetTick()-last_rx>M34_TIMEOUT)
			  M34_state = memcmp(SDbuff,M29_CMD,sizeof(M29_CMD)-1)==0 ? M34_RX : M34_IDLE;
		  if(M34_state==M34_IDLE)
			  last_rx = HAL_GetTick();
	//    		  char serial_char = MYSERIAL.read();
	//    		  switch(M34_state) {
	//    		  	  case M34_IDLE:
	//    		  		  if((millis()-last_rx>M34_TIMEOUT) && serial_char=='M') {
	//    		  			  SERIAL_PROTOCOLPGM("M Received\r\n");
	//    		  			  M34_state = M34_M; }
	//    		  		  else {
	////    		  			  p_card->write_buff(&serial_char,1);
	//    		  			  SDbuff[SD_idx++] = serial_char;
	//    		  			  last_rx = millis();
	//    		  		  }
	//    		  		  break;
	//    		  	  case M34_M:
	//    		  		  if((millis()-last_rx>M34_TIMEOUT) && serial_char=='2') {
	//    		  			SERIAL_PROTOCOLPGM("M2 Received\r\n");
	//    		  			  M34_state = M34_M2; }
	//    		  		  else {
	//    		  			  SERIAL_PROTOCOLPGM("M Received\r\n");
	////    		  			  p_card->write_buff(&serial_char,1);
	//    		  			  SDbuff[SD_idx++] = serial_char;
	//    		  			  last_rx = millis();
	//    		  			  M34_state = M34_IDLE;
	//    		  		  }
	//    		  		  break;
	//    		  	  case M34_M2:
	//    		  		  if((millis()-last_rx>M34_TIMEOUT) && serial_char=='9') {
	//    		  			  SERIAL_PROTOCOLPGM("M29Received\r\n");
	//    		  			  M34_state = M34_M29; }
	//    		  		  else {
	////    		  			  p_card->write_buff(&serial_char,1);
	//    		  			  SDbuff[SD_idx++] = serial_char;
	//    		  			  last_rx = millis();
	//    		  			  M34_state = M34_IDLE;
	//    		  		  }
	//    		  		  break;
	//    		  	  case M34_M29:
	//    		  		  if((millis()-last_rx>M34_TIMEOUT) && (serial_char=='\r' || serial_char=='\n')) {
	//    		  			  SERIAL_PROTOCOLPGM("M\\r\\n Received\r\n");
	//    		  			  M34_state = M34_RX; }
	//    		  		  else {
	////    		  			  p_card->write_buff(&serial_char,1);
	//    		  			  SDbuff[SD_idx++] = serial_char;
	//    		  			  last_rx = millis();
	//    		  			  M34_state = M34_IDLE;
	//    		  		  }
	//    		  		  break;
	//    		  	  case M34_RX:
	//    		  		  break;
	//    		  } //switch(M34_state)
	  } //if MYSERIAL.available()>0
	  //Flush buff to SD if timeout or we fill sector size
	  if(M34_state !=M34_RX && (SD_idx>0) && (SD_idx==sizeof(SDbuff) || HAL_GetTick()-last_rx > M34_TIMEOUT))  {
		  unsigned int count;
		  f_write(&WFILE,SDbuff,SD_idx,&count);
//		  p_card->write_buff(SDbuff,SD_idx);
		  SD_idx = 0;
//		  BSP_UartPrintf("OK%d\r\n",HAL_GetTick()%10);
		  char writebuff[20];
		  sprintf(writebuff,"OK%d\r\n",HAL_GetTick()%10);
		  HAL_UART_Transmit(&gBspUartData.handle,writebuff,5,1000);
//		  printf("OK%d\r\n",HAL_GetTick()%10);
	  }
//	  else if(HAL_GetTick()-last_rx > M34_TIMEOUT)
//	  {
//		printf("OK\r\n");
//		last_rx = HAL_GetTick();
//	  }
	} //while(M34_state!= M34_RX)
	f_close(&WFILE);
//	p_card->closefile();
	printf("Done saving file.");
//	SERIAL_PROTOCOLLNPGM(MSG_FILE_SAVED);
	BSP_LED_Off(LED_BLUE);
}

void copy_benchmark() {
	BSP_LED_On(LED_BLUE);
//		char filename[40] = "TEST.TXT";
//		char filename[40] = "40HEX.GCO";
	char filename[40] = "COFFEE.GCO";
	char filenameout[40] = "COFFEE2.GCO";
//		char filenameout[40] = "40HEX.GCO";
	FIL RFILE, WFILE;
	UINT count=0xFFFF;
	int eof = 0;
    printf("Start profiling\r\n");
	volatile uint32_t start = HAL_GetTick();
	volatile FRESULT res1 = f_open(&RFILE,(const char *)filename,FA_OPEN_EXISTING | FA_READ);
	volatile FRESULT res2 = f_open(&WFILE,(const char *)filenameout,FA_CREATE_ALWAYS | FA_WRITE);
	if(res1!=FR_OK || res2!=FR_OK)
		return;
	while(count>0 && eof==0)
	{
		char buffer[513] = "";
		res1 = f_read(&RFILE,buffer,512,&count);
		eof = f_eof(&RFILE);
		res2 = f_write(&WFILE,buffer,512,&count);
//		buffer[count] = '\0';
//			printf("%s\r\n",buffer);
	}
	volatile uint32_t total = HAL_GetTick()-start;
    printf("Stop profiling\r\n");
//		printf("decompression took %d\r\n",total);
    printf("%d\r\n",f_tell(&RFILE));
	f_close(&RFILE);
	f_close(&WFILE);
}

void spi_dr_test() {
	BSP_LED_Toggle(LED_BLUE);
	*(uint8_t *)&hnucleo_Spi.Instance->DR = 0xAA55;
	BSP_LED_Toggle(LED_BLUE);
	uint16_t temp = hnucleo_Spi.Instance->DR;
	BSP_LED_Toggle(LED_BLUE);
}

void uzlib_benchmark() {
	BSP_LED_On(LED_BLUE);
	volatile uint32_t start = HAL_GetTick();
//    	p_card->openFile(current_command_args,true,false);
	char filename[40] = "COFFEE.GZ";
	char filenameout[40] = "COFFEE2.GCO";
//	strncpy(filename,current_command_args,ext-current_command_args);
//	filename[ext-current_command_args] = '\0';
    printf("Start profiling\r\n");
	decompress_gzip(filename,filenameout);
//    	unsigned int fsize = p_card->filesize;
	volatile uint32_t total = HAL_GetTick()-start;
//	    printf("%d\r\n",f_tell(&FILE));
//		printf("decompression took %d\r\n",total);
//	SERIAL_PROTOCOLLNPGM("decompression took ");
//	SERIAL_PROTOCOL(total);
}

void sd_read_benchmark() {
	BSP_LED_On(LED_BLUE);
//		char filename[40] = "TEST.TXT";
//		char filename[40] = "40HEX.GCO";
	char filename[40] = "COFFEE.GCO";
//		char filenameout[40] = "40HEX.GCO";
	FIL FILE;
	UINT count=0xFFFF;
	int eof = 0;
	BSP_CdcPrintf("Start profiling\r\n");
//    printf("Start profiling\r\n");
	volatile uint32_t start = HAL_GetTick();
	volatile FRESULT res = f_open(&FILE,(const char *)filename,FA_OPEN_EXISTING | FA_READ);
	if(res!=FR_OK)
		return;
	while(count>0 && eof==0)
	{
		char buffer[513] = "";
		res = f_read(&FILE,buffer,512,&count);
		eof = f_eof(&FILE);
		buffer[count] = '\0';
//			printf("%s\r\n",buffer);
	}
	volatile uint32_t total = HAL_GetTick()-start;
	BSP_CdcPrintf("Stop profiling\r\n");
//    printf("Stop profiling\r\n");
//		printf("decompression took %d\r\n",total);
	BSP_CdcPrintf("%d\r\n",f_tell(&FILE));
	//    printf("%d\r\n",f_tell(&FILE));
	f_close(&FILE);
}

void loop() {
	uint8_t buttonPressed;
	buttonPressed = BSP_PB_GetState(BUTTON_USER);
	if(buttonPressed==GPIO_PIN_RESET)
	{
		usb_benchmark();
	}
	else
	{
//		printf("Hello World\r\n");
//		BSP_CdcPrintf("Hello World\r\n");
		BSP_LED_Off(LED_BLUE);
	}
	HAL_Delay(50);
//	int ch = BSP_UartGetNextRxBytes();
//	if(ch!=-1)
//		__io_putchar((unsigned char)ch);
////		printf("%c",(unsigned char)ch);
//		printf("fuckyou%c",(char)ch);
//		printf("%c",char(ch));
//	else
//		printf("hell world\r\n");

}



int main(void)
{
	BSP_LED_Init(LED_BLUE);
//    BSP_LED_Init(LED_RED);
//    BSP_LED_Init(LED_BLUE);
    BSP_LED_Off(LED_BLUE);
    setup();
	for(;;)
		loop();
}

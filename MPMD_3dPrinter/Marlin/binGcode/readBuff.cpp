#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "readBuff.h"

// char filename[] = "coffee_hanger2B.gcode";
// FILE *fin;

read_buff::read_buff() {
   pReadEnd = readBuff+512;
   pRead = pReadEnd;
}

int read_buff::read_buf(unsigned char *buf,uint32_t len)
{
	unsigned int bytesCopied = 0;
    unsigned int bytesRemaining = len;
	unsigned int bytesRead = -1;
	// FRESULT readStatus;

	while(bytesCopied<len && bytesRead!=0) {
		if(pRead>=pReadEnd) {
			// BSP_LED_On(LED_RED);
			// BSP_LED_On(LED_GREEN);
			// BSP_LED_On(LED_BLUE);
			// readStatus = f_read(&file, readBuff, 512, &bytesRead);
            bytesRead = fread(readBuff,1,512,fin);
            // printf("Reading %d bytes, got %d bytes\n",512,bytesRead);
		//   if( 	(readStatus != FR_OK))
		//   {
			// SERIAL_ERROR_START;
			// SERIAL_ERRORLNPGM(MSG_SD_ERR_READ);
		//   }
			// BSP_LED_Off(LED_RED);
			// BSP_LED_Off(LED_GREEN);
			// BSP_LED_Off(LED_BLUE);
			pRead = readBuff;
			pReadEnd = readBuff +bytesRead;
		}
		int bytesAvailable = pReadEnd - pRead;
		if(bytesAvailable>0) {
            unsigned int bytestoCopy = min(bytesAvailable,bytesRemaining);
			memcpy(buf,pRead,bytestoCopy);
			bytesCopied+=bytestoCopy;
            pRead+=bytestoCopy;
            buf+=bytestoCopy;
            bytesRemaining-=bytestoCopy;
		}
	}
    return bytesCopied;
}

void read_buff::push(int len)
{
    if((pRead-len)<readBuff) {
        unsigned long int curpos;
        curpos = ftell(fin);
        fseek(fin,curpos-((pReadEnd)-pRead+len),SEEK_SET);
        unsigned int bytesRead = fread(readBuff,1,512,fin);
        // printf("Rewinding %ld Reading %d bytes, got %d bytes\n",pRead-readBuff-len,512,bytesRead);
        pRead = readBuff;
        pReadEnd = readBuff+bytesRead;
    }
    else
        pRead = pRead-len;
}

// read_buff rb;
// char line[81]="";
// char * pNL;
// int main() {
//     fin = fopen(filename, "rb");
//     int bytesCopied = -1;
//     while(bytesCopied!=0) {
//         bytesCopied = rb.read_buf((unsigned char *)line,80);
//         if(bytesCopied==0)
//             break;
//         pNL = strchr((char *)line,'\n');
//         if(pNL!=NULL) {
//             rb.push(bytesCopied-((char *)pNL-line+1));
//             *(pNL+1) = '\0';
//         }
//         else
//             rb.push(0);
//         printf("%s",line);
//     }
//     fclose(fin);
//     return 0;
// }
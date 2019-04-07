#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "readBuff.h"

read_buff::read_buff() {
   pReadEnd = readBuff+512;
   pRead = pReadEnd;
}

int read_buff::read_buf(unsigned char *buf,uint32_t len)
{
	unsigned int bytesCopied = 0;
    unsigned int bytesRemaining = len;
	unsigned int bytesRead = -1;

	while(bytesCopied<len && bytesRead!=0) {
		if(pRead>=pReadEnd) {
            bytesRead = fread(readBuff,1,512,fin);
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

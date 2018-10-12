#include <stdint.h>
#include <stdio.h>
#define min(a,b) (((a)<(b))?(a):(b))
#define max(a,b) (((a)>(b))?(a):(b))
class read_buff {
    public:
    read_buff();
	char readBuff[512];
	char *pRead;
	char *pReadEnd;
    FILE *fin;

    int read_buf(unsigned char *buf, uint32_t len);    
    void push(int len);
};
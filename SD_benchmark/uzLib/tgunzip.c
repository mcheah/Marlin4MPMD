/*
 * tgunzip  -  gzip decompressor example
 *
 * Copyright (c) 2003 by Joergen Ibsen / Jibz
 * All Rights Reserved
 *
 * http://www.ibsensoftware.com/
 *
 * Copyright (c) 2014-2016 by Paul Sokolovsky
 *
 * This software is provided 'as-is', without any express
 * or implied warranty.  In no event will the authors be
 * held liable for any damages arising from the use of
 * this software.
 *
 * Permission is granted to anyone to use this software
 * for any purpose, including commercial applications,
 * and to alter it and redistribute it freely, subject to
 * the following restrictions:
 *
 * 1. The origin of this software must not be
 *    misrepresented; you must not claim that you
 *    wrote the original software. If you use this
 *    software in a product, an acknowledgment in
 *    the product documentation would be appreciated
 *    but is not required.
 *
 * 2. Altered source versions must be plainly marked
 *    as such, and must not be misrepresented as
 *    being the original software.
 *
 * 3. This notice may not be removed or altered from
 *    any source distribution.
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#ifdef __cplusplus
 extern "C" {
#endif
#include "uzlib.h"
#include "ff.h"
#ifdef __cplusplus
}
#endif
//#include "cardreader.h"

/* produce decompressed output in chunks of this size */
/* default is to decompress byte by byte; can be any other length */
#define OUT_CHUNK_SIZE 512
#define IN_CHUNK_SIZE 512
//#define printf(x)
void exit_error(const char *what)
{
//   printf("ERROR: %s\n", what);
   exit(1);
}
char *filename;
FIL *p_rFIL;
FIL *p_wFIL;
unsigned char *psource;
int read_bytes(struct uzlib_uncomp *uncomp)
{
	//    static FIL r_FIL;
	    static short done=0;
	    static unsigned int bytes = 0;
//	    unsigned char *ch = uncomp->source;
//	    unsigned char ch = -1;
	    if(!done) {
	        if(p_rFIL->obj.fs==NULL) {
	//        	p_card->openFile(filename,true,true);
	        	volatile FRESULT res = f_open(p_rFIL, (const char *)filename, FA_OPEN_EXISTING | FA_READ);
	        }
	        UINT count;
	        volatile FRESULT res2 = f_read(p_rFIL,psource,IN_CHUNK_SIZE,&count);
	        uncomp->source = psource+1;
	        uncomp->source_limit = psource+count;
	        bytes+=count;
	//        int eof= feof(FIL);
	        int eof = f_eof(p_rFIL);
	        if(eof) {
	//            printf("bytes=%d count=%d eof=%d char=%02X\n",bytes,count,eof,(unsigned char)ch);
//				uncomp->eof = 1;
	            done=1;
	//        	p_card->closefile();
	            f_close(p_rFIL);
	//	    	fclose(FIL);
	        }
	        return *psource;
	    }
	    else
	        return -1;
}
int read_byte(struct uzlib_uncomp *uncomp)
{
//    static FIL r_FIL;
    static short done=0;
    static unsigned int bytes = 0;
    unsigned char ch = -1;
    if(!done) {
        if(p_rFIL->obj.fs==NULL) {
//        	p_card->openFile(filename,true,true);
        	volatile FRESULT res = f_open(p_rFIL, (const char *)filename, FA_OPEN_EXISTING | FA_READ);
        }
        UINT count;
        volatile FRESULT res2 = f_read(p_rFIL,&ch,1,&count);
        bytes+=count;
//        int eof= feof(FIL);
        int eof = f_eof(p_rFIL);
	if(count!=1 || eof) {
//            printf("bytes=%d count=%d eof=%d char=%02X\n",bytes,count,eof,(unsigned char)ch);
            done=1;
//        p_card->closefile();
        f_close(p_rFIL);
//	    fclose(FIL);
	    return -1;
        }
        else
	    return ch;
    }
    else
        return -1;
}

int decompress_gzip(char *fname_in, char *fname_out)
{
    FIL w_FIL;
    FIL r_FIL;
    p_wFIL = &w_FIL;
    p_rFIL = &r_FIL;
//    FILE *fin, *fout;
    volatile unsigned int len, dlen, outlen;
//    const unsigned char *source;
//    unsigned char *dest;
//    int res;
    filename = fname_in;
//    printf("tgunzip - example from the tiny inflate library (www.ibsensoftware.com)\n\n");

//    if (argc < 3)
//    {
//       printf(
//          "Syntax: tgunzip <source> <destination>\n\n"
//          "Both input and output are kept in memory, so do not use this on huge files.\n");
//
//       return 1;
//    }

    uzlib_init();

    /* -- open files -- */
	volatile FRESULT res = f_open(&r_FIL, (const char *)fname_in, FA_OPEN_EXISTING | FA_READ);
//    if ((fin = fopen(argv[1], "rb")) == NULL) exit_error("source file");

//    if ((fout = fopen(argv[2], "wb")) == NULL) exit_error("destination file");

    /* -- read source -- */
    FSIZE_t wsize = f_size(&r_FIL);

//	f_lseek(&w_FIL, 0, SEEK_END);

//    len = ftell(fin);
    len = 4;
    f_lseek(&r_FIL, wsize-4);
    unsigned char source[IN_CHUNK_SIZE];
    psource = source;
//    source = (unsigned char *)malloc(len);

//    if (source == NULL) exit_error("memory");
    UINT bytes_read;
    f_read(&r_FIL,source, len,&bytes_read);

    f_close(&r_FIL);
//    strcpy(filename,argv[1]);
    if (len < 4) exit_error("file too small");

    /* -- get decompressed length -- */

    dlen =            source[len - 1];
    dlen = 256*dlen + source[len - 2];
    dlen = 256*dlen + source[len - 3];
    dlen = 256*dlen + source[len - 4];

    outlen = dlen;

    /* there can be mismatch between length in the trailer and actual
       data stream; to avoid buffer overruns on overlong streams, reserve
       one extra byte */
    dlen++;
    unsigned char dest[520];
//    dest = (unsigned char *)malloc(dlen);

//    if (dest == NULL) exit_error("memory");

    /* -- decompress data -- */

    struct uzlib_uncomp d;
    unsigned char dict_ring[520];
    uzlib_uncompress_init(&d, dict_ring, sizeof(dict_ring));
    // uzlib_uncompress_init(&d, NULL, 0);

    /* all 3 fields below must be initialized by user */
    d.source = source+IN_CHUNK_SIZE;
//    d.source_limit = source + len - 4;
    d.source_limit = source+IN_CHUNK_SIZE;
    d.source_read_cb = read_bytes;

    int res3 = uzlib_gzip_parse_header(&d);
    if (res3 != TINF_OK) {
//        printf("Error parsing header: %d\n", res);
    	exit_error(1);
    }

    d.dest_start = d.dest = dest;
	volatile FRESULT res4 = f_open(&w_FIL, (const char *)fname_out, FA_CREATE_ALWAYS | FA_WRITE);

    while (dlen) {
	    d.dest = dest;
        unsigned int chunk_len = dlen < OUT_CHUNK_SIZE ? dlen-1 : OUT_CHUNK_SIZE;
        d.dest_limit = d.dest + chunk_len;
        res3 = uzlib_uncompress_chksum(&d);
        dlen -= chunk_len;
        volatile UINT bytes_written;
        volatile FRESULT res2 = f_write(&w_FIL, (unsigned char *)dest, chunk_len, (UINT *) &bytes_written);
        if (res3 != TINF_OK) {
            break;
        }
	    // printf("%s\n",dest);
    }

    if (res3 != TINF_DONE) {
//        printf("Error during decompression: %d\n", res);
    	exit_error(-res3);
    }

//    printf("Decompressed %lu bytes\n", d.dest - dest);
    printf("Stop profiling\r\n");
    printf("%d\r\n",outlen);

#if 0
    if (d.dest - dest != gz.dlen) {
//        printf("Invalid decompressed length: %lu vs %u\n", d.dest - dest, gz.dlen);
    }

    if (tinf_crc32(dest, gz.dlen) != gz.crc32) {
//        printf("Invalid decompressed crc32\n");
    }
#endif

    /* -- write output -- */

    // fwrite(dest, 1, outlen, fout);

    f_close(&w_FIL);
    //Clear pointers
    p_rFIL = NULL;
    p_wFIL = NULL;
    filename = NULL;
    psource = NULL;
    return 0;
}

/*
 * testbinGcodeCommand.cpp
 *
 *  Created on: Oct 2, 2018
 *      Author: MCheah
 */
#include "binGcodeCommand.h"
#include "readBuff.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
typedef struct bpartestPar{
    char parPrefix;
    float parVal;
    gcodeParameterFormat parFormat;
    char parStr[80];
}bpartestPar;

typedef struct bgctestData {
    uint8_t inputdata[80];
    bool isBinary;
    bool usePrevFormat;
    char cmdPrefix;
    uint16_t cmdCode;
    uint8_t numPar;
    bpartestPar par[8];
}bgctestData;

bpartestPar par= {'\0',0,gcode_F32};
// bgctestData test = {{ 0x00,0x01,0x02,0x03 },true,true};
bgctestData testData4[] = {
{
        {0x67,0x00,0xd7,0xf8,0x53,0x87,0xc0,0xd8,0xc1,0xca,0x98,0x41,0xc4,0x00,0xa5,0x3b,0x43},
        true,
        false,
        'G',
        1,
        3,
        {{'X',-4.229,gcode_F32},
        {'Y',19.099,gcode_F32},
        {'E',187.64453,gcode_F32}}
}
,
{
        {0x37,0x37,0x89,0x91,0xc0,0xbe,0x9f,0x98,0x41,0x44,0xb3,0x3b,0x43},
        true,
        true,
        'G',
        1,
        3,
        {{'X',-4.548,gcode_F32},
        {'Y',19.078,gcode_F32},
        {'E',187.70025,gcode_F32}}
}
,
{
        {0xa2,0x2f,0x32,0x32},
        true,
        false,
        'M',
        190,
        1,
        {{'S',50.0,gcode_U8}}
}
,
{
        {0x61,0x05},
        true,
        false,
        'G',
        21,
        0
}
,
{
        {0x29,0x00,0x45,0x44,0x16,0xd7,0x3f,0x35,0xea,0xc0,0xd8,0xe7,0xfb,0xb9,0xc0,0xd9,0x3d,0x0a,0xd7,0x3e},
        true,
        false,
        'G',
        0,
        4,
        {{'F',5700.0,gcode_U16},
        {'X',-7.319,gcode_F32},
        {'Y',-5.812,gcode_F32},
        {'Z',0.42,gcode_F32}}
}
,
{
        {0x21,0x07},
        true,
        false,
        'G',
        28,
        0
}
,
{
        {0x65,0x00,0x45,0x3c,0x0f,0xc4,0x00,0x00,0xb0,0xc0},
        true,
        false,
        'G',
        1,
        2,
        {{'F',3900.0,gcode_U16},
        {'E',-5.5,gcode_F32}}
}
,
{
        {0x67,0x00,0xd7,0x6f,0x12,0xd3,0xc0,0xd8,0xdb,0xf9,0xde,0xc0,0xc4,0x2f,0x6e,0x43,0x3e},
        true,
        false,
        'G',
        1,
        3,
        {{'X',-6.596,gcode_F32},
        {'Y',-6.968,gcode_F32},
        {'E',0.19085,gcode_F32}}
}
,
{
        {0x69,0x00,0x45,0x2a,0x03,0xd7,0xa4,0x70,0xdd,0xc0,0xd8,0xa8,0xc6,0xcb,0xc0,0xc4,0xef,0xc9,0xc3,0x3d},
        true,
        false,
        'G',
        1,
        4,
        {{'F',810.0,gcode_U16},
        {'X',-6.92,gcode_F32},
        {'Y',-6.368,gcode_F32},
        {'E',0.0956,gcode_F32}}
}
,
{
        {0x23,0x01,0xb2,0xa0,0x86,0x01,0x00},
        true,
        false,
        'G',
        4,
        1,
        {{'S',100000.0,gcode_I32}}
}
,
{
        {0x63,0x00,0x99,0x18,0xfc},
        true,
        false,
        'G',
        1,
        1,
        {{'Z',-1000.0,gcode_I16}}
}
,
{
        {0x63,0x00,0x05},
        true,
        false,
        'G',
        1,
        1,
        {{'F',NAN,gcode_NONE}}
}
,
{
        {0x62,0x1d,0xef,0x72,0x69,0x6e,0x74,0x69,0x6e,0x67,0x2e,0x2e,0x2e,0x2e,0x00},
        true,
        false,
        'M',
        117,
        1,
        {{'P',0,gcode_string,"rinting...."}}
}
};
bgctestData testData3[] = {
    {
    {0x67,0x00,0xd7,0xc0,0x87,0x53,0xf8,0xd8,0x41,0x98,0xca,0xc1,0xc4,0x43,0x3b,0xa5,0x00},
    true,
    false,
    'G',
    1,
    3,
    {{'X',-4.229,gcode_F32},
     {'Y',19.099,gcode_F32},
     {'Z',187.64453,gcode_F32}}
},
{
    {0x67,0x00,0xd7,0xc0,0x87,0x53,0xf8,0xd8,0x41,0x98,0xca,0xc1,0xc4,0x43,0x3b,0xa5,0x00},
    true,
    false,
    'G',
    1,
    3,
    {{'X',-4.229,gcode_F32},
     {'Y',19.099,gcode_F32},
     {'Z',187.64453,gcode_F32}}
},
{
    {0x67,0x00,0xd7,0xc0,0x87,0x53,0xf8,0xd8,0x41,0x98,0xca,0xc1,0xc4,0x43,0x3b,0xa5,0x00},
    false,
    false,
    'G',
    1,
    3,
    {       {'X',-4.229,gcode_F32},
    {'Y',19.099,gcode_F32},
    {'E',187.64453,gcode_F32}}
}};


bgctestData testData1 = {
    {0x67,0x00,0xd7,0xc0,0x87,0x53,0xf8,0xd8,0x41,0x98,0xca,0xc1,0xc4,0x43,0x3b,0xa5,0x00},
    true,
    false,
    'G',
    1,
    3,
    {{'X',-4.229,gcode_F32},
     {'Y',19.099,gcode_F32},
     {'Z',187.64453,gcode_F32}}
};
bgctestData testData2 = {
    {0x67,0x00,0xd7,0xc0,0x87,0x53,0xf8,0xd8,0x41,0x98,0xca,0xc1,0xc4,0x43,0x3b,0xa5,0x00},
    false,
    false,
    'G',
    1,
    3,
    {       {'X',-4.229,gcode_F32},
    {'Y',19.099,gcode_F32},
    {'E',187.64453,gcode_F32}}
};
bool testBGC(const bgctestData testdata,binGcodeCommand BGC) {
    bool isEqual = true;
    isEqual &= BGC.isBinary==testdata.isBinary;
    isEqual &= BGC.usePrevFormat==testdata.usePrevFormat;
    isEqual &= BGC.cmdPrefix==testdata.cmdPrefix;
    isEqual &= BGC.cmdCode==testdata.cmdCode;
    isEqual &= BGC.numPar==testdata.numPar;
    for(int i=0;i<testdata.numPar;i++) {
        if(!isEqual)
            break;
        isEqual&=BGC.par[i].parPrefix==testdata.par[i].parPrefix;
        isEqual&=(fabs(BGC.par[i].parVal-testdata.par[i].parVal)<0.001 ||
                 (isnanf(BGC.par[i].parVal) && isnanf(testdata.par[i].parVal)) ||
                 (isinff(BGC.par[i].parVal) && isinff(testdata.par[i].parVal)));
        isEqual&=BGC.par[i].parFormat==testdata.par[i].parFormat;
        if(BGC.par[i].parFormat==gcode_string)
            isEqual&=strcmp(BGC.par[i].parStr,testdata.par[i].parStr)==0;
        else
            isEqual&=BGC.par[i].parStr==NULL;
    }
    return isEqual;
}


bool test_case1() {
    uint8_t bytes[] = {0xA2,0x2F,0x32,0x32,0x22,0x1A,0x32,0xC8,0x62,0x1B};
    char buff[80];
    binGcodeCommand gc2;
    binGcodeCommand gc3;
    bool passfail[sizeof(testData4)/sizeof(bgctestData)];
    // gc1.decodeBinGcode(bytes,gc2);
    uint8_t *pData = testData4[2].inputdata;
    gc3.decodeBinGcode(pData,gc2);
    pData = testData4[0].inputdata;
    gc2.decodeBinGcode(pData,gc2);
    for(int i=0;i<sizeof(testData4)/sizeof(bgctestData);i++)
    {
        pData = testData4[i].inputdata;
        binGcodeCommand gc1;
        gc1.decodeBinGcode(pData,gc2);
        gc1.writeGcode(buff);
        printf("%s",buff);
        passfail[i] = testBGC(testData4[i],gc1);
        if(!passfail[i])
            printf("Failure at index %d\n",i);
    }
    return true;
}

bool test_case2() {
    char filename[] = "coffee_hanger2B.bgcode";
    char filename_out[] = "coffee_hanger2B.gcode";
    FILE *fin,*fout;
    unsigned long int len,dlen,outlen;
    const char *source;
    const char *dest;
    char *wp;
    unsigned char *rp;
    fin = fopen(filename, "rb");
    // fin = fopen(filename,"rb");
    fout = fopen(filename_out,"wb");
    fseek(fin,0,SEEK_END);
    len = ftell(fin);
    fseek(fin,0,SEEK_SET);
    source = (char *)malloc(len);
    dest = (char *)malloc(5000000);
    fread((unsigned char*)source,1,len,fin);
    rp = (unsigned char *)source;
    wp = (char *)dest;
    uint32_t i = 0;
    binGcodeCommand gc2;
    char buff[80];
    while(rp-(unsigned char *)source<len-1) {
        binGcodeCommand gc1;
        unsigned char *pStart = rp;
        gc1.decodeBinGcode(rp,gc2); //rp is incremented for us internally
        gc2.decodeBinGcode(pStart,gc2); //run again since we don't have a method for copy
        gc1.writeGcode(buff);
        printf("%s",buff);
        int count = gc1.writeGcode(wp);
        wp+=count;
        i++;
    }
    fwrite(dest,1,wp-dest,fout);
    fclose(fin);
    fclose(fout);
    return true;
} 
read_buff rb;
char line[81]="";
char *pNL;
bool test_case3() {
    char filename[] = "coffee_hanger2B.bgcode";
    char filename_out[] = "coffee_hanger2C.gcode";
    FILE /**fin,*/*fout;
    // rb.fin = fin;
    unsigned long int len,dlen,outlen;
    // const char *source;
    // const char *dest;
    // char *wp;
    unsigned char *rp;
    rb.fin = fopen(filename, "rb");
    // fin = fopen(filename,"rb");
    fout = fopen(filename_out,"wb");
    fseek(rb.fin,0,SEEK_END);
    len = ftell(rb.fin);
    fseek(rb.fin,0,SEEK_SET);
    // source = (char *)malloc(len);
    // dest = (char *)malloc(5000000);
    // fread((unsigned char*)source,1,len,fin);
    // rp = (unsigned char *)source;
    // wp = (char *)dest;
    uint32_t i = 0;
    // binGcodeCommand gc2;
    char buff[80];
    int bytesCopied=-1;
    // while(rp-(unsigned char *)source<len-1) {
    while(bytesCopied!=0) {
        bytesCopied = rb.read_buf((unsigned char *)line,80);
        if(bytesCopied==0)
            break;
        rp = (unsigned char *)line;
        binGcodeCommand gc1;
        static binGcodeCommand gc2;
        unsigned char *pStart = rp;
        gc1.decodeBinGcode(rp,gc2); //rp is incremented for us internally
        gc2.decodeBinGcode(pStart,gc2); //run again since we don't have a method for copy
        int count = gc1.writeGcode(buff);
        rb.push(bytesCopied-((char *)rp-line));
        printf("%s",buff);
        fwrite(buff,1,count,fout);
        // int count = gc1.writeGcode(wp);
        // wp+=count;
        i++;
    }
    // fwrite(dest,1,wp-dest,fout);
    fclose(rb.fin);
    fclose(fout);
    return true;
} 

int main() {
    // test_case1();
    test_case2();
    test_case3();
    return 0;
}
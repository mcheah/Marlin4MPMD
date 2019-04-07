/*
 * binGcodePar.cpp
 *
 *  Created on: Oct 2, 2018
 *      Author: MCheah
 */
#include <stdlib.h>
#include <stdint.h>
// #include <inttypes.h>
#include "binGcodePar.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
char binGcodePar::strbuff[80];
char *binGcodePar::nextAvailableBuff = binGcodePar::strbuff;
binGcodePar::binGcodePar() {
    parPrefix = '\0';
    parVal = NAN;
    parFormat = gcode_NONE;
    parStr = NULL;
}
void binGcodePar::resetBuff() {
    binGcodePar::nextAvailableBuff = binGcodePar::strbuff;
}
// TODO Auto-generated constructor stub
bool binGcodePar::isEqualFormat(const binGcodePar &comm) {
    return parPrefix == comm.parPrefix;
}
bool binGcodePar::isEqual(const binGcodePar &comm) {
    return parVal == comm.parVal;
}
void binGcodePar::decodeBinParFormat(uint8_t *buff) {
    uint8_t parByte = buff[0];
    parFormat = (gcodeParameterFormat) (parByte>>5);
    parPrefix = (parByte & ((1<<5)-1)) + 'A';
}    
void binGcodePar::decodeBinParData(uint8_t*& buff) {
    if(buff==NULL)
        return;
    parStr = NULL;
    switch(parFormat) {
        case gcode_NONE:
        parVal = NAN;
        break;
        case gcode_U8:
        parVal = *(uint8_t *)buff++;
        break;
        case gcode_U16: {
        uint16_t temp;
        memcpy(&temp,buff,sizeof(uint16_t));
        parVal = temp;
        buff+=2; }
        break;
        case gcode_I8:
        parVal = *(int8_t *)buff++;
        break;
        case gcode_I16: {
        int16_t temp;
        memcpy(&temp,buff,sizeof(int16_t));            
        parVal = temp;
        buff+=2;
        break; }
        case gcode_I32: {
        int32_t temp;
        memcpy(&temp,buff,sizeof(int32_t));            
        parVal = temp;
        buff+=4;
        break; }
        case gcode_F32: {
        memcpy(&parVal,buff,sizeof(float));
        buff+=4;
        break; }
        case gcode_string:
        parVal = 0;
        parStr = binGcodePar::nextAvailableBuff;
        strcpy(parStr,(const char*)buff);
        binGcodePar::nextAvailableBuff+=strlen((const char*)buff)+1;
        buff+=strlen((const char*)buff)+1;
        break;
        default:
        parVal = NAN;

        break;
    }
}
int binGcodePar::writeGcode(char *buff) {
    int count =0 ;
    switch(parFormat)
    {
        case gcode_NONE:
        count = sprintf(buff,"%c ",parPrefix);
        break;
        case gcode_U8:
        case gcode_U16:
        case gcode_I8:
        case gcode_I16:
        case gcode_I32:
        count = sprintf(buff,"%c%0.0f ",parPrefix,parVal);
        break;
        case gcode_F32:
        count = sprintf(buff,"%c%.9g ",parPrefix,parVal);
        break;        
        case gcode_string:
        count = sprintf(buff,"%c%s ",parPrefix,parStr);
    }
    return count;
}

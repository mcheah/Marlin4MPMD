/*
 * binGcodeCommand.cpp
 *
 *  Created on: Oct 2, 2018
 *      Author: MCheah
 */

#include "binGcodeCommand.h"
#include "binGcodePar.h"
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
binGcodeCommand::binGcodeCommand() {
	isBinary = true;
	usePrevFormat = false;
	cmdPrefix = '\0';
	cmdCode = 0;
	numPar = 0;
}
bool binGcodeCommand::isEqualFormat(const binGcodeCommand &comm) {
	bool isEqual= (cmdPrefix == comm.cmdPrefix);
	isEqual&= (cmdCode == comm.cmdCode);
	isEqual&= (numPar == comm.numPar);
	for(int i=0;i<numPar;i++) {
		if(isEqual)
			isEqual&= par[i].isEqualFormat(comm.par[i]);
		else
			break;
	}
	return isEqual;
}
bool binGcodeCommand::isEqual(const binGcodeCommand &comm) {
	bool isEqual = isEqualFormat(comm);
	for(int i=0;i<numPar;i++) {
		if(!isEqual)
			break;
		isEqual&= par[i].isEqual(comm.par[i]);
	}
	return isEqual;
}

void binGcodeCommand::decodeBinGcode(uint8_t*& buff, const binGcodeCommand &comm) {
	if(buff==NULL)
		return;
	uint8_t cmdBytes[2];
	cmdBytes[0] = *buff++;
	isBinary = cmdBytes[0]>>5;
	usePrevFormat = (cmdBytes[0]>>4) & 0x01;
	numPar = (cmdBytes[0]>>1) & ((1<<3)-1);
	if(!isBinary || (usePrevFormat && &comm==NULL))
		return;
	if(!usePrevFormat) {
		cmdPrefix = (cmdBytes[0] & 0x01) ? 'G' : 'M';
		cmdBytes[1] = *buff++;
		cmdCode = *((uint16_t *)cmdBytes) >>6;
	}
	else {
		cmdPrefix = comm.cmdPrefix;
		cmdCode = comm.cmdCode;
		for(int i=0;i<numPar;i++) {
			par[i].parPrefix = comm.par[i].parPrefix;
			par[i].parFormat = comm.par[i].parFormat;
		}
	}
	for(int i=0;i<numPar;i++) {
		if(!usePrevFormat)
			par[i].decodeBinParFormat(buff++);
		par[i].decodeBinParData(buff);
	}
}

int binGcodeCommand::writeGcode(char *buff) {
	int total = sprintf(buff,"%c%d ",cmdPrefix,cmdCode);
	buff+=total;
	for(int i=0;i<numPar;i++){
		int count = par[i].writeGcode(buff);
		buff+=count;
		total+=count;
	}
	sprintf(buff,"\r\n");
	buff+=2;
	total+=2;
	return total;
}

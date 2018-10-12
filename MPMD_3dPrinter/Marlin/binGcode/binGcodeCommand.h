/*
 * binGcodeCommand.h
 *
 *  Created on: Oct 2, 2018
 *      Author: Kuli
 */
#include <stdint.h>
#include "binGcodePar.h"
#ifndef __BINGCODECOMMAND_H_
#define __BINGCODECOMMAND_H_

class binGcodeCommand {
public:
	bool isBinary;
	bool usePrevFormat;
	char cmdPrefix;
	uint16_t cmdCode;
	binGcodePar par[8];
	uint8_t numPar;
	binGcodeCommand();
	// bool parseGcode(char *line);
	// void encodeBinGcode(char *outBuff);
	bool isEqualFormat(const binGcodeCommand &comm);
	bool isEqual(const binGcodeCommand &comm);
	void decodeBinGcode(uint8_t*& buff, const binGcodeCommand &comm);
	int writeGcode(char *buff);
	// static readUntilChar(char *buff);
//	static int findNextSpace(char *buff);
// private:

};

#endif /* BINGCODECOMMAND_H_ */

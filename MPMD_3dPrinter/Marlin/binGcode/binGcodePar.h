/*
 * binGcodePar.h
 *
 *  Created on: Oct 2, 2018
 *      Author: MCheah
 */

#ifndef __BINGCODEPAR_H_
#define __BINGCODEPAR_H_

typedef enum gcodeParameterFormat {
	gcode_NONE=0,
	gcode_U8,
	gcode_U16,
	gcode_I8,
	gcode_I16,
	gcode_I32,
	gcode_F32,
	gcode_string
} gcodeParameterFormat;

class binGcodePar {
public:
	char parPrefix;
	float parVal;
	char *parStr;
    gcodeParameterFormat parFormat;
	binGcodePar();
	bool isEqualFormat(const binGcodePar &comm);
	bool isEqual(const binGcodePar &comm);
    void decodeBinParFormat(uint8_t *buff);
	void decodeBinParData(uint8_t*& buff);
	int writeGcode(char * buff);
// private:
	//TODO:find a less hacky way to deal with large string buffers without excessive memory or malloc
	static void resetBuff();
	static char strbuff[80];
	static char *nextAvailableBuff;
};

#endif /* __BINGCODECOMMAND_H_ */

#include "DataProtocol.h"

uint8_t* BuildBuffer(DataBlock dB)
{
	uint8_t *bufferOut = new uint8_t(12);
	
	bufferOut[0] = dB.IDboard;
	bufferOut[1] = dB.command;
	bufferOut[2] = dB.RFID[0];
	bufferOut[3] = dB.RFID[1];
	bufferOut[4] = dB.RFID[2];
	bufferOut[5] = dB.RFID[3];
	bufferOut[6] = dB.door;
	bufferOut[7] = dB.day;
	bufferOut[8] = dB.hourFrom;
	bufferOut[9] = dB.minuteFrom;
	bufferOut[10] = dB.hourTo;
	bufferOut[11] = dB.minuteTo;
	
	return bufferOut;
}

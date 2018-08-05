#ifndef __DATAPROTOCOL_H
#define __DATAPROTOCOL_H
 
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f10x.h"

#define NONE 0
#define IDCHECK 1
#define IDADD	2
#define IDDEL 3
#define SETDS1307 4
#define DS1307 5
#define OPEN 6
#define CLEARALL 7
#define STOREOK 8
#define GETINFO 9
#define SETINFO 10

// Stuct 12 bytes
typedef struct
{
	uint8_t IDboard;
	uint8_t command;
	uint8_t RFID[5];
	uint8_t door;
	uint8_t day;
	uint8_t hourFrom;
	uint8_t minuteFrom;
	uint8_t hourTo;
	uint8_t minuteTo;
}DataBlock;

uint8_t* BuildBuffer(DataBlock dB);
	
#ifdef __cplusplus
}
#endif
 
#endif

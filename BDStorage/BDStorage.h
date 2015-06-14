

#ifndef __BDSTORAGE_H__
#define __BDSTORAGE_H__

#include <stdio.h>
//#include "pins_arduino.h"
#include <Wire.h>
#include <BubbleDetector.h>

//constants up here
const uint8_t DEV_ADDR_LOW =  0b01010000;	//this is the 7 bit I2C address - the wire library adds in the 8th bit depending on read or write
const uint8_t DEV_ADDR_HIGH = 0b01010100;
const uint8_t EMPTY_PAGE = 	0xFF;
const uint16_t MAX_RECORDS = 1024;

class BDStorage{

public:
	uint16_t begin();
	uint8_t saveBubbleEvent(struct eventinfo* eventInfo, struct bubbledata * bdata);
	uint8_t readEvent(struct eventinfo* eventInfo, struct bubbledata * bdata);
	uint32_t getRdAddr();
 	uint32_t getWrAddr();
	uint32_t getDataStartAddr();
	uint16_t getNumRecords();
	void deleteData();
	void resetRead();
	
	
private:
	uint32_t dataStartAddr;	
	uint32_t rdAddr;
	uint32_t wrAddr;
	uint16_t numRecords;
	
	void Write8bit(uint8_t data);
	void Write16bit(uint16_t data);
	void Write32bit(uint32_t data);
	uint8_t Read8bit(uint8_t sendStop);
	uint16_t Read16bit(uint8_t sendStop);
	uint32_t Read32bit(uint8_t sendStop);
};

extern BDStorage bdstorage;

#endif //__BDSTORAGE_H__
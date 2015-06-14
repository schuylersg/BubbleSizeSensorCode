#include "BDStorage.h"

BDStorage bdstorage;

// Public methods
uint16_t BDStorage::begin(){
	//Start I2C communication
	Wire.begin();
	
	//find data
	rdAddr = 0;	
	uint8_t val;

	uint8_t prevHasData;
	
	rdAddr = 0x00020000 - 128;
	Wire.beginTransmission(DEV_ADDR_HIGH);
	Wire.write((uint8_t)(rdAddr >> 8));    //MSB = most significant bits	
	Wire.write((uint8_t)(rdAddr & 0xFF));
	Wire.endTransmission();
	val = Read8bit((uint8_t) true);	
	if(val == 0xFF)
		prevHasData = 0;
	else
		prevHasData = 1;
	
	rdAddr = 0;
	
	wrAddr = 0xFFFFFFFF;
	dataStartAddr = 0xFFFFFFFF;
	numRecords = 0;
	
	while(rdAddr < 0x00020000 && (wrAddr == 0xFFFFFFFF || dataStartAddr == 0xFFFFFFFF)){
		if(rdAddr > 0xFFFF){
			Wire.beginTransmission(DEV_ADDR_HIGH);
		}else{
			Wire.beginTransmission(DEV_ADDR_LOW);
		}
		
		Wire.write((uint8_t)(rdAddr >> 8));    //MSB = most significant bits	
		Wire.write((uint8_t)(rdAddr & 0xFF));
		Wire.endTransmission();
		val = Read8bit((uint8_t) true);
		if(prevHasData == 0 && val !=0xFF){
			dataStartAddr = rdAddr;
		}
		if(prevHasData == 1 && val == 0xFF)
			wrAddr = rdAddr;
		
		rdAddr += 128;
		if(val == 0xFF)
			prevHasData = 0;
		else{
			numRecords++;
			prevHasData = 1;
		}
	}
	
    uint32_t a1, a2, a3;
	/**
	a1 = analogRead(A6);
	a2 = analogRead(A7);
	a3 = ((a2 & 0x1) << 16) | ((a1 & 0x1) << 15) | ((a1 & 0x2) << 14) | ((a1 & 0x2) << 13) | ((a1 & 0x4) << 12) | ((a2 & 0x4) << 11) | ((a1 & 0x8) << 10) | ((a1 & 0x8) << 9) ;
	**/
	a3 = 0;
	if(numRecords == MAX_RECORDS){	//case where memory is full - we don't know where the actual "start" is.
	//TO DO - look for earliest time stamp
		wrAddr = 0;
		rdAddr = 0;
	}else if(numRecords == 0){	//case where there is no data
		dataStartAddr = a3;
		wrAddr = a3;			//maybe make this a random number
		rdAddr = a3;
		numRecords = 0;
	}/**else if(wrAddr > dataStartAddr)
		numRecords = (wrAddr - dataStartAddr) >>7;
	else 
		numRecords = ((0x00020000 - dataStartAddr) + wrAddr) >> 7;
	**/
	return numRecords;
}

uint8_t BDStorage::saveBubbleEvent(struct eventinfo* eventInfo, struct bubbledata * bdata)
{
	//Check to make sure there is space to write
	if(numRecords >= MAX_RECORDS){
		return 1;
	}

	if(wrAddr > 0xFFFF){
		Wire.beginTransmission(DEV_ADDR_HIGH);
	} else {
		Wire.beginTransmission(DEV_ADDR_LOW);
	}
	
	Wire.write((uint8_t)(wrAddr >> 8));    //MSB = most significant bits	
	Wire.write((uint8_t)(wrAddr & 0xFF));  //LSB = least " "
	
	Write8bit(eventInfo->errorVal);
	Write32bit(eventInfo->eventTime);
	Write8bit(min(eventInfo->numBubbles, 4));
	Write16bit(eventInfo->det2avg);
	Write16bit(eventInfo->det2startval);
	Write16bit(eventInfo->det2endval);
	Write16bit(eventInfo->det3avg);
	Write16bit(eventInfo->det3startval);
	Write16bit(eventInfo->det3endval);
	Wire.endTransmission((uint8_t)true);	//do not send a stop signal to the chip
	delay(5);
	wrAddr += 18;
	
	//total bytes written to chip == 18
	//total bytes sent to wire buffer 21
	
	//can only save event info and 4 bubbles in one page of memory
	for (uint8_t i = 0; i < min(eventInfo->numBubbles, 4); i ++){
		if(wrAddr > 0xFFFF)
			Wire.beginTransmission(DEV_ADDR_HIGH);
		else
			Wire.beginTransmission(DEV_ADDR_LOW);
		
		Wire.write((uint8_t)(wrAddr >> 8));    //MSB = most significant bits	
		Wire.write((uint8_t)(wrAddr & 0xFF));  //LSB = least " "

		Write32bit(bdata[i].d2stime);
		Write16bit(bdata[i].d2sval);
		Write32bit(bdata[i].d2etime);
		Write16bit(bdata[i].d2eval);
		Write32bit(bdata[i].d3stime);
		Write16bit(bdata[i].d3sval);
		Write32bit(bdata[i].d3etime);
		Write16bit(bdata[i].d3eval);
		if(i < min(eventInfo->numBubbles, 4)-1)
			Wire.endTransmission((uint8_t)true);
		else
			Wire.endTransmission((uint8_t)true);
		delay(5);
		wrAddr += 24;
		//24 bytes
	}
		
	wrAddr = ((wrAddr >> 7) + 1) << 7;
	numRecords += 1;
	
	//wrap the write address around to zero
	if(wrAddr >=0x00020000)
		wrAddr = 0;
	
	if(eventInfo->numBubbles > 4){
		delay(5);	//wait for page to write - can do with polling later
		eventInfo->numBubbles -= 4;
		//save the remaining bubbles on new memory page
		saveBubbleEvent(eventInfo, bdata+sizeof(bubbledata)*4);
	}
	return 0;
}

uint8_t BDStorage::readEvent(struct eventinfo* eventInfo, struct bubbledata * bdata){
	
	if(wrAddr > dataStartAddr)
	{
		if(rdAddr >= wrAddr)
		{
			return 1;
		}
	}else{
		if(rdAddr < dataStartAddr && rdAddr >=wrAddr)
		{
			return 2;
		}
	}	
	
	if(rdAddr > 0xFFFF){
		Wire.beginTransmission(DEV_ADDR_HIGH);
	}else{
		Wire.beginTransmission(DEV_ADDR_LOW);
	}
	
	Wire.write((uint8_t)(rdAddr >> 8));    //MSB = most significant bits	
	Wire.write((uint8_t)(rdAddr & 0xFF));
	Wire.endTransmission();
	
	
	//The value of rdAddr is used (but not changed) in Read_bit()
	//So do not change the value of rdAddr until done
	//reading from this page	
	eventInfo->errorVal = Read8bit((uint8_t)false);
	eventInfo->eventTime = Read32bit((uint8_t)false);
	eventInfo->numBubbles = Read8bit((uint8_t)false);
	eventInfo->det2avg = Read16bit((uint8_t)false);
	eventInfo->det2startval = Read16bit((uint8_t)false);
	eventInfo->det2endval = Read16bit((uint8_t)false);
	eventInfo->det3avg = Read16bit((uint8_t)false);
	eventInfo->det3startval = Read16bit((uint8_t)false);	
	eventInfo->det3endval = Read16bit((uint8_t)false);
	
	//Check to make sure we can't overrun memory
	if(eventInfo->numBubbles > MAX_NUMBER_BUBBLES){
		eventInfo->numBubbles = 4;
		eventInfo->errorVal = eventInfo->errorVal | bdeMEMORY;
	}	
	for(uint8_t j = 0; j < eventInfo->numBubbles; j++){
		bdata[j].d2stime = Read32bit((uint8_t)false); 
		bdata[j].d2sval = Read16bit((uint8_t)false);
		bdata[j].d2etime= Read32bit((uint8_t)false); 
		bdata[j].d2eval = Read16bit((uint8_t)false);
		bdata[j].d3stime= Read32bit((uint8_t)false);
		bdata[j].d3sval = Read16bit((uint8_t)false);
		bdata[j].d3etime= Read32bit((uint8_t)false);
		bdata[j].d3eval = Read16bit((uint8_t)false);
	}
		
	Read8bit((uint8_t)true); //just do this to send stop bit

	rdAddr = rdAddr + 128; //increment read address to next page	
	//wrap the write address around to zero
	if(rdAddr >= 0x00020000){
		rdAddr = 0;
	}

	return 0;
}

uint32_t BDStorage::getRdAddr(){
	return rdAddr;
}

uint32_t BDStorage::getWrAddr(){
	return wrAddr;
}

uint32_t BDStorage::getDataStartAddr(){
	return dataStartAddr;
}

uint16_t BDStorage::getNumRecords(){
	return numRecords;
}

void BDStorage::deleteData(){

	uint8_t val;
	rdAddr = 0;
	while(rdAddr < 0x00020000){
		if(rdAddr > 0xFFFF)
			Wire.beginTransmission(DEV_ADDR_HIGH);
		else
			Wire.beginTransmission(DEV_ADDR_LOW);
		
		Wire.write((uint8_t)(rdAddr >> 8));    //MSB = most significant bits	
		Wire.write((uint8_t)(rdAddr & 0xFF));
		Wire.endTransmission();
		val = Read8bit((uint8_t) true);
		
		if(val != 0xFF){
			if(rdAddr > 0xFFFF)
				Wire.beginTransmission(DEV_ADDR_HIGH);
			else
				Wire.beginTransmission(DEV_ADDR_LOW);
			
			Wire.write((uint8_t)(rdAddr >> 8));    //MSB = most significant bits	
			Wire.write((uint8_t)(rdAddr & 0xFF));  //LSB = least " "			
			Write8bit(0xFF);
			Wire.endTransmission(true);
			delay(5);
		}
		rdAddr += 128;
	}
	dataStartAddr = wrAddr;
	numRecords = 0;
}

void BDStorage::resetRead(){
	rdAddr = dataStartAddr;
}

// Private Methods
void BDStorage::Write8bit(uint8_t data)
{
  Wire.write(data);
}

void BDStorage::Write16bit(uint16_t data)
{
  Wire.write(data >> 8);
  Wire.write(data);
}
 
void BDStorage::Write32bit(uint32_t data)
{
  Wire.write(data >> 24);
  Wire.write(data >> 16);
  Wire.write(data >> 8);
  Wire.write(data);
}

uint8_t BDStorage::Read8bit(uint8_t sendStop)
{
  uint8_t mydata = 0;
  uint8_t bytesReceived = 0;
  uint8_t devAddr;
  if(rdAddr > 0xFFFF)
		devAddr = DEV_ADDR_HIGH;
	else
		devAddr = DEV_ADDR_LOW;
  bytesReceived = Wire.requestFrom(devAddr, (uint8_t)1, (sendStop));
  while (Wire.available())
  {
    mydata = Wire.read();
  } //end of while loop
  return mydata;
}

uint16_t BDStorage::Read16bit(uint8_t sendStop)
{
  uint16_t mydata = 0;
  uint8_t bytesReceived = 0;
  uint8_t c = 0; 

	uint8_t devAddr;
	if(rdAddr > 0xFFFF)
		devAddr = DEV_ADDR_HIGH;
	else
		devAddr = DEV_ADDR_LOW;
  
  //read the 2 bytes from eeprom memory
  bytesReceived = Wire.requestFrom(devAddr, (uint8_t)2, sendStop);
  while (Wire.available())
  {
    c = Wire.read();
    mydata = (mydata << 8) + c;
  } //end of while loop
  return mydata;
}

uint32_t BDStorage::Read32bit(uint8_t sendStop)
{
  uint32_t mydata = 0;
  uint8_t bytesReceived = 0;
  uint8_t c = 0; 

  uint8_t devAddr;
  if(rdAddr > 0xFFFF)
		devAddr = DEV_ADDR_HIGH;
	else
		devAddr = DEV_ADDR_LOW;  
  
  //read the 4 bytes from eeprom memory
  bytesReceived = Wire.requestFrom(devAddr, (uint8_t)4, sendStop);
  while (Wire.available())
  {
    c = Wire.read();
    mydata = (mydata << 8) + c;
  } //end of while loop
  return mydata;
}
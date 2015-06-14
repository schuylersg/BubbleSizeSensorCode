/**
Description:	Header file for all constants and definitions for
				bubble detector code.

Author: 		Schuyler Senft-Grupp
Version: 		1.1
Date: 			5/15/2013

**/

#ifndef __BUBBLE_DETECTOR__
#define __BUBBLE_DETECTOR__

#include <Arduino.h>

const uint8_t MAX_NUMBER_BUBBLES = 4;  //The maximum number of bubbles that can be stored at the same time.
const uint8_t NUM_BKGD_POINTS = 8;	//the number of background points to store

//bde stands for bubble detector error
const uint8_t bdeTIMEOUT =         0b00000001;
const uint8_t bdeDET2MOREBUBBLES = 0b00000010;
const uint8_t bdeDET3MOREBUBBLES = 0b00000100;
const uint8_t bdeDET1NOEND =       0b00001000;
const uint8_t bdeDET2NOEND =       0b00010000;
const uint8_t bdeDET3NOEND =       0b00100000;
const uint8_t bdeFALSESTART =      0b01000000;
const uint8_t bdeMEMORY =      		 0b10000000;

const uint8_t bd2start = 1;
const uint8_t bd2end = 2;
const uint8_t bd3start = 3;
const uint8_t bd3end = 4;
const uint8_t bdError = 5;


/*************************************************
Declare structs to hold background measurements for each sensor
These use continuous ring buffers 
*************************************************/
struct backgrounddata{
  uint16_t rb [NUM_BKGD_POINTS]; //ring buffer for data
  uint8_t pos;      		//pos in the ring buffer - 0 to 7
  uint16_t minvalue;  		//minimum value in the ring buffer
  uint16_t maxvalue;  		//maximum value in the ring buffer
  uint32_t total;    		//the sum of all values in the ring buffer - divide by NUM_BKGD_POINTS to get avg value
  uint16_t startdetvalue;	//the value required to signify a bubble start event
  uint16_t enddetvalue;     //the value required to signify a bubble end event
};

/*************************************************
Declare structs to hold event information
*************************************************/
struct eventinfo{
  uint32_t eventTime;
  uint8_t numBubbles; 				
  uint8_t errorVal;
  uint16_t det2avg;
  uint16_t det2startval;
  uint16_t det2endval;  
  uint16_t det3avg;
  uint16_t det3startval;
  uint16_t det3endval;  
};

/*************************************************
Declare structs to hold bubble information
*************************************************/
struct bubbledata{
  uint32_t d2stime;
  uint16_t d2sval;
  uint32_t d2etime; 
  uint16_t d2eval;
  uint32_t d3stime; 
  uint16_t d3sval;
  uint32_t d3etime; 
  uint16_t d3eval;
};


#endif

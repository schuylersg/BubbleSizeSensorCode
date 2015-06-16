/**
Description:	Header file for all constants and definitions for
				bubble detector code.

Author: 		Schuyler Senft-Grupp
Version: 		2.0
Date: 			6/15/2015

**/

#ifndef __BUBBLE_DETECTOR__
#define __BUBBLE_DETECTOR__

#include <Arduino.h>

//DEFINE THE DIFFERENT MESSAGE TYPES - they can be values between 1 and 254 - 0 and 255 are reserved
#define MSG_EVENT_START 1
#define MSG_EVENT_END 	2
#define MSG_DET1_START	3
#define MSG_DET2_START	4
#define MSG_DET3_START	5
#define MSG_DET1_END		6
#define MSG_DET2_END		7
#define MSG_DET3_END		8	
#define MSG_ERROR				9

//DEFINE THE SIZE OF EACH MESSAGE (in bytes)
#define SIZE_EVENT_START 	10 //one uint32_t for millis and three uint16_t for the avg detector values
#define SIZE_EVENT_END 		4	 //time just before sleep (1 uint32_t) millis()
#define SIZE_DET1_START		4	 //time
#define SIZE_DET2_START		4	 //time
#define SIZE_DET3_START		4	 //time
#define SIZE_DET1_END			4  //time
#define SIZE_DET2_END			4  //time
#define SIZE_DET3_END			4  //time
#define SIZE_ERROR				1	 //8-bit error code

#define NUM_BKGD_POINTS  8 	//the number of background points to store

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
Declare struct to hold background measurements for each sensor
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

#endif

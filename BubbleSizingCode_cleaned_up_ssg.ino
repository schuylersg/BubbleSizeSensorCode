/****************************************************************************************
Title: Bubble Sizing Detector Firmware 
Version: 1.0
Date: PICK SOME DATE - MAYBE DATE OF RELEASE

Description: This code is the firmware for the bubble size detector.

Authors: Schuyler Senft-Grupp (skysg@alum.mit.edu) and Kyle Delwiche

License: KYLE - YOU SHOULD PICK ONE TO USE

 *****************************************************************************************/


/****************************************************************************************
 * LIBRARIES - For instructions on how to install libraries, see http://arduino.cc/en/Guide/Libraries
 *****************************************************************************************/
#include <Wire.h>                            // Arduino library for I2C communications
#include <digitalWriteFast.h>                // WEBSITE LINK TO FILE
#include <BubbleDetector.h>                  // WEBSITE LINK TO FILE
#include <BDStorage.h>                       // WEBSITE LINK TO FILE

/****************************************************************************************
 * PIN DECLARATIONS
 *****************************************************************************************/
const uint8_t ledOne = 4;                    // Detector 1 LED
const uint8_t ledTwo = 3;                    // Detector 2 LED
const uint8_t ledThree = 2;                  // Detector 3 LED
const uint8_t detOne = A0;                   // Detector 1 ADC input
const uint8_t detTwo = A1;                   // Detector 2 ADC input
const uint8_t detThree = A2;                 // Detector 3 ADC input

//A4 and A5 are needed for I2C communication with memory chip
//See Eagle file for required circuitry FILE LINK XXXXXXXXXXX

/****************************************************************************************
 * GLOBAL CONSTANTS
 *****************************************************************************************/
const uint8_t LOOK_FOR_START = 0;
const uint8_t LOOK_FOR_END = 1;

const unsigned long MAX_TIMEOUT = 10000000;  // Maximum time allowed for data-collection sequence

const uint16_t MIN_THRESHOLD = 30;	     // The minimum change in ADC value required to trigger data collection sequence

const uint8_t NOISE_MULTIPLILER = 2;         // The amount to multiply the difference betwen max/min backgound readings
                                             // for estimate of background noise threshold

const uint16_t BKGD_STORAGE = 1000;          // Store every BKGD_STORAGE ADC reading in background readings 

/****************************************************************************************
 * GLOBAL VARIABLE DECLARATIONS
 *****************************************************************************************/
unsigned long startOfEvents;          //WHAT IS THIS? - SSG: SEEMS LIKE IT'S JUST LEFTOVER AND CAN BE DELETED
unsigned long timeoutClockStart = 0;

//Each of the detectors has a state variable. The detector is always either 
//waiting to see the start of a bubble (LOOK_FOR_START) or waiting to see 
//the end of a bubble (LOOK_FOR_END).
uint8_t detOneState = LOOK_FOR_START;
uint8_t detTwoState = LOOK_FOR_START;
uint8_t detThreeState = LOOK_FOR_START;

//Structures defined in BubbleDetector.h file 
//They store estimates of background readings and noise
eventinfo eventInfo;                   // A struct to store information about bubble event
bubbledata bData [MAX_NUMBER_BUBBLES]; // An array of structs to store bubble data
backgrounddata detOneBkgd;             // A struct with information on detector 1 background ADC value
backgrounddata detTwoBkgd;             // A struct with information on detector 2 background ADC value
backgrounddata detThreeBkgd;           // A struct with information on detector 3 background ADC value
uint8_t detTwoNumBubbles;              // Number of bubbles Detector 2 sees during a bubbling event
uint8_t detThreeNumBubbles;            // Number of bubbles Detector 3 sees during a bubbling event
uint8_t mostBubblesSeen;               // The higher of either detTwoNumBubbles or detThreeNumBubbles
uint16_t bkgdCounter;                  // Variable to keep track of when to record background ADC reading

/****************************************************************************************
 * HELPER VARIABLES - these are generic variables but
 * declaring them here gives better estimate of RAM usage
 *****************************************************************************************/
uint16_t adcReading = 0;
uint8_t i;                              // Used for counting in loops
uint8_t numBubblesInTube;               // Used to keep track of number of bubbles in tube at once
uint8_t nextState;                      // Will either be LOOK_FOR_START or LOOK_FOR_END
uint32_t tempTime;

/****************************************************************************************
 * PERFORM SETUP TASKS
 *****************************************************************************************/
void setup(){
  // Initialize pins
  // DELETE THIS: It might be, that for power reasons these should be kept as inputs until they need
  // DELETE THIS: to be used as outputs - check documentation
  pinMode(ledOne, OUTPUT);
  pinMode(ledTwo, OUTPUT);
  pinMode(ledThree, OUTPUT);
  pinMode(detOne, INPUT);
  pinMode(detTwo, INPUT);
  pinMode(detThree, INPUT);

  // uncomment if pin 3 on memory chip is not hard wired to 5V 
  // pinMode(13, OUTPUT);
  // digitalWrite(13, HIGH);

  //setup serial for debugging purposes
  Serial.begin(115200);               // Baud communication rate
  
  analogReference(INTERNAL);          // Use the built-in reference voltage 
  InitializeBkgdStructs();            // Initialize background data structs.
  bkgdCounter = 0;                    // This counter increments every time detector 1 does not sense a bubble. 

  //Text to appear at the beginning of serial communication
  Serial.print(bdstorage.begin());    // Number of data points stored on EEPROM chip
  Serial.println(" entries found in memory");
  Serial.println("Hit any key to enter menu:");
  Serial.println("Logging");
}

/****************************************************************************************
 * BEGIN LOOKING FOR BUBBLES
 *****************************************************************************************/
void loop(){                                   // Create loop that checks if a bubble has entered glass tube

  detOneState = LOOK_FOR_START;                // Detector 1 is initially looking for start of bubble

  //clear start and end time arrays after each detection event
  for(i = 0; i < MAX_NUMBER_BUBBLES; i++){
    bData[i].d2stime = 0;
    bData[i].d2sval  = 0;
    bData[i].d2etime = 0;
    bData[i].d2eval  = 0;
    bData[i].d3stime = 0;
    bData[i].d3sval  = 0;
    bData[i].d3etime = 0;
    bData[i].d3eval  = 0;
  }

  while(detOneState == LOOK_FOR_START){        // Before the first bubble enters the tube

    delay(1);                                  // Delay could be replaced by custom sleep function to preserve battery life

    if(Serial.available() > 0){                // Allows for communication between Arduino and computer if connected
      talkToComputer();                        // talkToComputer defined below
    }

    //Take ADC reading 
    digitalWriteFast(ledOne, HIGH);            // Turn on the LED for detector 1

    //Check if it is time to take a background reading and if so turn on the other detector LEDs
    if(bkgdCounter == BKGD_STORAGE){
      digitalWriteFast(ledTwo, HIGH);          // Turn on the LED for detector 2
    }
    //this is a throw away reading that takes ~100 microseconds
    analogRead(detOne);                        // WHAT DO I SAY ABOUT THIS???  MAYBE DELETE IT, BUT WILL NEED TO VERIFY THAT ADC READINGS ARE STILL ACCURATE.  OR, SAY THIS IS NECEWSSARY BECAUSE FIRST ADC READING IS SOMETIMES WRONG. IF KEEPING IT, TRY MOVING IT FARTHER DOWN CODE
    SSG: LEAVE IT IN, BUT I THINK THE CURRENT COMMENT ABOVE IT IS EXPLANATORY ENOUGH

    if(bkgdCounter == BKGD_STORAGE){
      digitalWriteFast(ledThree, HIGH);        // Turn on the LED for detector 3
    }

    delayMicroseconds(200);                    // Delay necessary to allow photocells to stabilize
    adcReading = analogRead(detOne);           // Read the ADC for detector 1
    digitalWriteFast(ledOne, LOW);             // Turn off the LED for detector 1

    // Compare ADC reading to background readings.  If sensor determines there is a bubble in the tube, detOneState will switch to LOOK_FOR_END
    detOneState = CheckForBubble(&detOneBkgd, adcReading, LOOK_FOR_START);  

    // If no bubble has been detected (detOneState is still LOOK_FOR_START), check if it is time to store a background reading
    if(bkgdCounter >= BKGD_STORAGE && detOneState == LOOK_FOR_START){
      //Read each detector and update its background value
      UpdateBkgd(&detOneBkgd, adcReading);     
      adcReading = analogRead(detTwo);         
      UpdateBkgd(&detTwoBkgd, adcReading);     
      digitalWriteFast(ledTwo, LOW);           
      adcReading = analogRead(detThree);          
      UpdateBkgd(&detThreeBkgd, adcReading);   
      digitalWriteFast(ledThree, LOW);         
      bkgdCounter = 0;                         // Reset background counter
      analogRead(detOne);  		       // Throw away reading for detector one    
    }

    bkgdCounter = bkgdCounter + 1;             // Increment background counter
  }

  /****************************************************************************************
   * A BUBBLE HAS NOW BEEN DETECTED ENTERING THE GLASS TUBE
   *****************************************************************************************/

  eventInfo.eventTime = millis();            // Begin by recording the event time

  // Turn on all three LEDs 
  digitalWriteFast(ledOne, HIGH);           
  digitalWriteFast(ledTwo, HIGH);  
  digitalWriteFast(ledThree, HIGH);
  delayMicroseconds(100);                    // Delay necessary to allow phototransistors to stabilize (ABOVE WE DO 200 MICROSECONDS, WHY NOT 100 HERE TOO?) - SSG: HRMMM THEY SHOULD PROBABLY BE THE SAME

  numBubblesInTube = 1;                      // Initially set to 1 because the first bubble has been detected 

  // Initialize detTwo and detThree state and the number
  // of bubbles they've seen
  detTwoState        = LOOK_FOR_START;
  detThreeState      = LOOK_FOR_START;
  detTwoNumBubbles   = 0;
  detThreeNumBubbles = 0;
  mostBubblesSeen    = 0;

  // Get the current time so that we know when to timeout the operation of looking for a bubble
  timeoutClockStart = micros();  
  tempTime = timeoutClockStart;

  //DO I STILL NEED THESE??  HOW DO I JUSTIFY THEM?? SSG: I don't know if they are still necessary but leave it in - the following comment does a fine job of explaining it
  analogRead(detOne);                        // Need to throw out 1st ADC reading, it tends to be noisy. (also adds a small delay for everything to stabilize)
  analogRead(detTwo);
  analogRead(detThree);

  while ((numBubblesInTube > 0) && (tempTime - timeoutClockStart < MAX_TIMEOUT)) {
    
    // Take ADC reading from detector 1 to see if another bubble has entered the glass tube 
    adcReading = analogRead(detOne);       // Read the ADC for detector 1
    nextState  = CheckForBubble(&detOneBkgd, adcReading, detOneState);

    // If another bubble is detected, update the number of bubbles in the tube
    if(detOneState == LOOK_FOR_START && nextState == LOOK_FOR_END){ 
      numBubblesInTube += 1;                
    }
    detOneState = nextState;                // Update detector 1 status

    //  Take ADC reading from detector 2.  At this detector we want to capture both the beginning and end of the bubble
    tempTime   = micros();                  // Record current time
    adcReading = analogRead(detTwo);        // Read the ADC for detector 2
    nextState  = CheckForBubble(&detTwoBkgd, adcReading, detTwoState);

    // If detector 2 sees the start of a bubble, record the current time
    if(detTwoState == LOOK_FOR_START && nextState == LOOK_FOR_END){
      bData[detTwoNumBubbles].d2stime = tempTime;
      bData[detTwoNumBubbles].d2sval  = adcReading;
    }

    // If detector 2 sees the end of a bubble, record the current time
    if(detTwoState == LOOK_FOR_END && nextState == LOOK_FOR_START){
      bData[detTwoNumBubbles].d2etime = tempTime;
      bData[detTwoNumBubbles].d2eval  = adcReading;
      detTwoNumBubbles += 1;                // Increment total number of bubbles detector 2 has seen
    } 
    detTwoState = nextState;                // Update detector 2's status
    

    //Take a measurement for detector 3.  This is almost the same as detector 2.
    tempTime   = micros();                  // Record current time
    adcReading = analogRead(detThree);      // Read the ADC for detector 3
    nextState  = CheckForBubble(&detThreeBkgd, adcReading, detThreeState);  

    // If detector 3 sees the start of a bubble, record the current time
    if(detThreeState == LOOK_FOR_START && nextState == LOOK_FOR_END){
      bData[detThreeNumBubbles].d3stime = tempTime;
      bData[detThreeNumBubbles].d3sval  = adcReading;
    }
   
   // If detector 3 sees the end of a bubble, record the current time
    if(detThreeState == LOOK_FOR_END && nextState == LOOK_FOR_START){
      bData[detThreeNumBubbles].d3etime = tempTime;
      bData[detThreeNumBubbles].d3eval  = adcReading;
      detThreeNumBubbles += 1;
      numBubblesInTube   -= 1;              // Decrease total number of bubbles in the tube
    }
    detThreeState = nextState;              // Update detector 3's status

    //Error checking to make sure we don't overflow the maximum number of bubbles that can be counted
    if(detTwoNumBubbles == MAX_NUMBER_BUBBLES || detThreeNumBubbles == MAX_NUMBER_BUBBLES){
      timeoutClockStart = 0;                // This will cause while loop to exit
    }
  }

/****************************************************************************************
 * STORE DATA
 *****************************************************************************************/
  //No more bubbles in tube, so save the data we have
  //or transmit if in debug mode.

  // First turn off all LEDs
  digitalWriteFast(ledOne, LOW);
  digitalWriteFast(ledTwo, LOW);
  digitalWriteFast(ledThree, LOW);

  // Keep track of errors
  eventInfo.errorVal = 0;
  //?????WHERE IS bdeMEMORY =      0b10000000 STORED?????
  //???? CAN I REARRANGE THE ERROR CODES BELOW SO THEY ARE IN SEQUENCE?????
  SSG: I think all the error codes are in the bubble detector .h file - also we do not need to include the code
  in the comment - that's the point of using the constants like bdeTIMEOUT. But if you want to reorder, there
  shouldn't be a problem
  
  // Error: Sensor timed out before finishing data collection sequence, 00000001
  if((tempTime - timeoutClockStart > MAX_TIMEOUT)){
    eventInfo.errorVal = bdeTIMEOUT;
  }
 // Error: Neither detector 2 or 3 saw a bubble, 01000000
  if(detTwoNumBubbles == 0 && detThreeNumBubbles == 0){
    eventInfo.errorVal = eventInfo.errorVal | bdeFALSESTART;
  }

  // Error: Detector 1 is still looking for the end of a bubble, 00001000
  if(detOneState == LOOK_FOR_END){
    eventInfo.errorVal = eventInfo.errorVal | bdeDET1NOEND;
  }

  // Error: Detector 2 is still looking for the end of a bubble, 00010000
  if(detTwoState == LOOK_FOR_END){
    eventInfo.errorVal = eventInfo.errorVal | bdeDET2NOEND;
  }

  // Error: Detector 3 is still looking for the end of a bubble, 00100000  
  if(detThreeState == LOOK_FOR_END){
    eventInfo.errorVal = eventInfo.errorVal | bdeDET3NOEND;
  }

  // Error: Detector 2 saw more bubbles than detector 3, 00000010
  if(detTwoNumBubbles > detThreeNumBubbles){
    eventInfo.errorVal = eventInfo.errorVal | bdeDET2MOREBUBBLES;
  } 

  // Error: Detector 3 saw more bubbles than detector 2, 00000100
  if(detTwoNumBubbles < detThreeNumBubbles){
    eventInfo.errorVal = eventInfo.errorVal | bdeDET3MOREBUBBLES;
  } 

  eventInfo.numBubbles =  max(detTwoNumBubbles, detThreeNumBubbles);  // Maximum number of bubbles seen by detectors 2 and 3
  eventInfo.det2avg = detTwoBkgd.total/NUM_BKGD_POINTS;               // Average background ADC reading for detector 2 during bubble event
  eventInfo.det2startval = detTwoBkgd.startdetvalue;                  // ADC value that triggered beginning of detector 2 data collection sequence
  eventInfo.det2endval = detTwoBkgd.enddetvalue;                      // ADC value that triggered end of detector 2 data collection sequence
  eventInfo.det3avg = detThreeBkgd.total/NUM_BKGD_POINTS;             // Average background ADC reading for detector 3 during bubble event
  eventInfo.det3startval = detThreeBkgd.startdetvalue;                // ADC value that triggered beginning of detector 3 data collection sequence
  eventInfo.det3endval = detThreeBkgd.enddetvalue;                    // ADC value that triggered end of detector 3 data collection sequence

  //Save the data to the EEPROM chip
  bdstorage.saveBubbleEvent(&eventInfo, bData);

  //For debugging purposes, data can be printed while the Arduino is connected to a computer.  
  //For field deployment, keep this commented out.
  PrintData();              // PrintData() defined BELOW

}  // end of loop()


/****************************************************************************************
 * CHECK TO SEE IF DETECTOR SEES THE BEGINNING OR END OF A BUBBLE
 *****************************************************************************************/
uint8_t CheckForBubble(backgrounddata* bkgd, uint16_t newValue, uint8_t state){
  if(state == LOOK_FOR_START){        // While the detector is looking for the start of a bubble
    if(newValue < bkgd->startdetvalue){
      return LOOK_FOR_END;            // A new bubble has been detected
    }
    else{
      return LOOK_FOR_START;          // No bubble has been detected
    }
  }
  else{
    if(newValue > bkgd->enddetvalue){ // While the detector is looking for the end of a bubble
      return LOOK_FOR_START;          // End of bubble has been detected
    }
    else{
      return LOOK_FOR_END;            // Detector still looking for end of bubble
    }
  }
}

/****************************************************************************************
 * UPDATE MAX, MIN, AND DETECTION VALUE OF THE BACKGROUNDDATA STRUCT
 *****************************************************************************************/
void UpdateBkgd(backgrounddata * bkgd, uint16_t newValue){

  // Update the total value
  bkgd->total = bkgd->total - bkgd->rb[bkgd->pos] + newValue;

  // Store the new value
  bkgd->rb[bkgd->pos] = newValue;

  // Update the pos value
  bkgd->pos = bkgd->pos + 1;
  if(bkgd->pos == NUM_BKGD_POINTS){
    bkgd->pos  = 0;
  }

  // Update max/min and total values of ringbuffer 
  bkgd->maxvalue = 0;
  bkgd->minvalue = 1024;  // Maximum value of 10 bit ADC

  for (uint8_t j = 0; j < NUM_BKGD_POINTS; j++){
    if(bkgd->rb[j] > bkgd->maxvalue){
      bkgd->maxvalue = bkgd->rb[j];
    }
    if(bkgd->rb[j] < bkgd->minvalue){
      bkgd->minvalue = bkgd->rb[j];
    }

    // Pick the more conservative noise threshold
    uint16_t noiseThreshold = max((bkgd->maxvalue - bkgd->minvalue) * NOISE_MULTIPLILER, 
    MIN_THRESHOLD);  

    // If the threshold is greater than the average value, then detection is impossible
    if((bkgd->total/NUM_BKGD_POINTS) < noiseThreshold){
      bkgd->startdetvalue = 0;
      bkgd->enddetvalue   = 1023;       // 2^10-1
    }
    else{
      // Pick the more conservative detection value
      bkgd->startdetvalue = (bkgd->total/NUM_BKGD_POINTS) - noiseThreshold;

      // The enddetvalue is arbitrarily larger than the startdetvalue by half the noise threshold.
      // This should add significant hysteresis so that a bubble is not instantly detected and then "undetected"
      bkgd->enddetvalue = bkgd->total/NUM_BKGD_POINTS - 20;  //???????????HOW DO WE JUSTIFY THAT 20???
    }
  }
}

/****************************************************************************************
 * INITIALIZE THE BACKGROUNDDATA STRUCTS USED IN THE PROGRAM
 *****************************************************************************************/
//This program must be called in setup()
void InitializeBkgdStructs(){
  for (i = 0; i < NUM_BKGD_POINTS; i++){
    detOneBkgd.rb[i] = 0;
    detTwoBkgd.rb[i] = 0;
    detThreeBkgd.rb[i] = 0; 
  }
  detOneBkgd.pos = 0;
  detTwoBkgd.pos = 0;
  detThreeBkgd.pos = 0;
  UpdateBkgd(&detOneBkgd, 0);
  UpdateBkgd(&detTwoBkgd, 0);
  UpdateBkgd(&detThreeBkgd, 0);
}

/****************************************************************************************
 * COMMUNICATION BETWEEN ARDUINO AND COMPUTER
 *****************************************************************************************/
//This program must be called in setup()
void talkToComputer(){
  char c = Serial.read();                 // Throw out character that got into communicate mode (???????What is this????????)
  uint32_t startTime = millis();          // initialize startTime which is used to return to logging after a period of inactivity
  boolean talkingToComputer = true;
  Serial.println("Menu");
  Serial.print(bdstorage.getNumRecords());
  Serial.println(" Events Stored");
  bdstorage.resetRead();                  // Reset the data read address when entering menu
  Serial.println("A: All    H: Header   N: Next  R: Reset\nX: Delete E: Exit");
  while(talkingToComputer){

    if(Serial.available() > 0){
      c = Serial.read();
      startTime = millis();
    }
    else if(millis() - startTime > 60000){// Go back to logging after minute of inactivity
      c = 'E';                            // E to exit the talking to computer state
    }
    else{
      c = ' '; 
    }

    if(c == 'A'){                         // If user enters 'A', read the data and print to screen
      bdstorage.resetRead();
      uint16_t numR = bdstorage.getNumRecords(); 
      for (uint16_t t = 0; t < numR; t++){
        bdstorage.readEvent(&eventInfo, bData);
        PrintData();                      // PrintData() defined below
      }
    }
    else if(c == 'E'){                    // If user enters 'E', exit the talking to computer state and return to logging
      talkingToComputer = false;
    }
    else if(c == 'H'){                    // If user enters 'H', display data column headers
      Serial.println("ms,Bubbles,Error,D2 Back,D2 Det,D2 End,D3 Back,D3 Det,D3 End,D2s,D2val,D2e,d2val,D3s,D3val,D3e,d3val");
    }
    else if(c == 'N'){                    // If user enters 'N', display next line of data
      int r = bdstorage.readEvent(&eventInfo, bData);
      if(r==0){
        PrintData();                      // PrintData() defined farther down
      }
      else{
        Serial.println("End of Data");
      }
    }
    else if(c == 'R'){                    // If user enters 'R',  reset N so it displays first line of data
      bdstorage.resetRead(); 
    }
    else if (c == 'X'){                   // If user enters 'X', delete all data upon verification
      Serial.println("Delete?(Y/N)");     // Verify user wants to delete all data, Y = yes and N = no
      while(Serial.available() == 0){
      }
      c = Serial.read();
      if(c == 'Y'){
        Serial.print("Deleting...");
        bdstorage.deleteData();
        Serial.println("Done");
      }
    } 
    else if(c == 'W'){                     
      Serial.print("WrA: ");
      Serial.println(bdstorage.getWrAddr(), HEX);
      Serial.print("StartAddr: ");
      Serial.println(bdstorage.getDataStartAddr(), HEX);
      Serial.print("RdA: ");
      Serial.println(bdstorage.getRdAddr(), HEX);
    } //end else if
  }   //end while
  Serial.println("Logging");
}     //end talking to computer

/****************************************************************************************
 * PRINTING DATA TO COMPUTER DURING SERIAL CONNECTION
 *****************************************************************************************/

void PrintData(){
  Serial.print(eventInfo.eventTime);       // Time in microseconds of bubble event
  Serial.print(",");
  Serial.print(eventInfo.numBubbles);      // Number of bubbles recorded during bubble event
  Serial.print(",");
  Serial.print(eventInfo.errorVal, BIN);   // All error codes associated with bubbling event
  Serial.print(","); 
  Serial.print(eventInfo.det2avg);         // Background ADC value for detector 2 at time of bubble event
  Serial.print(",");
  Serial.print(eventInfo.det2startval);    // ADC threshold value that will trigger detector 2 data collection sequence
  Serial.print(",");
  Serial.print(eventInfo.det2endval);      // ADC threshold value that will end detector 2 data collection sequence
  Serial.print(","); 
  Serial.print(eventInfo.det3avg);         // Background ADC value for detector 3 at time of bubble event
  Serial.print(",");
  Serial.print(eventInfo.det3startval);    // ADC threshold value that will trigger detector 3 data collection sequence
  Serial.print(",");
  Serial.print(eventInfo.det3endval);      // ADC threshold value that will end detector 3 data collection sequence
  Serial.print(",");
  for (i = 0; i < eventInfo.numBubbles; i ++){
    if(i > 0)
      Serial.print(",,,,,,,,,");
    Serial.print(bData[i].d2stime);        // Bubble start time for detector 2
    Serial.print(",");
    Serial.print(bData[i].d2sval);         // ADC value that triggers beginning of detector 2 data collection sequence
    Serial.print(",");
    Serial.print(bData[i].d2etime);        // Bubble end time for detector 2
    Serial.print(",");
    Serial.print(bData[i].d2eval);         // ADC value that triggers end of detector 2 data collection sequence
    Serial.print(","); 
    Serial.print(bData[i].d3stime);        // Bubble start time for detector 3
    Serial.print(",");
    Serial.print(bData[i].d3sval);         // ADC value that triggers beginning of detector 3 data collection sequence
    Serial.print(",");
    Serial.print(bData[i].d3etime);        // Bubble end time for detector 3
    Serial.print(",");
    Serial.println(bData[i].d3eval);       // ADC value that triggers end of detector 3 data collection sequence
  }  
}







/****************************************************************************************
Title: Bubble Detector Firmware
Version: 2.0
Date: 6/15/2015

Description: This code is the firmware for the bubble size detector.

Authors: Schuyler Senft-Grupp (skysg@alum.mit.edu) and Kyle Delwiche

License: KYLE - YOU SHOULD PICK ONE TO USE

 *****************************************************************************************/


/****************************************************************************************
 * LIBRARIES - For instructions on how to install libraries, see http://arduino.cc/en/Guide/Libraries
 *****************************************************************************************/
#include <BDDataLogger.h>
#include <BubbleDetector.h>                  // WEBSITE LINK TO FILE

#define HW_VERSION "3.0"
#define FW_VERSION "2.0"

/****************************************************************************************
 * PIN DECLARATIONS
 *****************************************************************************************/
#define LED_1_PORT PORTD
#define LED_2_PORT PORTD
#define LED_3_PORT PORTD
#define LED_1_BIT 4
#define LED_2_BIT 3
#define LED_3_BIT 2

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

const unsigned long MAX_TIMEOUT = 3000000;  // Maximum time allowed for data-collection sequence in microseconds

const uint16_t MIN_THRESHOLD = 30;	     // The minimum change in ADC value required to trigger data collection sequence

const uint8_t NOISE_MULTIPLILER = 2;         // The amount to multiply the difference betwen max/min backgound readings
// for estimate of background noise threshold

const uint16_t BKGD_STORAGE = 1000;          // Store every BKGD_STORAGE ADC reading in background readings

/****************************************************************************************
 * GLOBAL VARIABLE DECLARATIONS
 *****************************************************************************************/
unsigned long eventStartTime;        //stores time (millis) when event begins
unsigned long timeoutClockStart = 0;
unsigned long timeOfLastTimeout = 0;
uint8_t numberTimeouts = 0;
const uint8_t MAX_NUM_TIMEOUTS = 5;

//Each of the detectors has a state variable. The detector is always either
//waiting to see the start of a bubble (LOOK_FOR_START) or waiting to see
//the end of a bubble (LOOK_FOR_END).
uint8_t detOneState = LOOK_FOR_START;
uint8_t detTwoState = LOOK_FOR_START;
uint8_t detThreeState = LOOK_FOR_START;

//Structures defined in BubbleDetector.h file
//They store estimates of background readings and noise
backgrounddata detOneBkgd;             // A struct with information on detector 1 background ADC value
backgrounddata detTwoBkgd;             // A struct with information on detector 2 background ADC value
backgrounddata detThreeBkgd;           // A struct with information on detector 3 background ADC value
uint8_t detTwoNumBubbles;              // Number of bubbles Detector 2 sees during a bubbling event
uint8_t detThreeNumBubbles;            // Number of bubbles Detector 3 sees during a bubbling event
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
void setup() {
  //set LED pins to outputs
  sbi(DDRD, LED_1_BIT);
  sbi(DDRD, LED_2_BIT);
  sbi(DDRD, LED_3_BIT);
  
  //set detector pins as inputs
  pinMode(detOne, INPUT);
  pinMode(detTwo, INPUT);
  pinMode(detThree, INPUT);

  //setup serial for debugging purposes
  Serial.begin(115200);               // Baud communication rate
  PrintName();
  Serial.println("Parsing EEPROM...");  //this can take up to 30 seconds
  delay(100);

  analogReference(INTERNAL);          // Use the built-in reference voltage
  InitializeBkgdStructs();            // Initialize background data structs.
  bkgdCounter = 0;                    // This counter increments every time detector 1 does not sense a bubble.

  //Start EEPROM data logger
  DL_Initialise();
  DL_Write3(MSG_ERROR, BDE_RESET);
  
  //Text to appear at the beginning of serial communication
  Serial.print((((uint32_t)DL_Page_Write_Addr()) << 8) + DL_Buffer_Wr_Pos());  // Number of data points stored on EEPROM chip
  Serial.println(" data bytes found in memory");
  Serial.println("Hit any key to enter menu");
  Serial.println("Logging");
}

/****************************************************************************************
 * BEGIN LOOKING FOR BUBBLES
 *****************************************************************************************/
void loop() {                                  // Create loop that checks if a bubble has entered glass tube

  detOneState = LOOK_FOR_START;                // Detector 1 is initially looking for start of bubble

  while (detOneState == LOOK_FOR_START) {      // Before the first bubble enters the tube

    delay(1);                                  // Delay could be replaced by custom sleep function to preserve battery life
    // but then we need to time it and add that to a different counter

    if (Serial.available() > 0) {              // Allows for communication between Arduino and computer if connected
      talkToComputer();                        // talkToComputer defined below
    }

    //Take ADC reading
    sbi(LED_1_PORT, LED_1_BIT);                // Turn on the LED for detector 1

    //Check if it is time to take a background reading and if so turn on the other detector LEDs
    if (bkgdCounter == BKGD_STORAGE) {
      sbi(LED_2_PORT, LED_2_BIT);;          // Turn on the LED for detector 2
    }
    //this is a throw away reading that takes ~100 microseconds
    analogRead(detOne);

    if (bkgdCounter == BKGD_STORAGE) {
      sbi(LED_3_PORT, LED_3_BIT);;        // Turn on the LED for detector 3
    }

    delayMicroseconds(200);                    // Delay necessary to allow photocells to stabilize
    adcReading = analogRead(detOne);           // Read the ADC for detector 1
    cbi(LED_1_PORT, LED_1_BIT);                // Turn off the LED for detector 1

    // Compare ADC reading to background readings.  If sensor determines there is a bubble in the tube, detOneState will switch to LOOK_FOR_END
    detOneState = CheckForBubble(&detOneBkgd, adcReading, LOOK_FOR_START);

    // If no bubble has been detected (detOneState is still LOOK_FOR_START), check if it is time to store a background reading
    if (bkgdCounter >= BKGD_STORAGE && detOneState == LOOK_FOR_START) {
      //Read each detector and update its background value
      UpdateBkgd(&detOneBkgd, adcReading);
      adcReading = analogRead(detTwo);
      UpdateBkgd(&detTwoBkgd, adcReading);
      cbi(LED_2_PORT, LED_2_BIT);
      adcReading = analogRead(detThree);
      UpdateBkgd(&detThreeBkgd, adcReading);
      cbi(LED_3_PORT, LED_3_BIT);
      bkgdCounter = 0;                         // Reset background counter
      analogRead(detOne);  		       // Throw away reading for detector one
    }

    bkgdCounter = bkgdCounter + 1;             // Increment background counter
  }

  /****************************************************************************************
   * A BUBBLE HAS NOW BEEN DETECTED ENTERING THE GLASS TUBE
   *****************************************************************************************/

  eventStartTime = millis();            // Begin by recording the event time
  //Store the time the event began and the background average values
  DL_Write2(MSG_EVENT_START, eventStartTime,
            (uint16_t)detOneBkgd.total / NUM_BKGD_POINTS,
            (uint16_t)detTwoBkgd.total / NUM_BKGD_POINTS,
            (uint16_t)detThreeBkgd.total / NUM_BKGD_POINTS);
  DL_Write1(MSG_DET1_START , micros());

  // Turn on all three LEDs
  PORTD |= (1 << LED_1_BIT) | (1 << LED_2_BIT) | (1 << LED_3_BIT);
  delayMicroseconds(100);                    // Delay necessary to allow phototransistors to stabilize

  numBubblesInTube = 1;                      // Initially set to 1 because the first bubble has been detected

  // Initialize detTwo and detThree state and the number
  // of bubbles they've seen
  detTwoState        = LOOK_FOR_START;
  detThreeState      = LOOK_FOR_START;
  detTwoNumBubbles   = 0;
  detThreeNumBubbles = 0;

  // Get the current time so that we know when to timeout the operation of looking for a bubble
  timeoutClockStart = micros();
  tempTime = timeoutClockStart;
  while ((numBubblesInTube > 0) && (tempTime - timeoutClockStart < MAX_TIMEOUT)) {

    // Take ADC reading from detector 1 to see if another bubble has entered the glass tube
    adcReading = analogRead(detOne);       // Read the ADC for detector 1
    nextState  = CheckForBubble(&detOneBkgd, adcReading, detOneState);

    // If another bubble is detected, update the number of bubbles in the tube
    if (detOneState == LOOK_FOR_START && nextState == LOOK_FOR_END) {
      DL_Write1(MSG_DET1_START , tempTime);
      numBubblesInTube += 1;
      timeoutClockStart = tempTime;  //any time there is a new bubble, restart the timeout clock
    }
    // If detector 1 sees the end of a bubble, record the current time
    if (detOneState == LOOK_FOR_END && nextState == LOOK_FOR_START) {
      DL_Write1(MSG_DET1_END , tempTime);
    }
    detOneState = nextState;                // Update detector 1 status

    //  Take ADC reading from detector 2.  At this detector we want to capture both the beginning and end of the bubble
    tempTime   = micros();                  // Record current time
    adcReading = analogRead(detTwo);        // Read the ADC for detector 2
    nextState  = CheckForBubble(&detTwoBkgd, adcReading, detTwoState);

    // If detector 2 sees the start of a bubble, record the current time
    if (detTwoState == LOOK_FOR_START && nextState == LOOK_FOR_END) {
      DL_Write1(MSG_DET2_START , tempTime);
    }

    // If detector 2 sees the end of a bubble, record the current time
    if (detTwoState == LOOK_FOR_END && nextState == LOOK_FOR_START) {
      DL_Write1(MSG_DET2_END , tempTime);
      detTwoNumBubbles += 1;                // Increment total number of bubbles detector 2 has seen
    }
    detTwoState = nextState;                // Update detector 2's status

    //Take a measurement for detector 3.  This is almost the same as detector 2.
    tempTime   = micros();                  // Record current time
    adcReading = analogRead(detThree);      // Read the ADC for detector 3
    nextState  = CheckForBubble(&detThreeBkgd, adcReading, detThreeState);

    // If detector 3 sees the start of a bubble, record the current time
    if (detThreeState == LOOK_FOR_START && nextState == LOOK_FOR_END) {
      DL_Write1(MSG_DET3_START , tempTime);
    }

    // If detector 3 sees the end of a bubble, record the current time
    if (detThreeState == LOOK_FOR_END && nextState == LOOK_FOR_START) {
      DL_Write1(MSG_DET3_END , tempTime);
      detThreeNumBubbles += 1;
      numBubblesInTube   -= 1;              // Decrease total number of bubbles in the tube
    }
    detThreeState = nextState;              // Update detector 3's status

    tempTime = micros();
    if (tempTime < timeoutClockStart) { //micros overflow
      timeoutClockStart = 0;  //just set to zero
    }
    
    //because this could infinite loop, allow a specific serial communication to break out of it
    if (Serial.available() > 0) {              // Allows for communication between Arduino and computer if connected
      char c = Serial.read();
      if (c == 'A')
        delay(1000);//delay 1 second to allow second character to be sent
        break;  //break out of while loop
    }
  }
  DL_Write1(MSG_EVENT_END, millis());
  
  /****************************************************************************************
   * STORE DATA
   *****************************************************************************************/
  //No more bubbles in tube, so save the data we have
  //or transmit if in debug mode.

  // First turn off all LEDs
  PORTD &= ~((1 << LED_1_BIT) | (1 << LED_2_BIT) | (1 << LED_3_BIT));

  //Check for any error conditions
  //Error: Timeout
  if ((tempTime - timeoutClockStart > MAX_TIMEOUT)) {
    DL_Write3(MSG_ERROR, BDE_TIMEOUT);
    if ((tempTime - timeOfLastTimeout) < (MAX_TIMEOUT*2)){
      numberTimeouts++;
    }else{
      numberTimeouts = 1;
    }
    timeOfLastTimeout = tempTime; 
  }
  if(numberTimeouts >= MAX_NUM_TIMEOUTS){
    DL_Write3(MSG_ERROR, TOOMANYTIMEOUTS);
    InitializeBkgdStructs();
    numberTimeouts = 0;
  }

  // Error: Neither detector 2 or 3 saw a bubble, 01000000
  if (detTwoNumBubbles == 0 && detThreeNumBubbles == 0) {
    DL_Write3(MSG_ERROR, BDE_FALSESTART);
  }

  // Error: Detector 1 is still looking for the end of a bubble, 00001000
  if (detOneState == LOOK_FOR_END) {
    DL_Write3(MSG_ERROR, BDE_DET1NOEND);
  }

  // Error: Detector 2 is still looking for the end of a bubble, 00010000
  if (detTwoState == LOOK_FOR_END) {
    DL_Write3(MSG_ERROR, BDE_DET2NOEND);
  }

  // Error: Detector 3 is still looking for the end of a bubble, 00100000
  if (detThreeState == LOOK_FOR_END) {
    DL_Write3(MSG_ERROR, BDE_DET3NOEND);
  }

  // Error: Detector 2 saw more bubbles than detector 3, 00000010
  if (detTwoNumBubbles > detThreeNumBubbles) {
    DL_Write3(MSG_ERROR, BDE_DET2MORE);
  }

  // Error: Detector 3 saw more bubbles than detector 2, 00000100
  if (detTwoNumBubbles < detThreeNumBubbles) {
    DL_Write3(MSG_ERROR, BDE_DET3MORE);
  }
  
  //Let data logger know it's a good time to save right now
  DL_Safe_To_Write();

}  // end of loop()


/****************************************************************************************
 * CHECK TO SEE IF DETECTOR SEES THE BEGINNING OR END OF A BUBBLE
 *****************************************************************************************/
uint8_t CheckForBubble(backgrounddata* bkgd, uint16_t newValue, uint8_t state) {
  if (state == LOOK_FOR_START) {      // While the detector is looking for the start of a bubble
    if (newValue < bkgd->startdetvalue) {
      return LOOK_FOR_END;            // A new bubble has been detected
    }
    else {
      return LOOK_FOR_START;          // No bubble has been detected
    }
  }
  else {
    if (newValue > bkgd->enddetvalue) { // While the detector is looking for the end of a bubble
      return LOOK_FOR_START;          // End of bubble has been detected
    }
    else {
      return LOOK_FOR_END;            // Detector still looking for end of bubble
    }
  }
}

/****************************************************************************************
 * UPDATE MAX, MIN, AND DETECTION VALUE OF THE BACKGROUNDDATA STRUCT
 *****************************************************************************************/
void UpdateBkgd(backgrounddata * bkgd, uint16_t newValue) {

  // Update the total value
  bkgd->total = bkgd->total - bkgd->rb[bkgd->pos] + newValue;

  // Store the new value
  bkgd->rb[bkgd->pos] = newValue;

  // Update the pos value
  bkgd->pos = bkgd->pos + 1;
  if (bkgd->pos == NUM_BKGD_POINTS) {
    bkgd->pos  = 0;
  }

  // Update max/min and total values of ringbuffer
  bkgd->maxvalue = 0;
  bkgd->minvalue = 1024;  // Maximum value of 10 bit ADC

  for (uint8_t j = 0; j < NUM_BKGD_POINTS; j++) {
    if (bkgd->rb[j] > bkgd->maxvalue) {
      bkgd->maxvalue = bkgd->rb[j];
    }
    if (bkgd->rb[j] < bkgd->minvalue) {
      bkgd->minvalue = bkgd->rb[j];
    }

    // Pick the more conservative noise threshold
    uint16_t noiseThreshold = max((bkgd->maxvalue - bkgd->minvalue) * NOISE_MULTIPLILER,
                                  MIN_THRESHOLD);

    // If the threshold is greater than the average value, then detection is impossible
    if ((bkgd->total / NUM_BKGD_POINTS) < noiseThreshold) {
      bkgd->startdetvalue = 0;
      bkgd->enddetvalue   = 1023;       // 2^10-1
    }
    else {
      // Pick the more conservative detection value
      bkgd->startdetvalue = (bkgd->total / NUM_BKGD_POINTS) - noiseThreshold;

      // The enddetvalue is arbitrarily larger than the startdetvalue by half the noise threshold.
      // This should add significant hysteresis so that a bubble is not instantly detected and then "undetected"
      bkgd->enddetvalue = bkgd->total / NUM_BKGD_POINTS - 20; //???????????HOW DO WE JUSTIFY THAT 20???
    }
  }
}

/****************************************************************************************
 * INITIALIZE THE BACKGROUNDDATA STRUCTS USED IN THE PROGRAM
 *****************************************************************************************/
//This program must be called in setup()
void InitializeBkgdStructs() {
  for (i = 0; i < NUM_BKGD_POINTS; i++) {
    detOneBkgd.rb[i] = 0;
    detTwoBkgd.rb[i] = 0;
    detThreeBkgd.rb[i] = 0;
  }
  detOneBkgd.pos = 0;
  detTwoBkgd.pos = 0;
  detThreeBkgd.pos = 0;
  detOneBkgd.total = 0;
  detTwoBkgd.total = 0;
  detThreeBkgd.total = 0;
  UpdateBkgd(&detOneBkgd, 0);
  UpdateBkgd(&detTwoBkgd, 0);
  UpdateBkgd(&detThreeBkgd, 0);
}

/****************************************************************************************
 * COMMUNICATION BETWEEN ARDUINO AND COMPUTER
 *****************************************************************************************/
//This program must be called in setup()
void talkToComputer() {
  DL_Flush_All();   //flush any data in the buffer
  char c = Serial.read();                 // Throw out character that got into communicate mode
  char q;
  uint32_t startTime = millis();          // initialize startTime which is used to return to logging after a period of inactivity
  boolean talkingToComputer = true;
  PrintName();
  Serial.print("Bytes in memory: ");
  Serial.println((((uint32_t)DL_Page_Write_Addr()) << 8) + DL_Buffer_Wr_Pos());  // Number of data points stored on EEPROM chip        
  Serial.println("COMMANDS: B: Stored bytes D: Background R: Read T: Show Time X: Delete E: Exit");
  while (talkingToComputer) {
    if (Serial.available() > 0) {
      c = Serial.read();
      startTime = millis();
    }
    else if (millis() - startTime > 60000) { // Go back to logging after minute of inactivity
      c = 'E';                            // E to exit the talking to computer state
    }
    else {
      c = ' ';
    }
    switch (c)
    {
      case 'B':
        Serial.print((((uint32_t)DL_Page_Write_Addr()) << 8) + DL_Buffer_Wr_Pos());  // Number of data points stored on EEPROM chip        break;
        break;
      case 'D':
        PrintBackground(&detOneBkgd);
        PrintBackground(&detTwoBkgd);
        PrintBackground(&detThreeBkgd);
        break;
      case 'E':  //exit
        talkingToComputer = false;
        break;
      case 'R':
        uint16_t i;
        while (DL_Read_Page())
        {
          for (i = 0; i < 256; i++)
            Serial.write(DL_get_value(i));
        }
        DL_Reset_Read_Addr();
        break;
      case 'T':
        Serial.print("Current millis(): ");
        Serial.println(millis());
        Serial.print("Current micros(): ");
        Serial.println(micros());
        break;
      case 'X':  //send X to delete known data
      case 'Y':  //send Y to do a complete overwrite of the EEPROM - really only needed in debugging
        Serial.println("Delete?(Y/N)");     // Verify user wants to delete all data, Y = yes and N = no (or any other character
        while (Serial.available() == 0) {
          if (millis() - startTime > 60000)
            break;
        }
        q = Serial.read();
        if (q == 'Y') {
          Serial.print("Deleting...");
          DL_Clear_Memory(c - 'X') ;
          Serial.println("Done");
        }
        else {
          Serial.println("Cancel");
        }
        break;
      default:
        break;
    }  //end switch
  }   //end while
  Serial.println("Logging");
}     //end talkToComputer

/****************************************************************************************
 * PRINTING DATA TO COMPUTER DURING SERIAL CONNECTION
 *****************************************************************************************/
void PrintName() {
  Serial.print("Bubble Size Sensor");
  Serial.print(" HW: v");
  Serial.print(HW_VERSION);
  Serial.print(" FW: v");
  Serial.println(FW_VERSION);
}

void PrintBackground(backgrounddata * bkgd){
  uint8_t c;
  for (c = 0; c <NUM_BKGD_POINTS; c++){
    Serial.print(bkgd->rb[c]);
    Serial.print(' '); 
  }
  Serial.println();
  Serial.print(bkgd->minvalue);
  Serial.print(' ');
  Serial.print(bkgd->maxvalue);
  Serial.print(' ');
  Serial.print(bkgd->total);
  Serial.print(' ');
  Serial.print(bkgd->startdetvalue);
  Serial.print(' ');
  Serial.println(bkgd->enddetvalue);
}


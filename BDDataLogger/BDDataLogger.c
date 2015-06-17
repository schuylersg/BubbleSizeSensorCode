#include <math.h>
#include <stdlib.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "Arduino.h" // for digitalWrite
#include "../BubbleDetector/BubbleDetector.h"

#include "BDDataLogger.h"

#define DEV_ADDR 0b10100000
#define MAX_PAGE_VALUE 2048	//2^11 pages
#define TWI_BUFFER_SIZE 256	//512
#define HIGH_LEVEL_MARK 128
#define TWI_TWBR            0x0C        // TWI Bit rate Register setting. Set to 400 kHz
                                        // See Application note for detailed 
                                        // information on setting this value.

static volatile uint8_t buf_1[TWI_BUFFER_SIZE];
static volatile uint8_t buf_2[TWI_BUFFER_SIZE];																			
static volatile uint8_t *wr_buffer;
static volatile uint8_t *rd_buffer;
static volatile uint8_t *temp_buffer;
static volatile uint8_t buf_wr_pos;
static uint16_t page_wr_addr;
static uint16_t page_rd_addr;
static uint8_t addr_buf[3];
static volatile uint8_t ready_to_save_flag;

static volatile unsigned char TWI_state = TWI_NO_STATE;      // State byte. Default set to TWI_NO_STATE.
union TWI_statusReg TWI_statusReg = {0};            // TWI_statusReg is defined in TWI_Master.h
static uint16_t TWI_msgSize;
static volatile uint8_t operation;

/****************************************************************************
Call this function to set up the TWI master to its initial standby state.
Remember to enable interrupts from the main application after initializing the TWI.
****************************************************************************/
void DL_Initialise(void)
{
	//setup TWI registers
  TWBR = TWI_TWBR;                                  // Set bit rate register (Baudrate). Defined in header file.
// TWSR = TWI_TWPS;                                  // Not used. Driver presumes prescaler to be 00.
  TWDR = 0xFF;                                      // Default content = SDA released.
  TWCR = (1<<TWEN)|                                 // Enable TWI-interface and release TWI pins.
         (0<<TWIE)|(0<<TWINT)|                      // Disable Interupt.
         (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // No Signal requests.
         (0<<TWWC);                                 //
	
	//set the buffer and eeprom addresses to 0
	wr_buffer = buf_1;
	rd_buffer = buf_2;
	buf_wr_pos = 0;
	page_wr_addr = MAX_PAGE_VALUE-1;
	page_rd_addr = 0;
	ready_to_save_flag = 0;
	uint16_t i;
	//find the end of the file
	i = 0;
	while (DL_Read_Page()){
		while( i < 256){
			if (rd_buffer[i] == 255)
				break;
			//each case should be a message type
			//cases with the same size value are placed together with no "breaks"
			switch(rd_buffer[i])
			{ 
				case MSG_EVENT_START:
					i += SIZE_EVENT_START + 1;
					break;
				case MSG_EVENT_END:
				case MSG_DET1_START:	
				case MSG_DET2_START:
				case MSG_DET3_START:
				case MSG_DET1_END:
				case MSG_DET2_END:	
				case MSG_DET3_END:
					i += SIZE_DET3_END + 1;
					break;
				case MSG_ERROR:
					i += SIZE_ERROR;
					break;
				default:
					i++;
					break;
			}
		}
		if (i > 255)
			i = i-256;
		else	//end of file
		{
			page_wr_addr = page_rd_addr - 1;
			swap_buffers();
			ready_to_save_flag = 0;
			buf_wr_pos = i;
			page_rd_addr = 0;
			break;
		}
	}
	page_rd_addr = 0;
}

//Call to erase memory
//if it's called with True, then all memory is written to
//else, it just erases up through the last write
void DL_Clear_Memory( uint8_t total_clear){
	
	sbi(PORTB, 5);
	uint16_t max_page_wr_addr = page_wr_addr;
	if(total_clear)
		max_page_wr_addr = MAX_PAGE_VALUE-1;

	//fill buffer with 255
	uint16_t i;
	for ( i = 0; i < 256; i++)
		rd_buffer[i] = 255;
	
	//do all writes
	page_wr_addr = 0;
	while (page_wr_addr < max_page_wr_addr){
		DL_Write_Page();
	}
	
	//reset state variables
	page_rd_addr = 0;
	page_wr_addr = 0;
	buf_wr_pos = 0;
	ready_to_save_flag = 0;
	cbi(PORTB, 5);
	
}

void DL_Write_Page( void )
{
	if (page_wr_addr >= MAX_PAGE_VALUE)
			return;
	//set select and address bytes
  while ( TWI_Transceiver_Busy() );             // Wait until TWI is ready for next transmission.
	addr_buf[0] = (uint8_t) (DEV_ADDR | (page_wr_addr>>8)<<1);	//highest three bits of address, bit shifted left 1 for r/w bit
	addr_buf[1] = (uint8_t)(page_wr_addr);
	addr_buf[2] = (uint8_t) 0;	//always at start of page
	page_wr_addr++;	//increment write page address
	TWI_msgSize = TWI_BUFFER_SIZE;
	ready_to_save_flag = 0;	//reset flag
	operation = DL_WRITE;
  TWI_statusReg.all = 0;
  TWI_state = TWI_NO_STATE ;
  TWCR = (1<<TWEN)|                             // TWI Interface enabled.
         (1<<TWIE)|(1<<TWINT)|                  // Enable TWI Interupt and clear the flag.
         (0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|       // Initiate a START condition.
         (0<<TWWC);                             //
}

/****************************************************
Call from main application to read page. Any data not
stored will be deleted, so it is best to call DL_Flush_All()
before this.
****************************************************/
uint8_t DL_Read_Page( void )
{
  while ( TWI_Transceiver_Busy() );             // Wait until TWI is ready for next transmission.	
	if (page_rd_addr > page_wr_addr)
		return 0;
  addr_buf[0] = (uint8_t) (DEV_ADDR | (page_rd_addr>>8)<<1);	//highest three bits of address, bit shifted left 1 for r/w bit
	addr_buf[1] = (uint8_t)(page_rd_addr);
	addr_buf[2] = (uint8_t) 0;	//always at start of page
	page_rd_addr++;	//increment write page address
	TWI_msgSize = 256;	//no message, just writing address
  TWI_statusReg.all = 0;
  TWI_state = TWI_NO_STATE ;
	operation = DL_READ_PHASE_1;
  TWCR = (1<<TWEN)|                             // TWI Interface enabled.
         (1<<TWIE)|(1<<TWINT)|                  // Enable TWI Interupt and clear the flag.
         (0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|       // Initiate a START condition.
         (0<<TWWC);                            //
	__asm__("nop\n\t"); 
	while ( TWI_Transceiver_Busy() );             // Wait until TWI is ready for next transmission.
	return 1;	
	}

/***************************************
Called from main application to write a 
data packet of one uint32_t.
**************************************/
void DL_Write1(uint8_t data_type, uint32_t d32)
{
	write_byte(data_type);
	write_byte((uint8_t) (d32>>24));
	write_byte((uint8_t) (d32>>16));
	write_byte((uint8_t) (d32>>8));
	write_byte((uint8_t) (d32));
	
	//if the write buffer is greater than some threshold, 
	//start doing a save
	if (ready_to_save_flag && buf_wr_pos > HIGH_LEVEL_MARK)
		DL_Write_Page();
}

void DL_Write2(uint8_t data_type, uint32_t d32, uint16_t d1, uint16_t d2, uint16_t d3 )
{
	write_byte(data_type);
	write_byte((uint8_t) (d32>>24));
	write_byte((uint8_t) (d32>>16));
	write_byte((uint8_t) (d32>>8));
	write_byte((uint8_t) (d32));
	write_byte((uint8_t) (d1>>8));
	write_byte((uint8_t) (d1));
	write_byte((uint8_t) (d2>>8));
	write_byte((uint8_t) (d2));
	write_byte((uint8_t) (d3>>8));
	write_byte((uint8_t) (d3));
	
	//if the write buffer is greater than some threshold, 
	//start doing a save
	if (ready_to_save_flag && buf_wr_pos > HIGH_LEVEL_MARK)
		DL_Write_Page();
}

void DL_Write3(uint8_t data_type, uint8_t d)
{
	write_byte(data_type);
	write_byte(d);
	
	//if the write buffer is greater than some threshold, 
	//start doing a save
	if (ready_to_save_flag && buf_wr_pos > HIGH_LEVEL_MARK)
		DL_Write_Page();
}

/***********************************************
Call this function to swap read and write buffers 
and set the ready_to_save_flag.
***********************************************/
void swap_buffers(void){
  while ( TWI_Transceiver_Busy() );             // Wait until TWI has completed the transmission.
	temp_buffer = rd_buffer;
	rd_buffer = wr_buffer;
	wr_buffer = temp_buffer;
	ready_to_save_flag = 1;
}

/*********************************************
Store byte in wr_buffer, swapping buffers if it gets full.
*********************************************/
void write_byte(uint8_t data)
{
	wr_buffer[buf_wr_pos++] = data;
	if (buf_wr_pos == 0)
		swap_buffers();
}

uint16_t DL_Page_Read_Addr(void){
	return page_rd_addr;
}

uint16_t DL_Page_Write_Addr(void){
	return page_wr_addr;
}

uint8_t DL_Buffer_Wr_Pos(void){
	return buf_wr_pos;
}

uint8_t * DL_Get_Wr_Buffer(void){
	return wr_buffer;
}

uint8_t * DL_Get_Rd_Buffer(void){
	return rd_buffer;
}

void DL_Clear_Buffers(void){
	uint16_t i;
	for (i = 0; i<256; i ++){
		wr_buffer[i] = 0;
		rd_buffer[i] = 0;
	}
}

/************************************
Called by the main application if there is some "down time"
to write the data to EEPROM if one of the buffers is full.
************************************/
void DL_Safe_To_Write( void )
{
	if(ready_to_save_flag)
		DL_Write_Page();
}

/*******************************************************
Flush all data waiting to be written.
*******************************************************/
uint8_t DL_Flush_All( void ){
	uint8_t retval = 0;
	if (ready_to_save_flag)	//one buffer is full but hasn't yet been stored to the EEPROM
	{
		DL_Write_Page();
		retval = 127;
	}
	if(buf_wr_pos > 0)		//there is data in the second partially-filled buffer to write
	{
		int16_t i;
		//fill the remaining buffer up with 255
		for (i = buf_wr_pos; i < 256; i++)
			wr_buffer[i] = 255;
		swap_buffers();
		DL_Write_Page();
		swap_buffers();	//we want to swap back so that if the user keeps storing data,
										//it's in the correct buffer at the correct position buf_wr_pos
		ready_to_save_flag = 0;	//swap_buffers set this to 1, but that data was actually already written so ignore
		retval = retval | 0b11110000;
	}
	return retval;
}

uint8_t DL_get_value(uint8_t index){
	return *(rd_buffer+index);
}
/****************************************************************************
Call this function to test if the TWI_ISR is busy transmitting.
****************************************************************************/
unsigned char TWI_Transceiver_Busy( void )
{
  return ( TWCR & (1<<TWIE) );                  // IF TWI Interrupt is enabled then the Transceiver is busy
}

/****************************************************************************
Call this function to fetch the state information of the previous operation. The function will hold execution (loop)
until the TWI_ISR has completed with the previous operation. If there was an error, then the function 
will return the TWI State code. 
****************************************************************************/
unsigned char TWI_Get_State_Info( void )
{
  while ( TWI_Transceiver_Busy() );             // Wait until TWI has completed the transmission.
  return ( TWI_state );                         // Return error state.
}

// ********** Interrupt Handlers ********** //
/****************************************************************************
This function is the Interrupt Service Routine (ISR), and called when the TWI interrupt is triggered;
that is whenever a TWI event has occurred. This function should not be called directly from the main
application.
****************************************************************************/
//#pragma vector=TWI_vect
//__interrupt void TWI_ISR(void)
ISR(TWI_vect)
{ 
	static volatile uint8_t addr_ptr;
	static volatile uint16_t data_ptr;
	
  switch (TWSR)
  {
    case TWI_START:             // START has been transmitted
    case TWI_REP_START:         // Repeated START has been transmitted
			addr_ptr = 0;
			data_ptr = 0;
    case TWI_MTX_ADR_ACK:       // SLA+W has been transmitted and ACK received
    case TWI_MTX_DATA_ACK:      // Data byte has been transmitted and ACK received
			if (addr_ptr < 3 && operation < DL_READ_PHASE_2){
				TWDR = addr_buf[addr_ptr++];
				TWCR = (1<<TWEN)|                               // TWI Interface enabled
						 (1<<TWIE)|(1<<TWINT)|                      // Enable TWI Interupt and clear the flag to send byte
						 (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           //
						 (0<<TWWC);                               
			}else if(operation == DL_READ_PHASE_1){ //read phase 1
				operation = DL_READ_PHASE_2;
				TWCR = (1<<TWEN)|                         // TWI Interface enabled.
					 (1<<TWIE)|(1<<TWINT)|                  // Enable TWI Interupt and clear the flag.
					 (0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|       // Initiate a Repeated START condition.
					 (0<<TWWC);     		
			}else if(operation == DL_READ_PHASE_2){	//read phase 2
				TWDR = addr_buf[0] | 0b00000001;	//change to read device select
				TWCR = (1<<TWEN)|                               // TWI Interface enabled
						 (1<<TWIE)|(1<<TWINT)|                      // Enable TWI Interupt and clear the flag to send byte
						 (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           //
						 (0<<TWWC);      
			}else if (data_ptr < TWI_msgSize){
				TWDR = rd_buffer[data_ptr++];
        TWCR = (1<<TWEN)|                                 // TWI Interface enabled
               (1<<TWIE)|(1<<TWINT)|                      // Enable TWI Interupt and clear the flag to send byte
               (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           //
               (0<<TWWC);                                 //  
      }else                    // Send STOP after last byte
      {
        TWI_statusReg.lastTransOK = TRUE;                 // Set status bits to completed successfully. 
        TWCR = (1<<TWEN)|                                 // TWI Interface enabled
               (0<<TWIE)|(1<<TWINT)|                      // Disable TWI Interrupt and clear the flag
               (0<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|           // Initiate a STOP condition.
               (0<<TWWC);                                 //
      }
      break;
    case TWI_MRX_DATA_ACK:      // Data byte has been received and ACK transmitted
			rd_buffer[data_ptr++] = TWDR;
		case TWI_MRX_ADR_ACK:       // SLA+R has been transmitted and ACK received
      if (data_ptr < (TWI_msgSize-1) )                  // Detect the last byte to NACK it.
      {
        TWCR = (1<<TWEN)|                                 // TWI Interface enabled
               (1<<TWIE)|(1<<TWINT)|                      // Enable TWI Interupt and clear the flag to read next byte
               (1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // Send ACK after reception
               (0<<TWWC);                                 //  
      }else                    // Send NACK after next reception
      {
        TWCR = (1<<TWEN)|                                 // TWI Interface enabled
               (1<<TWIE)|(1<<TWINT)|                      // Enable TWI Interupt and clear the flag to read next byte
               (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // Send NACK after reception
               (0<<TWWC);                                 // 
      }    
      break; 
    case TWI_MRX_DATA_NACK:     // Data byte has been received and NACK transmitted
      rd_buffer[data_ptr] = TWDR;
      TWI_statusReg.lastTransOK = TRUE;                 // Set status bits to completed successfully. 
      TWCR = (1<<TWEN)|                                 // TWI Interface enabled
             (0<<TWIE)|(1<<TWINT)|                      // Disable TWI Interrupt and clear the flag
             (0<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|           // Initiate a STOP condition.
             (0<<TWWC);                                 //
      break;      
    case TWI_ARB_LOST:          // Arbitration lost
      TWCR = (1<<TWEN)|                                 // TWI Interface enabled
             (1<<TWIE)|(1<<TWINT)|                      // Enable TWI Interupt and clear the flag
             (0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|           // Initiate a (RE)START condition.
             (0<<TWWC);                                 //
      break;
    case TWI_MTX_ADR_NACK:      // SLA+W has been tramsmitted and NACK received
    case TWI_MRX_ADR_NACK:      // SLA+R has been tramsmitted and NACK received    
			TWCR = (1<<TWEN)|                             // TWI Interface enabled.
         (1<<TWIE)|(1<<TWINT)|                  // Enable TWI Interupt and clear the flag.
         (0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|       // Initiate a START condition.
         (0<<TWWC);                            //
			break;
    case TWI_MTX_DATA_NACK:     // Data byte has been tramsmitted and NACK received
//    case TWI_NO_STATE              // No relevant state information available; TWINT = “0”
    case TWI_BUS_ERROR:         // Bus error due to an illegal START or STOP condition
    default:
			cbi(PORTB, 5);
      TWI_state = TWSR;                                 // Store TWSR and automatically sets clears noErrors bit.
                                                        // Reset TWI Interface
      TWCR = (1<<TWEN)|                                 // Enable TWI-interface and release TWI pins
             (0<<TWIE)|(0<<TWINT)|                      // Disable Interupt
             (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // No Signal requests
             (0<<TWWC);                                 //
  }
}
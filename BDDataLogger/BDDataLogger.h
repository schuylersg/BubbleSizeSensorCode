/**
Description:	Header file for bubble data logger EEPROM controller

Author: 		Schuyler Senft-Grupp
Version: 		1.0
Date: 			6/15/2015

**/

#ifndef __BD_DATA_LOGGER__
	#define __BD_DATA_LOGGER__

	#include <inttypes.h>

	#ifdef __cplusplus
	extern "C" {
	#endif
	
	/****************************************************************************
		Global definitions
	****************************************************************************/
	union TWI_statusReg                       // Status byte holding flags.
	{
			unsigned char all;
			struct
			{
					unsigned char lastTransOK:1;      
					unsigned char unusedBits:7;
			};
	};

	extern union TWI_statusReg TWI_statusReg;

	/****************************************************************************
		Function definitions
	****************************************************************************/
	void DL_Initialise(void);
	uint8_t DL_Read_Page( void );		//read the next page if 0, 1 resets to beginning
								//then reads page into buffer and increments read addr
	uint8_t DL_Flush_All(void);	//force write of partially filled buffer
	void DL_Safe_To_Write(void); 	//allow page write
	uint16_t DL_Page_Read_Addr(void);
	uint16_t DL_Page_Write_Addr(void);
	uint8_t DL_get_value(uint8_t);
	uint8_t DL_Buffer_Wr_Pos(void);
	void DL_Clear_Buffers(void);
	uint8_t * DL_Get_Wr_Buffer(void);
	uint8_t * DL_Get_Rd_Buffer(void);
	void DL_Clear_Memory( uint8_t );
	void swap_buffers(void);
	void DL_Reset_Read_Addr( void);
	
	void DL_Write1(uint8_t , uint32_t);
	void DL_Write2(uint8_t, uint32_t, uint16_t, uint16_t, uint16_t);
	void DL_Write3(uint8_t, uint8_t);
	
	unsigned char TWI_Transceiver_Busy( void );

	/****************************************************************************
		Bit and byte definitions
	****************************************************************************/
	#define TWI_READ_BIT  0       // Bit position for R/W bit in "address byte".
	#define TWI_ADR_BITS  1       // Bit position for LSB of the slave address bits in the init byte.

	#define TRUE          1
	#define FALSE         0

	//Read/Write operation definitions
	#define DL_WRITE 0
	#define DL_READ_PHASE_1 1
	#define DL_READ_PHASE_2 2
	
	//Useful macros
	#ifndef cbi
	#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
	#endif

	#ifndef sbi
	#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
	#endif
	
	

	
	/****************************************************************************
		TWI State codes
	****************************************************************************/
	// General TWI Master status codes                      
	#define TWI_START                  0x08  // START has been transmitted  
	#define TWI_REP_START              0x10  // Repeated START has been transmitted
	#define TWI_ARB_LOST               0x38  // Arbitration lost

	// TWI Master Transmitter status codes                      
	#define TWI_MTX_ADR_ACK            0x18  // SLA+W has been tramsmitted and ACK received
	#define TWI_MTX_ADR_NACK           0x20  // SLA+W has been tramsmitted and NACK received 
	#define TWI_MTX_DATA_ACK           0x28  // Data byte has been tramsmitted and ACK received
	#define TWI_MTX_DATA_NACK          0x30  // Data byte has been tramsmitted and NACK received 

	// TWI Master Receiver status codes  
	#define TWI_MRX_ADR_ACK            0x40  // SLA+R has been tramsmitted and ACK received
	#define TWI_MRX_ADR_NACK           0x48  // SLA+R has been tramsmitted and NACK received
	#define TWI_MRX_DATA_ACK           0x50  // Data byte has been received and ACK tramsmitted
	#define TWI_MRX_DATA_NACK          0x58  // Data byte has been received and NACK tramsmitted

	// TWI Slave Transmitter status codes
	#define TWI_STX_ADR_ACK            0xA8  // Own SLA+R has been received; ACK has been returned
	#define TWI_STX_ADR_ACK_M_ARB_LOST 0xB0  // Arbitration lost in SLA+R/W as Master; own SLA+R has been received; ACK has been returned
	#define TWI_STX_DATA_ACK           0xB8  // Data byte in TWDR has been transmitted; ACK has been received
	#define TWI_STX_DATA_NACK          0xC0  // Data byte in TWDR has been transmitted; NOT ACK has been received
	#define TWI_STX_DATA_ACK_LAST_BYTE 0xC8  // Last data byte in TWDR has been transmitted (TWEA = “0”); ACK has been received

	// TWI Slave Receiver status codes
	#define TWI_SRX_ADR_ACK            0x60  // Own SLA+W has been received ACK has been returned
	#define TWI_SRX_ADR_ACK_M_ARB_LOST 0x68  // Arbitration lost in SLA+R/W as Master; own SLA+W has been received; ACK has been returned
	#define TWI_SRX_GEN_ACK            0x70  // General call address has been received; ACK has been returned
	#define TWI_SRX_GEN_ACK_M_ARB_LOST 0x78  // Arbitration lost in SLA+R/W as Master; General call address has been received; ACK has been returned
	#define TWI_SRX_ADR_DATA_ACK       0x80  // Previously addressed with own SLA+W; data has been received; ACK has been returned
	#define TWI_SRX_ADR_DATA_NACK      0x88  // Previously addressed with own SLA+W; data has been received; NOT ACK has been returned
	#define TWI_SRX_GEN_DATA_ACK       0x90  // Previously addressed with general call; data has been received; ACK has been returned
	#define TWI_SRX_GEN_DATA_NACK      0x98  // Previously addressed with general call; data has been received; NOT ACK has been returned
	#define TWI_SRX_STOP_RESTART       0xA0  // A STOP condition or repeated START condition has been received while still addressed as Slave

	// TWI Miscellaneous status codes
	#define TWI_NO_STATE               0xF8  // No relevant state information available; TWINT = “0”
	#define TWI_BUS_ERROR              0x00  // Bus error due to an illegal START or STOP condition

	#ifdef __cplusplus
	}
	#endif
	
#endif //__BD_DATA_LOGGER__
/*
 * System.h
 *
 * Created: 4/1/2015 9:17:44 AM
 *  Author: avasquez
 */ 


#ifndef SYSTEM_H_
#define SYSTEM_H_

#define systemBYTE_TIMEOUT	( 4 )	// Waiting-on-byte timeout

#define F_CPU 16000000UL
#define BAUDRATE 4800        //The baudrate that we want to use
#define BAUD_PRESCALLER (((F_CPU / (BAUDRATE * 16UL))) - 1)    //The formula that does all the required maths

//////////////////////////////////////////////////////////////////////////
// * System Constants Definition
//////////////////////////////////////////////////////////////////////////
// %%%%%% EEPROM
#define eepromDEVICE_MEM_DUMP		( ( void * ) 0x0120 )	// Device ID Memory Location
#define eepromDEVICE_ID				( ( void * ) 0x10 )	// Device ID Memory Location

#define eepromCOM_RX_A				( ( void * ) 0x20 )	// Device ID Memory Location
#define eepromCOM_RX_B				( ( void * ) 0x21 )	// Device ID Memory Location
#define eepromCOM_RX_C				( ( void * ) 0x22 )	// Device ID Memory Location
#define eepromCOM_RX_D				( ( void * ) 0x23 )	// Device ID Memory Location

#define eepromCOM_RX_RAW			( ( void * ) 0x30 )	// Device ID Memory Location
#define eepromCOM_TX_RAW			( ( void * ) 0xC0 )	// Device ID Memory Location

#define eepromPEYE_TIMEOUT			( ( void * ) 0x60 )	// Device ID Memory Location

#define eepromCOM_RX_SUCCESS		( ( void * ) 0x50 )	// Device ID Memory Location
#define eepromCOM_RX_DROPPED		( ( void * ) 0x51 )	// Device ID Memory Location
#define eepromCOM_RX_CORRUPT		( ( void * ) 0x52 )	// Device ID Memory Location

#define eepromCOM_RX_COUNT			( ( void * ) 0x90 )	// Device ID Memory Location
#define eepromCOM_RX_DEBUG			( ( void * ) 0xE0 )	// Device ID Memory Location

#define eepromSYS_CONVEYOR_SPEED	( ( void * ) 0x70 ) // Conveyor Speed Memory Location
#define eepromDYV_TABLE_VALUES		( ( void * ) 0x0100 ) // Conveyor Speed Memory Location

// %%%%%% Maths Constants
#define mathTO_MM_SEC				( ( float ) 5.08 )	// Conversion Constant
#define mathSLAT_WIDTH				( ( uint32_t ) 158750 )// System Constant
#define durationBYTE_DELAY			( ( uint16_t ) 2 )		// Wait 6ms before sending next byte
#define delayIR_PACKET				( ( uint32_t ) 15 )// System Constant
#define dataDEFAULT_COMMAND			( ( uint8_t ) 0b10000111 )	// <- Slat 1 Command

// ###### System Modes
#define SYSTEM_MODE_PROCESS		( ( uint8_t ) 0 )
#define SYSTEM_MODE_DIVERT		( ( uint8_t ) 1 )
#define SYSTEM_MODE_PHOTOEYE	( ( uint8_t ) 2 )
#define SYSTEM_MODE_PROGRAM		( ( uint8_t ) 3 )
#define SYSTEM_MODE_DEBUG		( ( uint8_t ) 4 )
#define SYSTEM_MODE_DATA_WAIT	( ( uint8_t ) 5 )

#define BOARD_TYPE_DIVERT		( ( uint8_t ) 0 )
#define BOARD_TYPE_PARAMETER	( ( uint8_t ) 1 )

//////////////////////////////////////////////////////////////////////////
// * Global System Variable
typedef struct mySystems {
	uint8_t devAddress;	// Device Address
	uint8_t chTran;	// Device Mode
	uint8_t chMode;	// Device Mode
	
	// Timing Mechanisms
	uint32_t clkCount;	// Status
	uint8_t clkDataTick;	// Data Timeout Tick
	
	uint8_t ConveyorSpeedByte;	// Byte representation for conveyor speed
	uint16_t ConveyorSpeed;		// True conveyor speed 
	uint16_t PhotoeyeDelayWindow;	// Time to wait for Photoeye
} mySystem;

// * USART Rx Data Structure
typedef struct myUSARTS {
	uint8_t PacketRxStatus;
	uint8_t PacketLength;
	uint8_t LoP;
	
	uint16_t TimeoutTickCount;
	
	char RxChar;
	char PacketData[30];
	
	// Byte Declarations
	int8_t ConveyorSpeed;
	int8_t RURD;
	int8_t DivertDelay;
	
	// Utilities
	uint8_t PreviousDroppedPackets, DroppedPackets;
	uint8_t PreviousRxPackets, RxPackets;
	uint8_t PreviousCorruptPackets, CorruptPackets;
	uint8_t IsPacketCorrupt;
} myUSART;

// * USART Rx Data Structure
typedef struct myIHerbDiverts {
	// Table of diverting values
	char tblDivertData[28];	// 28 values in a table that accounts for 1 to 7 action slats
							// Data must be retrieved using the linear relationship:
							// k = ( ( j * ( j + 1 ) ) / 2 ) + i;
	char command;	// Current command to be sent
	
	uint16_t SlatTravelTimes[7];	// This is a list of travel times
	
	uint8_t i,j;	// Data synchronizers
	uint8_t IsToGo;
	uint8_t irPacketLength;
	uint8_t irSendPacket;
	
	uint32_t ProgrammingDuration;
	uint32_t ByteDelay;	// Byte Delay Counter
	uint32_t irPacketDelay;
	uint16_t PhotoeyeTimeout;
	uint8_t	PhotoeyeTripped;
	uint8_t DivertDirection;
} myIHerbDivert;

#endif /* SYSTEM_H_ */
/* ##################################################################################
#####################################################################################
# iHerb Transmitter Routine
# Description: This is the simplest of codes that accounts for an internal network.
#####################################################################################
##################################################################################### */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include "System.h"

// Function Prototype
uint8_t fcnSerialInput( void );
void USART_SendByte(uint8_t u8Data);
void setup( void );
void usartCleanPacket( void );
void usartNetworkStatistics( void );
void fcnProcessPacket( void );
void fcnSendDivertData( void );
void fcnSelectCommandByte( void );
void fcnWaitForPhotoeye( void );
void fcnSendParameters( void );
void fcnInSeriesByteDelay( void );
void fcnRunDebugMode(void);
void fcnChipStop(void);

// Global Variables
mySystem mySys;
myUSART myCOM;
myIHerbDivert myDivert;
uint8_t intTestByteTotal = 0x00;


int main( void )
{
	// Run Setup 
	setup();
	
    while(1) {	// Loop forever 
		switch (mySys.chMode)  {	// Note the case
			case SYSTEM_MODE_PROCESS:	// On the Divert Idle mode....
				// Process a packet as soon as it is received
				//USART_SendByte(0x0F);
				//PORTC |= 0x01;	// YELLOW
				fcnProcessPacket();
			break;
			case SYSTEM_MODE_DIVERT:	// Perform Divert Sequence
				//PORTC &= ~(0x01);
				fcnSendDivertData();
			break;
			case SYSTEM_MODE_PHOTOEYE:	// Waiting on Photoeye
				//PORTC |= 0x04;	// GREEN
				fcnWaitForPhotoeye();
			break;
			case SYSTEM_MODE_PROGRAM:	// Start and Keep Transmitting Parameters
				//PORTC &= ~(0x01);
				//PORTC |= 0x04;	// GREEN
				fcnSendParameters();
			break;
			case SYSTEM_MODE_DEBUG:	// Enter Debug Mode
				//PORTC &= ~(0x01);
				//PORTC |= 0x04;	// GREEN
				fcnRunDebugMode();
			break;
			case SYSTEM_MODE_DATA_WAIT:	// On the Programmer Idle mode....
				//PORTC &= ~(0x04);
				// Start waiting for a Transmission packet
			break;
			default:
			break;
		}
    }
}

const uint32_t DebugDelay = 0x0002BF20;	// 3 Minutes in 
uint16_t DebugCommand;
void fcnRunDebugMode(void) {
	// Run the same command for about 3 minutes
	UCSR0B = 0b10001000;	// Rx Disabled. Rx Interrupt Enabled
	
	if ( mySys.devAddress == 0 ) {
		DebugCommand = myCOM.PacketData[2];
		//DebugCommand = 0x87;
		mySys.clkCount = 0;
		myDivert.ByteDelay = mySys.clkCount;
		myDivert.IsToGo = 0xFF;
		
		while ( !(mySys.clkCount > DebugDelay) )
		{
			// Check if Byte separator has been satisfied
			if ( mySys.clkCount > ( uint32_t ) ( myDivert.ByteDelay + durationBYTE_DELAY ) ) {
				myDivert.IsToGo = 0xFF;
				myDivert.ByteDelay = mySys.clkCount;
			}
			
			if ( myDivert.irPacketLength > 2 ) {
				// Since for now, we know that no bytes should be sent, disable
				myDivert.irSendPacket = 0x00;
				
				if ( mySys.clkCount > ( uint32_t ) ( myDivert.irPacketDelay + delayIR_PACKET ) ) {
					myDivert.irSendPacket = 0xFF;
					myDivert.irPacketLength = 0;
					
					//mySys.clkCount = 0;
					myDivert.ByteDelay = mySys.clkCount;
					myDivert.IsToGo = 0xFF;
				}
			}
			
			// Send the necessary data by stages
			if ( myDivert.IsToGo && myDivert.irSendPacket ) {
				myDivert.IsToGo = 0x00;
				
				// Check correct command based on time
				USART_SendByte( DebugCommand );
				//eeprom_write_byte(eepromCOM_TX_RAW + myDivert.j, myDivert.command);
				
				myDivert.irPacketLength++;
				if ( myDivert.irPacketLength == 3 ) {
					myDivert.irPacketDelay = mySys.clkCount;
				}
			}
			
			
		}
	}
	
	mySys.clkCount = 0;
	
	usartCleanPacket();
	mySys.chMode = SYSTEM_MODE_DATA_WAIT;
	UCSR0B = 0b10011000;	// Rx Disabled. Rx Interrupt Enabled
}

void fcnInSeriesByteDelay( void ) {
	// Keep in loop if delay is needed
	while ( !( mySys.clkCount > ( uint32_t ) ( myDivert.ByteDelay + durationBYTE_DELAY ) ) ) {
	}
	myDivert.ByteDelay = mySys.clkCount;
}

void fcnSendParameters( void ) {
	
	
	
		// Send Character
	USART_SendByte( myCOM.ConveyorSpeed );
		// Delay transmission	
	fcnInSeriesByteDelay();
		// Send Character
	USART_SendByte( myCOM.ConveyorSpeed );
		// Delay transmission
	fcnInSeriesByteDelay();
		// Send Character
	USART_SendByte( myCOM.RURD );
		// Delay transmission
	fcnInSeriesByteDelay();
		// Send Character
	USART_SendByte( myCOM.RURD );
		// Delay transmission
	fcnInSeriesByteDelay();
		// Send Character
	USART_SendByte( myCOM.DivertDelay );
		// Delay transmission
	fcnInSeriesByteDelay();
		// Send Character
	USART_SendByte( myCOM.DivertDelay );
		// Delay transmission
	fcnInSeriesByteDelay();
	
	// Check for end of transmission
	if ( mySys.clkCount > myDivert.ProgrammingDuration ) {
		usartCleanPacket();
		mySys.chMode = SYSTEM_MODE_PROCESS;	//	Finish Transmission and go back to idling
		UCSR0B = 0b10011000;	// Rx Enabled. Rx Interrupt Enabled
	}
}

void fcnWaitForPhotoeye( void ) {
	
	
	// Trigger Debug
	//eeprom_write_byte(eepromCOM_RX_CORRUPT, 0xEE);
	//if ( ( uint16_t ) mySys.clkCount > mySys.PhotoeyeDelayWindow - 1 ) {
	myDivert.PhotoeyeTripped = 0xFF;	// Reset the Photoeye trip
	//}
	
	// Release the Divert if Photoeye was tripped
	if ( myDivert.PhotoeyeTripped ) {
		mySys.chMode = SYSTEM_MODE_DIVERT;	
		myDivert.PhotoeyeTripped = 0x00;	// Reset the Photoeye trip
		
		myDivert.i = myCOM.PacketData[3] - 1;	// Populate i
		myDivert.j = 0;
		//myDivert.command = myDivert.tblDivertData[ ( ( ( myDivert.i * ( myDivert.i + 1 ) ) / 2 ) + myDivert.j ) ];
		mySys.clkCount = 0;
		myDivert.ByteDelay = mySys.clkCount;
		myDivert.IsToGo = 0xFF;	// Is to transmit right away
	} else {
		// Check for timeout condition
		if ( ( uint16_t ) mySys.clkCount > mySys.PhotoeyeDelayWindow ) {
			usartCleanPacket();
			mySys.chMode = SYSTEM_MODE_DATA_WAIT;	//	Finish Transmission and go back to idling
			UCSR0B = 0b10011000;	// Rx Enabled. Rx Interrupt Enabled
		}
	}
}

//////////////////////////////////////////////////////////////////////////
//	Serial Input Function:
//		This function will allow for timeout if a byte does is not
//		received in the allotted time
//////////////////////////////////////////////////////////////////////////
void fcnSelectCommandByte( void ) {
	if ( mySys.clkCount == myDivert.SlatTravelTimes[myDivert.i] ) {
		usartCleanPacket();
		mySys.chMode = SYSTEM_MODE_DATA_WAIT;	//	Finish Transmission and go back to idling
		UCSR0B = 0b10011000;	// Rx Enabled. Rx Interrupt Enabled
	} else {
		if ( mySys.clkCount < 1 ) {	// j = 0
			myDivert.command = myDivert.tblDivertData[ ( ( ( myDivert.i * ( myDivert.i + 1 ) ) / 2 ) + 0 ) ];	 // First Slat
		} else if ( mySys.clkCount == ( uint32_t ) myDivert.SlatTravelTimes[myDivert.j] ) {	
			myDivert.j++;
			myDivert.command = myDivert.tblDivertData[ ( ( ( myDivert.i * ( myDivert.i + 1 ) ) / 2 ) + myDivert.j ) ];	 // Remaining Slats
		} 
		
		myDivert.command |= myDivert.DivertDirection;
	}
}

//////////////////////////////////////////////////////////////////////////
//	Serial Input Function:
//		This function will allow for timeout if a byte does is not
//		received in the allotted time
//////////////////////////////////////////////////////////////////////////
void fcnSendDivertData( void ) {
	// Perform only if Divert is on Divert Mode
	if ( mySys.chMode == SYSTEM_MODE_DIVERT ) {
		
		// Check if Byte separator has been satisfied
		if ( mySys.clkCount > ( uint32_t ) ( myDivert.ByteDelay + durationBYTE_DELAY ) ) {
			myDivert.IsToGo = 0xFF;
			myDivert.ByteDelay = mySys.clkCount;
		}
		
		// Send the necessary data by stages
		if ( myDivert.IsToGo ) {
			myDivert.IsToGo = 0x00;
			
			// Check correct command based on time
			fcnSelectCommandByte();
			USART_SendByte(myDivert.command);
			//eeprom_write_byte(eepromCOM_TX_RAW + myDivert.j, myDivert.command);
			
			
		}
		
		if ( !(mySys.clkCount < myDivert.SlatTravelTimes[0] * myCOM.PacketData[3]) ) {
			
			// End of action, reset everything
			mySys.clkCount = 0;
			usartCleanPacket();
			mySys.chMode = SYSTEM_MODE_DATA_WAIT;	//	Finish Transmission and go back to idling
			UCSR0C = 0b00110110;
			UCSR0B = 0b10011000;	// Tx/Rx Enabled. Rx Interrupt Enabled
		}
	}
}

//////////////////////////////////////////////////////////////////////////
//	Serial Input Function:
//		This function will allow for timeout if a byte does is not
//		received in the allotted time
//////////////////////////////////////////////////////////////////////////
void fcnProcessPacket( void ) {
	uint8_t i;
	
	cli();
	// Do we have a packet waiting to be processed?
	if ( myCOM.PacketRxStatus == 2 ) {
		
		
		// Scan Packet for integrity
		
		for ( i=0;i<myCOM.PacketLength;i++ ) {	// Run Packet Analysis
			if ( myCOM.PacketData[i] == 0xFE ) myCOM.IsPacketCorrupt = 1;
			//eeprom_write_byte(eepromCOM_RX_RAW+i, myCOM.PacketData[i]);
		}
			
		// If Packet is corrupt: 
		if ( myCOM.IsPacketCorrupt ) {
			// This IS a corrupt packet and counts as CORRUPT Packet
			myCOM.CorruptPackets++;
			// Reset Packet info
			usartCleanPacket();
			mySys.chMode = SYSTEM_MODE_DATA_WAIT;
			UCSR0B = 0b10011000;	// Rx Disabled. Rx Interrupt Enabled
		} else {
			// This IS NOT a corrupt packet and counts as Rxed Packet
			myCOM.RxPackets++;
			
			myCOM.PacketRxStatus = 3;
			
			
			
			
			//////////////////////////////////////////////////////////////////////////
			// Perform the necessary Logic on the packet before cleaning
			//////////////////////////////////////////////////////////////////////////
			switch ( myCOM.PacketData[1] ) {
				case 0xF7:	// Divert Trigger
					// Reset main clock to avoid any overflow conditions
					myCOM.PacketData[3] &= 0x0F;
					if ( myCOM.PacketData[3] > 0 && myCOM.PacketData[3] < 8 && mySys.chTran == BOARD_TYPE_DIVERT ) {
						//PORTC &= ~(0x04);
						// Get direction from this packet
						myDivert.DivertDirection = 0x00;
						myDivert.DivertDirection = myCOM.PacketData[2] & 0b10000000;
						myDivert.DivertDirection = myDivert.DivertDirection >> 1;
						
						myCOM.PacketData[2] &= 0x7F;
						if ( myCOM.PacketData[2] == mySys.devAddress ) {
							UCSR0B = 0b10001000;	// Rx Disabled. Rx Interrupt Enabled
							i = UDR0;
							i = UDR0;
							myDivert.PhotoeyeTripped = 0x00;
							mySys.clkCount = 0;
							mySys.chMode = SYSTEM_MODE_PHOTOEYE;		// Photo eye Mode
						} else {
							// End of action, reset everything
							usartCleanPacket();
							mySys.chMode = SYSTEM_MODE_DATA_WAIT;	
							UCSR0B = 0b10011000;	// Rx Disabled. Rx Interrupt Enabled
						}
					} else {	// Accounting for packet asking for 0 diverts
						// End of action, reset everything
						
						//if ( mySys.chTran != BOARD_TYPE_DIVERT ) {
						
						//}
								
						//if ( mySys.chTran != BOARD_TYPE_DIVERT ) {
							//PORTC |= 0x01;
							//uint16_t *ramMemMap;
							//ramMemMap = ((void *) 0x0100 );
							//for ( i=0; i < 0x80; i++ ) {
								//eeprom_write_byte(eepromDEVICE_MEM_DUMP+i, *ramMemMap);
								//ramMemMap++;
							//}
						//}
											
						
						
						usartCleanPacket();
						mySys.chMode = SYSTEM_MODE_DATA_WAIT;
						UCSR0B = 0b10011000;	// Rx Disabled. Rx Interrupt Enabled
						
					}
				break;
				case 0xF6:	// Programmer Trigger
					myCOM.ConveyorSpeed = myCOM.PacketData[2];
					mySys.ConveyorSpeedByte = myCOM.ConveyorSpeed;
					mySys.ConveyorSpeed = ( mySys.ConveyorSpeedByte * 5 ) + 60;	// Obtain on ft/min
					myDivert.SlatTravelTimes[0] = ( uint16_t ) ( ( ( float ) mySys.ConveyorSpeed ) * ( ( float ) mathTO_MM_SEC ) );
					myDivert.SlatTravelTimes[0] = ( uint16_t ) ( ( ( uint32_t ) mathSLAT_WIDTH ) / ( ( uint32_t ) myDivert.SlatTravelTimes[0] ) );
					for ( myDivert.i=1;myDivert.i<7;myDivert.i++ ) {
						myDivert.SlatTravelTimes[myDivert.i] = myDivert.SlatTravelTimes[0] * (1 + myDivert.i);
					}
					eeprom_write_byte(eepromSYS_CONVEYOR_SPEED, mySys.ConveyorSpeedByte);
					eeprom_write_word(eepromSYS_CONVEYOR_SPEED+5,myDivert.SlatTravelTimes[0]);	// Debug
					
					//myDivert.ProgrammingDuration = ( ( uint32_t ) 70 * ( uint32_t ) myDivert.SlatTravelTimes[0] );
					myDivert.ProgrammingDuration = ( ( uint32_t ) 300 * ( uint32_t ) myDivert.SlatTravelTimes[0] );
					
					eeprom_write_dword(eepromSYS_CONVEYOR_SPEED+7,myDivert.ProgrammingDuration);	// Debug
					
					// Divert Delay now takes the role of slat-duration run time
					myCOM.DivertDelay = myCOM.DivertDelay & 0b11110000;	// Clear byte to send
					myCOM.PacketData[3] = myCOM.PacketData[3] & 0b00001111;
					// Correct value if needed
					if ( myCOM.PacketData[3] < 4 ) {
						myCOM.PacketData[3] = 4;
					}
					myCOM.DivertDelay = myCOM.DivertDelay | myCOM.PacketData[3];
					//eeprom_write_byte(eepromCOM_RX_RAW+2,myCOM.DivertDelay);
				
					if ( mySys.chTran == BOARD_TYPE_PARAMETER ) {
						// This is to be done since this is a Programming Node
						UCSR0B = 0b10001000;	// Rx Disabled. Rx Interrupt Enabled
						mySys.clkCount = 0;
						myDivert.ByteDelay = mySys.clkCount;
						mySys.chMode = SYSTEM_MODE_PROGRAM;
					} else {
						// End of action, reset everything
						usartCleanPacket();
						mySys.chMode = SYSTEM_MODE_DATA_WAIT;
						UCSR0B = 0b10011000;	// Rx Disabled. Rx Interrupt Enabled
					}
				break;
				case 0xF5:	// A whole new set of table data is being transmitted and will need to be stored
					UCSR0B = 0b10001000;	// Rx Disabled. Rx Interrupt Enabled
					for ( myDivert.i = 0; myDivert.i < 28 ; myDivert.i++ ) {
						myDivert.tblDivertData[myDivert.i] = myCOM.PacketData[myDivert.i + 2];	// Store the byte in RAM and EEPROM
						eeprom_write_byte( eepromDYV_TABLE_VALUES + myDivert.i, myCOM.PacketData[myDivert.i + 2] );
					}
					// End of action, reset everything
					usartCleanPacket();
					UCSR0B = 0b10011000;	// Rx Disabled. Rx Interrupt Enabled
					mySys.chMode = SYSTEM_MODE_DATA_WAIT;
				break;
				case 0xF4:	// Update the Photoeye Timeout accordingly
					UCSR0B = 0b10001000;	// Rx Disabled. Rx Interrupt Enabled
					mySys.PhotoeyeDelayWindow = myCOM.PacketData[2];
					mySys.PhotoeyeDelayWindow = mySys.PhotoeyeDelayWindow << 8;
					mySys.PhotoeyeDelayWindow |= myCOM.PacketData[3];
					eeprom_write_word(eepromPEYE_TIMEOUT, mySys.PhotoeyeDelayWindow);
					// End of action, reset everything
					usartCleanPacket();
					UCSR0B = 0b10011000;	// Rx Disabled. Rx Interrupt Enabled
					mySys.chMode = SYSTEM_MODE_DATA_WAIT;
				break;
				case 0xF1:		// Enter Debug Mode
					myDivert.irSendPacket = 0xFF;
					myDivert.irPacketLength = 0;
					mySys.chMode = SYSTEM_MODE_DEBUG;
					
					
					//usartCleanPacket();
					//UCSR0B = 0b10011000;	// Rx Disabled. Rx Interrupt Enabled
					//mySys.chMode = 'I';				
				break;
				default:
					// Reset Packet info
					// End of action, reset everything
					usartCleanPacket();
					UCSR0B = 0b10011000;	// Rx Disabled. Rx Interrupt Enabled
					mySys.chMode = SYSTEM_MODE_DATA_WAIT;
				break;
			}
		}
	} else {
		// End of action, reset everything
		usartCleanPacket();
		UCSR0B = 0b10011000;	// Rx Disabled. Rx Interrupt Enabled
		mySys.chMode = SYSTEM_MODE_DATA_WAIT;
	}
	// Run Network Statistics
	//usartNetworkStatistics();
	
	sei();
}

//////////////////////////////////////////////////////////////////////////
// Byte Transmission Function
//////////////////////////////////////////////////////////////////////////
void USART_SendByte(uint8_t u8Data) {
	// Wait until last byte has been transmitted
	while( ( UCSR0A & ( 1<< UDRE0 ) ) == 0);
	// Transmit data
	UDR0 = u8Data;
}


//////////////////////////////////////////////////////////////////////////
// USART RX INTERRUPT SERVICE ROUTINE
//////////////////////////////////////////////////////////////////////////
// Rx ISR
uint8_t intRxDummy;
ISR( USART_RX_vect ) {
	//intRxDummy = UDR0;
	//if (mySys.chTran != BOARD_TYPE_DIVERT (UCSR0A & ( 1 << FE0 )) ) {
	//PORTC |= 0x01;
	
	
		
	if ( myCOM.PacketRxStatus < 2 ) {	// Are we waiting for data?
		//if ( ( UCSR0A & (1<<RXC0) )) {	// If a byte has been received and is waiting to be picked up
		if ( (UCSR0A & ( 1 << FE0 )) ) {
			intRxDummy = UDR0;
			myCOM.PacketData[0] = 0x00;
			myCOM.PacketData[1] = 0xFE;
			myCOM.PacketData[2] = 0xFE;
			myCOM.PacketData[3] = 0xFE;	// Erroneous Data Byte
			myCOM.PacketLength = 4;
			myCOM.LoP = 0x00;
			UCSR0B = 0b10001000;	// Rx Disabled. Rx Interrupt Enabled
			mySys.chMode = SYSTEM_MODE_PROCESS;
			myCOM.PacketRxStatus = 2;	// Packet Rx Complete
		} else if ( ( UCSR0A & (1<<RXC0) ) ) {	// If a byte has been received and is waiting to be picked up
			
			if ( myCOM.PacketRxStatus == 0 ) {
				myCOM.PacketRxStatus = 1;	// We are in the process of receiving and packet
				mySys.chMode = SYSTEM_MODE_DATA_WAIT;
			}
			
			if ( !(UCSR0A & ( 1 << UPE0 ) ) ) {	// Is Parity fine?
				// Parity OK
				myCOM.PacketData[myCOM.PacketLength] = UDR0;
			} else {	// Error in Rx Detected
				myCOM.PacketData[myCOM.PacketLength] = 0xFE;	// Erroneous Data Byte
				intRxDummy = UDR0;
				myCOM.IsPacketCorrupt = 1;
			}
			
			myCOM.PacketLength++;	// Increment Packet Length
				// Check if LoP counter is being used
			myCOM.LoP--; // DECREASE: LoP Count
			
			if ( myCOM.PacketLength > 29 ) {
				myCOM.LoP = 0;
			}
			
			
			
			if ( myCOM.PacketLength == 2 ) {	// Set LoP if this is the second byte
				if ( myCOM.PacketData[0] == myCOM.PacketData[1] ) {
					switch ( myCOM.PacketData[1] ) {	// Set Lop to the correct value
						case 0xF1:
						myCOM.LoP = 1;	// Two bytes left until LoP
						break;
						case 0xF6:	// ADDED Feature: Number of slat-durations to run for
						case 0xF4:
						case 0xF7:
						myCOM.LoP = 2;	// Two bytes left until LoP
						break;
						case 0xF5:
						myCOM.LoP = 28;		// Twenty Eight Bytes left until LoP
						break;
						default:	// If none of the above, reset and ignore
						myCOM.LoP = 2;		// Twenty Eight Bytes left until LoP
						
						
						break;
					}
				} else {
					//PORTC |= 0x01;	// YELLOW
					intRxDummy = UDR0;
					// PORTC |= 0x01;	// YELLOW
					usartCleanPacket();
					mySys.chMode = SYSTEM_MODE_DATA_WAIT;	//	Finish Transmission and go back to idling
				}
			} 
			
			if ( myCOM.LoP == 0 && myCOM.PacketLength > 2 && myCOM.IsPacketCorrupt == 0) {
				UCSR0B = 0b10001000;	// Rx Disabled. Rx Interrupt Enabled
				mySys.chMode = SYSTEM_MODE_PROCESS;
				myCOM.PacketRxStatus = 2;	// Packet Rx Complete
			}
			
			// Reset main clock in order to avoid any overflow drama
			mySys.clkCount = 0;		// Reset clock
			
			if ( myCOM.PacketLength == 1 ) {
				if ( myCOM.PacketData[0] < 0xF0 || myCOM.PacketData[0] == 0xFE ) {
					intRxDummy = UDR0;
					// PORTC |= 0x01;	// YELLOW
					
					// End of action, reset everything
					mySys.clkCount = 0;
					usartCleanPacket();
					mySys.chMode = SYSTEM_MODE_DATA_WAIT;	//	Finish Transmission and go back to idling
					UCSR0C = 0b00110110;
					UCSR0B = 0b10011000;	// Tx/Rx Enabled. Rx Interrupt Enabled
				}
			}
		} // Timeout omitted, might have to be placed on timer
	}	
	//PORTC &= ~(0x01);
}


//////////////////////////////////////////////////////////////////////////
//	PCINT Interrupt Service Routine
//////////////////////////////////////////////////////////////////////////
volatile uint8_t portchistory = 0x00;     // default is LOW
ISR ( PCINT1_vect )
{
	uint8_t changedbits;
	uint8_t holder;
	
	holder = PINC;
	changedbits = holder & 0b00100000;
	changedbits ^= portchistory;
	portchistory = holder;
	portchistory &= 0b00100000;
	
	if(changedbits & (1 << PINC5)) {
		/* PCINT13 changed */
		if( ( portchistory & (1 << PINC5 ) ) ) {   /// ####### CONTROLS: Divert 90-Degrees
			// Low to High Transition
			// Start to transmit the correct protocol until timed out
			
			if ( mySys.chMode == SYSTEM_MODE_PHOTOEYE ) {
				//////////////////////////////////////////////////////////////////////////
				// *** The following are changes to the command as we make progress on duration
				
				myDivert.PhotoeyeTripped = 0xFF;
			}
		}
	}
	
	//PORTC |= 0x01;
	//
	//int i;
	//uint8_t *ramMemMap;
	//ramMemMap = &myCOM.PacketRxStatus;
	//for ( i=0; i < 0x80; i++ ) {
		//eeprom_write_byte(eepromDEVICE_MEM_DUMP+i, *ramMemMap);
		//ramMemMap++;
	//}
	//
	
}

//////////////////////////////////////////////////////////////////////////
// Main Timer ISR
//////////////////////////////////////////////////////////////////////////
// 1kHz Timer ISR
ISR( TIMER0_COMPA_vect ) {
	mySys.clkCount++;
	
	// Data Timeout
	if ( myCOM.PacketRxStatus == 1 ) {	// Determine if we are taking data timeout	(Currently Receiving a Packet)
		if ( ( ( uint8_t ) ( mySys.clkCount ) ) > ( ( uint8_t ) systemBYTE_TIMEOUT ) ) {	// If data has been timed out
			myCOM.DroppedPackets++;		// Add to the Dropped Packet Count
			//eeprom_write_word(eepromCOM_RX_DEBUG + 3, ( ( uint8_t ) ( mySys.clkCount ) ));
			//eeprom_write_byte(eepromCOM_RX_DEBUG + 9, 0xDD );
			usartCleanPacket();
			mySys.chMode = SYSTEM_MODE_DATA_WAIT;	//	Finish Transmission and go back to idling
			UCSR0B = 0b10011000;	// Rx Enabled. Rx Interrupt Enabled
		}
	}
}


//////////////////////////////////////////////////////////////////////////
//	Setup Function
//////////////////////////////////////////////////////////////////////////
void setup( void ) {
	uint8_t netStatHolder;
	
	//eeprom_write_byte(eepromCOM_RX_A + 0, 0x00);
	//eeprom_write_byte(eepromCOM_RX_A + 1, 0x00);
	//eeprom_write_byte(eepromCOM_RX_A + 2, 0x00);
	//eeprom_write_byte(eepromCOM_RX_A + 3, 0x00);
	//eeprom_write_byte(eepromCOM_RX_RAW, 0x00);
	//eeprom_write_byte(eepromCOM_RX_RAW + 1, 0x00);
	//eeprom_write_byte(eepromCOM_RX_RAW + 2, 0x00);
	//eeprom_write_byte(eepromCOM_RX_RAW + 3, 0x00);
	//eeprom_write_byte(eepromCOM_RX_RAW + 4, 0x00);
	//eeprom_write_byte(eepromCOM_RX_RAW + 5, 0x00);
	//eeprom_write_byte(eepromCOM_RX_RAW + 6, 0x00);
	//eeprom_write_byte(eepromCOM_RX_RAW + 7, 0x00);
	//eeprom_write_byte(eepromCOM_RX_RAW + 8, 0x00);
	//eeprom_write_byte(eepromCOM_RX_RAW + 9, 0x00);
	//eeprom_write_byte(eepromCOM_RX_RAW + 10, 0x00);
	//eeprom_write_byte(eepromCOM_RX_RAW + 11, 0x00);
	//eeprom_write_byte(eepromCOM_RX_RAW + 12, 0x00);
	//eeprom_write_byte(eepromCOM_RX_RAW + 13, 0x00);
	//eeprom_write_byte(eepromCOM_RX_RAW + 14, 0x00);
	//eeprom_write_byte(eepromCOM_RX_RAW + 15, 0x00);
	//eeprom_write_byte(eepromCOM_RX_RAW + 16, 0x00);
	//eeprom_write_byte(eepromCOM_RX_CORRUPT, 0x00);
	//eeprom_write_byte(eepromCOM_RX_DROPPED, 0x00);
	//eeprom_write_byte(eepromCOM_RX_SUCCESS, 0x00);
	//eeprom_write_byte(eepromCOM_RX_COUNT, 0x00);
	//eeprom_write_byte(eepromCOM_RX_COUNT+5, 0xFF);
	
	// Restart Network Statistics
	netStatHolder = eeprom_read_byte(eepromCOM_RX_CORRUPT);
	if ( netStatHolder == 0xFF ) {
		eeprom_write_byte(eepromCOM_RX_CORRUPT, 0x00);
	}
	netStatHolder = eeprom_read_byte(eepromCOM_RX_DROPPED);
	if ( netStatHolder == 0xFF ) {
		eeprom_write_byte(eepromCOM_RX_DROPPED, 0x00);
	}
	netStatHolder = eeprom_read_byte(eepromCOM_RX_SUCCESS);
	if ( netStatHolder == 0xFF ) {
		eeprom_write_byte(eepromCOM_RX_SUCCESS, 0x00);
	}
	netStatHolder = eeprom_read_byte(eepromCOM_RX_COUNT);
	if ( netStatHolder == 0xFF ) {
		eeprom_write_byte(eepromCOM_RX_COUNT, 0x00);
	}
	
	
	// System Variable Initialization
	mySys.clkCount = 0;	// Back to zero counts
	mySys.devAddress = 0;
	mySys.chMode = SYSTEM_MODE_DATA_WAIT;
	mySys.chTran = BOARD_TYPE_DIVERT;
	
	// Communication Variable Intialization
	myCOM.CorruptPackets = 0;
	myCOM.DroppedPackets = 0;
	myCOM.RxPackets = 0;		// Reset Network Statistics
	myCOM.PreviousCorruptPackets = 0;
	myCOM.PreviousDroppedPackets = 0;
	myCOM.PreviousRxPackets = 0;
	usartCleanPacket();
	
	// Determine the type of transmitter that this is supposed to be
		// Get the code number of the swtiches on board
	DDRC = 0b00000000;	// All as inputs
	PORTC = 0x0F;		// Set the pull-up resistors
	mySys.devAddress = PINC & 0x0F;		// Obtain the device address selected
	
	//////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////
	//mySys.devAddress = 0x02;
	//PORTC = 0;
	//////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////
	// Store the address for debugging purposes
	eeprom_write_byte(eepromDEVICE_ID, mySys.devAddress);
	// Populate the Transmitter Mode
	if ( mySys.devAddress > 0 ) {
		mySys.chTran = BOARD_TYPE_DIVERT;	 // This is a Diverting Transmitter
	} else {
		mySys.chTran = BOARD_TYPE_PARAMETER;	 // This is a Programmer Transmitter
	}
	
	//////////////////////////////////////////////////////////////////////////
	// CONVYOR SPEED and TIMING SETUP AND AUTO-INITIALIZATION
	//////////////////////////////////////////////////////////////////////////
	mySys.ConveyorSpeedByte = eeprom_read_byte(eepromSYS_CONVEYOR_SPEED);
	if ( mySys.ConveyorSpeedByte == 0xFF ) {	// Auto Initialization to Default Values 
		mySys.ConveyorSpeedByte = 0x30;	// Initialize the right speed
		eeprom_write_byte(eepromSYS_CONVEYOR_SPEED, mySys.ConveyorSpeedByte);	// Store Default
	}
	//mySys.ConveyorSpeedByte = 0x00;	// Initialize the right speed
	mySys.ConveyorSpeed = ( mySys.ConveyorSpeedByte * 5 ) + 60;	// Obtain on ft/min
	
	myDivert.SlatTravelTimes[0] = ( uint16_t ) ( ( ( float ) mySys.ConveyorSpeed ) * ( ( float ) mathTO_MM_SEC ) );
	myDivert.SlatTravelTimes[0] = ( uint16_t ) ( ( ( uint32_t ) mathSLAT_WIDTH ) / ( ( uint32_t ) myDivert.SlatTravelTimes[0] ) );
	for ( myDivert.i=1;myDivert.i<7;myDivert.i++ ) {
		myDivert.SlatTravelTimes[myDivert.i] = myDivert.SlatTravelTimes[0] * (1 + myDivert.i);
	}
	eeprom_write_word(eepromSYS_CONVEYOR_SPEED+5,myDivert.SlatTravelTimes[0]);	// Debug
	//myDivert.ProgrammingDuration = ( ( uint32_t ) 70 * ( uint32_t ) myDivert.SlatTravelTimes[0] );
	myDivert.ProgrammingDuration = ( ( uint32_t ) 300 * ( uint32_t ) myDivert.SlatTravelTimes[0] );
	myDivert.DivertDirection = 0x00;	// Default Divert Direction
	
	myCOM.ConveyorSpeed = mySys.ConveyorSpeedByte;
	myCOM.RURD = 0b01010000;			// These are Constants
	myCOM.DivertDelay = 0b01100111;		// This is a Constant-Minimum
	mySys.PhotoeyeDelayWindow = 300;	// Delay of 300ms
	
		// Diverting Table Values ( Auto Settings to default )
	for ( myDivert.i=0;myDivert.i<28;myDivert.i++ ) {
		myDivert.tblDivertData[myDivert.i] = eeprom_read_byte(eepromDYV_TABLE_VALUES + myDivert.i );
		if ( myDivert.tblDivertData[myDivert.i] == 0xFF ) {	
			// Overwrite Byte if not initialized
			myDivert.tblDivertData[myDivert.i] = dataDEFAULT_COMMAND;
			eeprom_write_byte( eepromDYV_TABLE_VALUES + myDivert.i, dataDEFAULT_COMMAND);
		}
	}
	
	mySys.PhotoeyeDelayWindow = eeprom_read_word(eepromPEYE_TIMEOUT);
	if ( mySys.PhotoeyeDelayWindow == ( uint16_t ) 0xFFFF || mySys.PhotoeyeDelayWindow == ( uint16_t ) 0x0000 ) {
		// Reset the value to the default 300ms
		mySys.PhotoeyeDelayWindow = 300;	// Default value
		eeprom_write_word(eepromPEYE_TIMEOUT, mySys.PhotoeyeDelayWindow);
	}
	
	// Packet info initialization
	myDivert.irPacketLength = 0;
	myDivert.irSendPacket = 0xFF; // Yes, ready to send packet
	
	//////////////////////////////////////////////////////////////////////////
	//	# System Timer Setup ( Timer 0 as a 1kHz Interrupt )
	//////////////////////////////////////////////////////////////////////////
	TCNT0=0x00;
	OCR0A = 249;
	TCCR0A = 0x03;
	TCCR0B |= 0x0B;
	TIMSK0 |= 0x0A;		// End of timer 0 Setup
	//////////////////////////////////////////////////////////////////////////
	
	//////////////////////////////////////////////////////////////////////////
	//	PWM Setup
	//////////////////////////////////////////////////////////////////////////
	DDRB |= 0b00000101;
	ICR1 = 285;
	TCCR1A = 0b00100010;
	TCCR1B = 0b00011001;
	OCR1B = 143;	// 50 % Duty Cycle
	//////////////////////////////////////////////////////////////////////////
	
	//////////////////////////////////////////////////////////////////////////
	// SERIAL Communication Setup
	//////////////////////////////////////////////////////////////////////////
	PORTB &= 0b11111110;
	DDRD |= 0b00000010;
	DDRD &= 0b11111110;
	UBRR0H = (uint8_t)(BAUD_PRESCALLER>>8);
	UBRR0L = (uint8_t)(BAUD_PRESCALLER);
	
	UCSR0C = 0b00110110;
	UCSR0B = 0b10011000;	// Tx/Rx Enabled. Rx Interrupt Enabled
	/////////////////////////// END SERIAL COMMUNICATION SETUP ////////////////
	
	//////////////////////////////////////////////////////////////////////////
	// PCINT ISR SETUP
	//////////////////////////////////////////////////////////////////////////
	PCICR |= ( 1 << PCIE1 );	// Set PCIE0 to enable PCMSK0 scan
	PCMSK1 |= ( 1 << PCINT13 );	// Set the PCINT0 to trigger an interrupt on state change
	/////////////////////////// END PCINT ISR SETUP //////////////////////////
	
	// Enable interrupts
	sei();
}

//////////////////////////////////////////////////////////////////////////
// USART Utility: Packet Information Resetter
//////////////////////////////////////////////////////////////////////////
void usartCleanPacket( void ) {
	myCOM.PacketRxStatus = 0;	// We are waiting for data/packet
	myCOM.IsPacketCorrupt = 0;	// Packet is not corrupt
	myCOM.PacketLength = 0;		// Reset Packet data: No bytes on current packet
	myCOM.LoP = 0xFF;				// Reset Packet data: Back to DC on Last-Of-Packet
	myCOM.TimeoutTickCount = 0;	// Reset the Timeout Counter
	myCOM.PacketRxStatus = 0;	// We are waiting for data/packet
	
}

//////////////////////////////////////////////////////////////////////////
//	Utility Network STATISTICS
//////////////////////////////////////////////////////////////////////////
void usartNetworkStatistics( void ) {
	char extraCount;
	
	if ( myCOM.CorruptPackets > myCOM.PreviousCorruptPackets ) {
		extraCount = eeprom_read_byte(eepromCOM_RX_CORRUPT);
		extraCount += myCOM.CorruptPackets - myCOM.PreviousCorruptPackets;
		eeprom_write_byte(eepromCOM_RX_CORRUPT, extraCount);
		myCOM.CorruptPackets = 0;
		myCOM.PreviousCorruptPackets = myCOM.CorruptPackets;
	}
	
	if ( myCOM.DroppedPackets > myCOM.PreviousDroppedPackets ) {
		extraCount = eeprom_read_byte(eepromCOM_RX_DROPPED);
		extraCount += myCOM.DroppedPackets - myCOM.PreviousDroppedPackets;
		eeprom_write_byte(eepromCOM_RX_DROPPED, extraCount);
		myCOM.DroppedPackets = 0;
		myCOM.PreviousDroppedPackets = myCOM.DroppedPackets;
	}
	
	if ( myCOM.RxPackets > myCOM.PreviousRxPackets ) {
		extraCount = eeprom_read_byte(eepromCOM_RX_SUCCESS);
		extraCount += myCOM.RxPackets - myCOM.PreviousRxPackets;
		eeprom_write_byte(eepromCOM_RX_SUCCESS, extraCount);
		myCOM.RxPackets = 0;
		myCOM.PreviousRxPackets = myCOM.RxPackets;
	}
}

void fcnChipStop(void) {
	UCSR0C = 0b00110110;
	UCSR0B = 0b00000000;	// Tx/Rx Enabled. Rx Interrupt Enabled
	
	PORTD &= 0b11111101;
}

/*
 Makibox A6 Firmware
 ---
 Copyright (c) 2012-2013 by Makible Limited.
 
 This file is part of the Makibox A6 Firmware.
 
 Makibox A6 Firmware is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 The Makibox A6 Firmware is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with the Makibox A6 Firmware.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <avr/io.h>

#include "../i2c/TWI_Master.h"
#include "../i2c/Master_I2C_Comms.h"
#include "../config.h"
#include "../pins_teensy.h"

#define TWI_GEN_CALL         0x00  // The General Call address is 0

// Sample TWI transmission commands
#define TWI_CMD_MASTER_WRITE 0x10
#define TWI_CMD_MASTER_READ  0x20

// Sample TWI transmission states, used in the main application.
#define SEND_DATA             0x01
#define REQUEST_DATA          0x02
#define READ_DATA_FROM_BUFFER 0x03

static unsigned char I2C_messageBuf[TWI_BUFFER_SIZE];
static unsigned char I2C_ReadmessageBuf[TWI_BUFFER_SIZE];
unsigned char Send_I2C_Msg = 0, I2C_Read_Request = 0;
unsigned char I2C_Send_Msg_Size = 0;
static unsigned char TWI_targetSlaveAddress = 0;
static unsigned char TWI_operation = 0;
unsigned char I2C_Locked = 1;
unsigned long previous_millis_service_i2c = 0;

unsigned char TWI_Act_On_Failure_In_Last_Transmission ( unsigned char TWIerrorMsg );
void Process_I2C_Message(unsigned char I2C_messageBuf[TWI_BUFFER_SIZE]);


unsigned char TWI_Act_On_Failure_In_Last_Transmission ( unsigned char TWIerrorMsg )
{
	// A failure has occurred, use TWIerrorMsg to determine the nature of the failure
	// and take appropriate actions.
	// See header file for a list of possible failures messages.
	
	// Here is a simple sample, where if received a NACK on the slave address,
	// then a retransmission will be initiated.
	
  if ( (TWIerrorMsg == TWI_MTX_ADR_NACK) | (TWIerrorMsg == TWI_MRX_ADR_NACK) )
    TWI_Start_Transceiver();
    
  return TWIerrorMsg; 
}


void init_I2C_Master(void)
{
	I2C_Locked = 1;
	
	// Init pins to output with open-drain
	DDRD |= (1 << PIND0) | (1 << PIND1);
	PORTD |= ( (1 << PIND0) | (1 << PIND1) );
	
	TWI_Master_Initialise();
	
	// Perform SW I2C reset to ensure all devices are in a known I2C state
	//I2C_SW_Reset();
	
	I2C_Send_Msg_Size = 0;
	I2C_Read_Request = 0;
	Send_I2C_Msg = 0;
	I2C_Locked = 0;
}


void Service_I2C_Master(void)
{
	if (millis() - previous_millis_service_i2c >= SERVICE_I2C_INTERVAL)
	{
		if (Send_I2C_Msg)
		{	
			if ( ! TWI_Transceiver_Busy() )
			{
				TWI_Start_Transceiver_With_Data( I2C_messageBuf, I2C_Send_Msg_Size);
				
				I2C_Send_Msg_Size = 0;
				Send_I2C_Msg = 0;
				I2C_Locked = 0;
			}
		}
		
		
		if ( (I2C_Read_Request) && (!Send_I2C_Msg) )
		{
			if ( ! TWI_Transceiver_Busy() )
			{
				TWI_Start_Transceiver_With_Data( I2C_ReadmessageBuf, I2C_ReadmessageBuf[3] + I2C_MSG_HEADER_SIZE);
				
				TWI_operation = REQUEST_DATA;
				
				I2C_Read_Request = 0;
			}
		}
		
		while ( TWI_Transceiver_Busy() );
		
		if ( ! TWI_Transceiver_Busy() )
		{
			// Check if the last operation was successful
			if ( TWI_statusReg.lastTransOK )
			{
				if ( TWI_operation ) // Section for follow-up operations.
				{
				// Determine what action to take now
				  if (TWI_operation == REQUEST_DATA)
				  { // Request/collect the data from the Slave
					I2C_ReadmessageBuf[0] = (TWI_targetSlaveAddress<<TWI_ADR_BITS) | (TRUE<<TWI_READ_BIT); // The first byte must always consist of General Call code or the TWI slave address.
					TWI_Start_Transceiver_With_Data( I2C_ReadmessageBuf, TWI_BUFFER_SIZE );       
					TWI_operation = READ_DATA_FROM_BUFFER; // Set next operation        
				  }
				  
				  while ( TWI_Transceiver_Busy() );
				  
				  if (TWI_operation == READ_DATA_FROM_BUFFER)
				  { // Get the received data from the transceiver buffer
					if ( TWI_Get_Data_From_Transceiver( I2C_ReadmessageBuf, TWI_BUFFER_SIZE ) )
					{
						Process_I2C_Message(I2C_ReadmessageBuf);
					}
					else
					{
						//serial_send("// I2C Debug: Get data from transceiver failed\r\n");
					}
					
					TWI_operation = FALSE;        // Set next operation
					I2C_Locked = 0;
				  }
				}
			}
			else // Got an error during the last transmission
			{
				// Use TWI status information to detemine cause of failure and take appropriate actions. 
				TWI_Act_On_Failure_In_Last_Transmission( TWI_Get_State_Info( ) );
			}
		}
		
		previous_millis_service_i2c = millis();
	}
}


void Process_I2C_Message(unsigned char I2C_ReadmessageBuf[TWI_BUFFER_SIZE])
{	
	switch (I2C_ReadmessageBuf[1])
	{/*
		case I2C_READ_E1_CURRENT_TEMP:
			e1_current_temp = (I2C_ReadmessageBuf[4] << 8) | I2C_ReadmessageBuf[3];
		break;
		
		
		case I2C_READ_E2_CURRENT_TEMP:
			e2_current_temp = (I2C_ReadmessageBuf[4] << 8) | I2C_ReadmessageBuf[3];
		break;
		
		
		case I2C_READ_ALL_E_TEMP_DUTY:
			e1_current_temp = (I2C_ReadmessageBuf[4] << 8) | I2C_ReadmessageBuf[3];
			e1_heater_duty = I2C_ReadmessageBuf[5];
			
			e2_current_temp = (I2C_ReadmessageBuf[7] << 8) | I2C_ReadmessageBuf[6];
			e2_heater_duty = I2C_ReadmessageBuf[8];
			
			e1_target_temp = (I2C_ReadmessageBuf[10] << 8) | I2C_ReadmessageBuf[9];
			e2_target_temp = (I2C_ReadmessageBuf[12] << 8) | I2C_ReadmessageBuf[11];
		break;
		
		
		case I2C_READ_E1_TARGET_TEMP:
			e1_target_temp = (I2C_ReadmessageBuf[4] << 8) | I2C_ReadmessageBuf[3];
		break;
		
		
		case I2C_READ_E2_TARGET_TEMP:
			e2_target_temp = (I2C_ReadmessageBuf[4] << 8) | I2C_ReadmessageBuf[3];
		break;
		*/
		
		default:
			// Unknown message
		break;
	}
}


void I2C_Send_Msg(unsigned char Slave_Address, unsigned char command, unsigned char NumofDataBytes, unsigned long Data)
{
	unsigned char i;
	
	if (!I2C_Locked)
	{
		I2C_Locked = 1;
		
		TWI_targetSlaveAddress = Slave_Address;
		
		// The first byte must always consit of General Call code or the TWI slave address.
		I2C_messageBuf[0] = (TWI_targetSlaveAddress<<TWI_ADR_BITS) | (FALSE<<TWI_READ_BIT);
		I2C_messageBuf[1] = TWI_CMD_MASTER_WRITE;
		I2C_messageBuf[2] = command;
		
		if (NumofDataBytes > 4)
		{
			NumofDataBytes = 4;
		}
		
		I2C_messageBuf[3] = NumofDataBytes;
		
		for (i = 0; i < NumofDataBytes; i++)
		{
			I2C_messageBuf[4 + i] = (unsigned char)( ( Data >> (8 * i) ) & (0x00FF) );
		}
		
		Send_I2C_Msg = 1;
	}
}


void I2C_Read_Msg(unsigned char Slave_Address, unsigned char command, unsigned char NumofDataBytes, unsigned long Data)
{
	unsigned char i;
	
	if (!I2C_Locked)
	{
		I2C_Locked = 1;
		
		TWI_targetSlaveAddress = Slave_Address;
		
		// The first byte must always consit of General Call code or the TWI slave address.
		I2C_ReadmessageBuf[0] = (TWI_targetSlaveAddress<<TWI_ADR_BITS) | (FALSE<<TWI_READ_BIT);
		
		I2C_ReadmessageBuf[1] = TWI_CMD_MASTER_READ;
		I2C_ReadmessageBuf[2] = command;
		
		if (NumofDataBytes > 4)
		{
			NumofDataBytes = 4;
		}
		
		I2C_ReadmessageBuf[3] = NumofDataBytes;
		
		for (i = 0; i < NumofDataBytes; i++)
		{
			I2C_ReadmessageBuf[4 + i] = (unsigned char)( ( Data >> (8 * i) ) & (0x00FF) );
		}
		
		I2C_Read_Request = 1;
	}
}


void I2C_SW_Reset(void)
{
	TWCR =	(1<<TWEN)|                         		// TWI Interface enabled.
			(1<<TWIE)|(1<<TWINT)|                  	// Enable TWI Interupt and clear the flag.
			(0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|       	// Initiate a START condition.
			(0<<TWWC);     

	TWDR =	0xFF;
	
	TWCR =	(1<<TWEN)|                                 // TWI Interface enabled
            (1<<TWIE)|(1<<TWINT)|                      // Enable TWI Interupt and clear the flag to send byte
            (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           //
            (0<<TWWC);   
	
	TWCR =	(1<<TWEN)|                              // TWI Interface enabled
            (0<<TWIE)|(1<<TWINT)|                   // Disable TWI Interrupt and clear the flag
            (0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|        // Initiate a START condition.
            (0<<TWWC); 
			
	TWCR =	(1<<TWEN)|                              // TWI Interface enabled
            (0<<TWIE)|(1<<TWINT)|                   // Disable TWI Interrupt and clear the flag
            (0<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|        // Initiate a STOP condition.
            (0<<TWWC);                                 
}


void I2C_digipots_set_defaults(void)
{
	if (!I2C_Locked)
	{
		I2C_Locked = 1;
		
		TWI_targetSlaveAddress = I2C_DIGIPOT_ADDRESS;
		
		I2C_messageBuf[0] = (TWI_targetSlaveAddress) | (FALSE<<TWI_READ_BIT);	// Slave Address | Write bit = 0
		
		I2C_messageBuf[1] = I2C_DIGIPOT_VOL_WIPER0_ADDR | I2C_DIGIPOT_WRITE;
		I2C_messageBuf[2] = DIGIPOT_XAXIS_DEFAULT;
		
		I2C_messageBuf[3] = I2C_DIGIPOT_VOL_WIPER1_ADDR | I2C_DIGIPOT_WRITE;
		I2C_messageBuf[4] = DIGIPOT_YAXIS_DEFAULT;
		
		I2C_messageBuf[5] = I2C_DIGIPOT_VOL_WIPER2_ADDR | I2C_DIGIPOT_WRITE;
		I2C_messageBuf[6] = 58; //0.6V dev testing value //DIGIPOT_ZAXIS_DEFAULT;
		
		I2C_messageBuf[7] = I2C_DIGIPOT_VOL_WIPER3_ADDR | I2C_DIGIPOT_WRITE;
		I2C_messageBuf[8] = 58; //0.6V dev testing value //DIGIPOT_EAXIS_DEFAULT;
		
		I2C_messageBuf[9] = I2C_DIGIPOT_VOL_TCON0_ADDR | I2C_DIGIPOT_WRITE | 0x01;
		I2C_messageBuf[10] = 0xFF;
		
		I2C_messageBuf[11] = I2C_DIGIPOT_VOL_TCON1_ADDR | I2C_DIGIPOT_WRITE | 0x01;
		I2C_messageBuf[12] = 0xFF;
		
		I2C_Send_Msg_Size = 13;
		Send_I2C_Msg = 1;
	}
}
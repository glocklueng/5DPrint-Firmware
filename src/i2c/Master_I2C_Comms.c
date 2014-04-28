/*
  5DPrint Firmware
  Designed for Printrboard (Rev B) and 5DPrint D8 Driver Board.
  ---
  Copyright (c) 2012-2014 by Makible Limited.
 
  This file is part of the 5DPrint Firmware.
 
  5DPrint Firmware is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  The 5DPrint Firmware is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with the 5DPrint Firmware.  If not, see <http://www.gnu.org/licenses/>.
*/
/**
   \file Master_I2C_Comms.c
   \brief Handles I2C communication for the Digipot
   
 */

#include <avr/io.h>

#include "../i2c/TWI_Master.h"
#include "../i2c/Master_I2C_Comms.h"
#include "../config.h"
#include "../pins_teensy.h"
#include "../stepper.h"

#if DIGIPOTS > 0

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
unsigned char I2C_Send_Msg_Size = 0, I2C_Read_Req_Size = 0, I2C_Read_Msg_Size = 0;
static unsigned char TWI_targetSlaveAddress = 0;
static unsigned char TWI_operation = 0;
unsigned char I2C_Locked = 1;
unsigned long previous_millis_service_i2c = 0;

unsigned char TWI_Act_On_Failure_In_Last_Transmission ( unsigned char TWIerrorMsg );
void Process_I2C_Message(unsigned char I2C_messageBuf[TWI_BUFFER_SIZE]);
unsigned char currentToWiperValue(unsigned short current);

/**
   \fn unsigned char TWI_Act_On_Failure_In_Last_Transmission ( unsigned char TWIerrorMsg)
   \brief
 */
unsigned char TWI_Act_On_Failure_In_Last_Transmission ( unsigned char TWIerrorMsg ){
    // A failure has occurred, use TWIerrorMsg to determine the nature of the failure
    // and take appropriate actions.
    // See header file for a list of possible failures messages.
	
    // Here is a simple sample, where if received a NACK on the slave address,
    // then a retransmission will be initiated.
	
    if ( (TWIerrorMsg == TWI_MTX_ADR_NACK) | (TWIerrorMsg == TWI_MRX_ADR_NACK) )
        TWI_Start_Transceiver();
    
    return TWIerrorMsg; 
}

/**
   \fn void init_I2C_Master(void)
   \brief
 */
void init_I2C_Master(void){
    I2C_Locked = 1;
	
    // Init pins to output with open-drain
    DDRD |= (1 << PIND0) | (1 << PIND1);
    PORTD |= ( (1 << PIND0) | (1 << PIND1) );
	
    TWI_Master_Initialise();
	
    // Perform SW I2C reset to ensure all devices are in a known I2C state
    I2C_SW_Reset();
    delay(1);
	
    I2C_Send_Msg_Size = 0;
    I2C_Read_Request = 0;
    Send_I2C_Msg = 0;
    I2C_Locked = 0;
}

/**
   \fn void Service_I2C_Master
   \brief
 */
void Service_I2C_Master(void){
    unsigned long I2C_Transceiver_Busy_Time = 0;
	
    if (millis() - previous_millis_service_i2c >= SERVICE_I2C_INTERVAL){
        if (Send_I2C_Msg){	
            if ( ! TWI_Transceiver_Busy() ) {
                TWI_Start_Transceiver_With_Data( I2C_messageBuf, I2C_Send_Msg_Size);
				
                I2C_Send_Msg_Size = 0;
                Send_I2C_Msg = 0;
                I2C_Locked = 0;
			}
		}
		
		
        if ( (I2C_Read_Request) && (!Send_I2C_Msg) ){
            if ( ! TWI_Transceiver_Busy() ){
                TWI_Start_Transceiver_With_Data( I2C_ReadmessageBuf, I2C_Read_Req_Size);
				
                TWI_operation = REQUEST_DATA;
				
                I2C_Read_Req_Size = 0;
                I2C_Read_Request = 0;
			}
		}
		
        I2C_Transceiver_Busy_Time = millis();
		
        while ( TWI_Transceiver_Busy() 
                && ( millis() - I2C_Transceiver_Busy_Time < I2C_TRANSCEIVER_BUSY_TIMEOUT ) );
        
        if ( ! TWI_Transceiver_Busy() ){
            // Check if the last operation was successful
            if ( TWI_statusReg.lastTransOK ){
                if ( TWI_operation ){ // Section for follow-up operations.
                    // Determine what action to take now
                    if (TWI_operation == REQUEST_DATA){ // Request/collect the data from the Slave
                            I2C_ReadmessageBuf[0] = (TWI_targetSlaveAddress) | (TRUE<<TWI_READ_BIT); // The first byte must always consist of General Call code or the TWI slave address.
                            TWI_Start_Transceiver_With_Data( I2C_ReadmessageBuf, TWI_BUFFER_SIZE );       
                            TWI_operation = READ_DATA_FROM_BUFFER; // Set next operation        
                    }
                    
                    I2C_Transceiver_Busy_Time = millis();
                    
                    while ( TWI_Transceiver_Busy() 
                            && ( millis() - I2C_Transceiver_Busy_Time < I2C_TRANSCEIVER_BUSY_TIMEOUT ) );
                    
                    if (TWI_operation == READ_DATA_FROM_BUFFER){ 
                        // Get the received data from the transceiver buffer
                        if ( TWI_Get_Data_From_Transceiver( I2C_ReadmessageBuf, I2C_Read_Msg_Size ) )
                            Process_I2C_Message(I2C_ReadmessageBuf);
                        //else{
                        //serial_send("// I2C Debug: Get data from transceiver failed\r\n");
                        //}
                        
                        TWI_operation = FALSE;        // Set next operation
                        I2C_Locked = 0;
                    }
				}
			}
            else{ // Got an error during the last transmission
                // Use TWI status information to detemine cause of failure and take appropriate actions. 
                TWI_Act_On_Failure_In_Last_Transmission( TWI_Get_State_Info( ) );
			}
		}
		
        previous_millis_service_i2c = millis();
	}
}

/**
   \fn void I2C_SW_Reset(void)
   \brief
 */
void Process_I2C_Message(unsigned char I2C_ReadmessageBuf[TWI_BUFFER_SIZE]){
    I2C_Read_Msg_Size = 0;
}

/**
   \fn void I2C_SW_Reset(void)
   \brief
 */
void I2C_SW_Reset(void){
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

/**
   \fn unsigned char currentToWiperValue (unsigned short current)
   \brief
 */
unsigned char currentToWiperValue (unsigned short current){
    unsigned short wiperValue;
    wiperValue = (unsigned short) (( (current/1000.0) * 8 * (stepper_sense_resistance/1000.0) ) 
                                  / DIGIPOT_VOLTS_PER_STEP);
    if (wiperValue > 0xFF) wiperValue = 0xFF;
    return (unsigned char) wiperValue;
}

/**
   \fn void I2C_digipots_set_defaults(void)
   \brief
 */
void I2C_digipots_set_defaults(void){
    if (!I2C_Locked){
            I2C_Locked = 1;
		
            TWI_targetSlaveAddress = I2C_DIGIPOT_ADDRESS;
		
            I2C_messageBuf[0] = (TWI_targetSlaveAddress) | (FALSE<<TWI_READ_BIT);	// Slave Address | Write bit = 0
		
            I2C_messageBuf[1] = I2C_DIGIPOT_VOL_WIPER0_ADDR | I2C_DIGIPOT_WRITE;
            I2C_messageBuf[2] = currentToWiperValue (max_x_motor_current);
				
            I2C_messageBuf[3] = I2C_DIGIPOT_VOL_WIPER1_ADDR | I2C_DIGIPOT_WRITE;
            I2C_messageBuf[4] = currentToWiperValue (max_y_motor_current);
				
            I2C_messageBuf[5] = I2C_DIGIPOT_VOL_WIPER2_ADDR | I2C_DIGIPOT_WRITE;
            I2C_messageBuf[6] = currentToWiperValue (max_z_motor_current);
				
            I2C_messageBuf[7] = I2C_DIGIPOT_VOL_WIPER3_ADDR | I2C_DIGIPOT_WRITE;
            I2C_messageBuf[8] = currentToWiperValue (max_e_motor_current);
				
            I2C_messageBuf[9] = I2C_DIGIPOT_VOL_TCON0_ADDR | I2C_DIGIPOT_WRITE | 0x01;
            I2C_messageBuf[10] = 0xFF; 
		
            I2C_messageBuf[11] = I2C_DIGIPOT_VOL_TCON1_ADDR | I2C_DIGIPOT_WRITE | 0x01;
            I2C_messageBuf[12] = 0xFF;
		
            I2C_Send_Msg_Size = 13;
            Send_I2C_Msg = 1;
	}
}

/**
   \fn void I2C_digipots_set_all_wipers(unsigned short MilliAmps0, unsigned short MilliAmps1, unsigned short MilliAmps2, unsigned short MilliAmps3)
   \brief 
 */
void I2C_digipots_set_all_wipers(unsigned short MilliAmps0,
                                 unsigned short MilliAmps1,
                                 unsigned short MilliAmps2,
                                 unsigned short MilliAmps3){
    if (!I2C_Locked){
        I2C_Locked = 1;
		
        TWI_targetSlaveAddress = I2C_DIGIPOT_ADDRESS;
		
        I2C_messageBuf[0] = (TWI_targetSlaveAddress) | (FALSE<<TWI_READ_BIT);	// Slave Address | Write bit = 0
		
        I2C_messageBuf[1] = I2C_DIGIPOT_VOL_WIPER0_ADDR | I2C_DIGIPOT_WRITE;
        I2C_messageBuf[2] = currentToWiperValue(MilliAmps0);
		
        I2C_messageBuf[3] = I2C_DIGIPOT_VOL_WIPER1_ADDR | I2C_DIGIPOT_WRITE;
        I2C_messageBuf[4] = currentToWiperValue(MilliAmps1);
		
        I2C_messageBuf[5] = I2C_DIGIPOT_VOL_WIPER2_ADDR | I2C_DIGIPOT_WRITE;
        I2C_messageBuf[6] = currentToWiperValue(MilliAmps2);
		
        I2C_messageBuf[7] = I2C_DIGIPOT_VOL_WIPER3_ADDR | I2C_DIGIPOT_WRITE;
        I2C_messageBuf[8] = currentToWiperValue(MilliAmps2);
		
        I2C_Send_Msg_Size = 9;
        Send_I2C_Msg = 1;
	}
}

/**
   \fn void I2C_digipots_set_wiper(unsigned char WiperAddr, unsigned short MilliAmps)
   \brief
 */
void I2C_digipots_set_wiper(unsigned char WiperAddr, 
                            unsigned short MilliAmps){
    if (!I2C_Locked){
        I2C_Locked = 1;
		
        TWI_targetSlaveAddress = I2C_DIGIPOT_ADDRESS;
		
        I2C_messageBuf[0] = (TWI_targetSlaveAddress) | (FALSE<<TWI_READ_BIT);	// Slave Address | Write bit = 0
		
        I2C_messageBuf[1] = WiperAddr | I2C_DIGIPOT_WRITE;
        I2C_messageBuf[2] = currentToWiperValue(MilliAmps);
		
        I2C_Send_Msg_Size = 3;
        Send_I2C_Msg = 1;
	}
}

/**
   \fn unsigned short I2C_digipots_read(unsigned char DeviceMemAddress)
   \brief
 */
unsigned short I2C_digipots_read(unsigned char DeviceMemAddress){
    unsigned long WaitForDigipotRead_Millis = 0;
	
    if (!I2C_Locked){
        I2C_Locked = 1;
		
        TWI_targetSlaveAddress = I2C_DIGIPOT_ADDRESS;
		
        I2C_messageBuf[0] = (TWI_targetSlaveAddress) | (FALSE<<TWI_READ_BIT);	// Slave Address | Write bit = 0
		
        I2C_messageBuf[1] = DeviceMemAddress | I2C_DIGIPOT_READ;
		
        I2C_Read_Req_Size = 2;
        I2C_Read_Msg_Size = 2;
        I2C_Read_Request = 1;
		
        WaitForDigipotRead_Millis = millis();
		
        // Wait for read result
        while ( ( millis() - WaitForDigipotRead_Millis < DIGIPOT_READ_RESULT_WAIT_TIMEOUT)
                && (I2C_Read_Msg_Size) )
            Service_I2C_Master();
		
        if (I2C_Read_Msg_Size) return (0xFFFF);
        else return ( (I2C_ReadmessageBuf[0] << 8) | I2C_ReadmessageBuf[1] ) ;
	}
    else return (0xFFFF);
}

#endif

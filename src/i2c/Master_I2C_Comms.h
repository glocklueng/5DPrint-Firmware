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

#ifndef MASTER_I2C_H
#define MASTER_I2C_H

#define I2C_DIGIPOT_ADDRESS			0x58 // TO BE CONFIMED!!! - Note the HVC/A0 pin

/***************************************
 * I2C Message Formatting:
 * -----------------------
 * Byte 0	Gen Call / Slave Address + Read/Write Bit
 * Byte 1	Bits[7:4] = Mem Addr; Bits[3:2] = Cmd; Bit[1] = X; Bit[0] = D8
 * Byte 2	Data
 * Byte 3	Bits[7:4] = Mem Addr; Bits[3:2] = Cmd; Bit[1] = X; Bit[0] = D8
 * Byte 4	Data
 * Byte 5	Bits[7:4] = Mem Addr; Bits[3:2] = Cmd; Bit[1] = X; Bit[0] = D8
 * Byte 6	Data
 * Byte 7	Bits[7:4] = Mem Addr; Bits[3:2] = Cmd; Bit[1] = X; Bit[0] = D8
 * Byte 8	Data
 ***************************************/

#define I2C_MSG_HEADER_SIZE			0

// MCP4451 DIGI-POT COMMANDS
// COMMAND BYTE FORMAT:		BIT		7	6	5	4	3	2	1	0
//									AD3	AD2	AD1	AD0	C1	C0	D9	D8
#define I2C_DIGIPOT_WRITE				0x00
#define I2C_DIGIPOT_READ				0x0C
#define I2C_DIGIPOT_INC					0x04
#define I2C_DIGIPOT_DEC					0x08

// MCP4451 DIGI-POT ADDRESSES
#define I2C_DIGIPOT_VOL_WIPER0_ADDR		0x00
#define I2C_DIGIPOT_VOL_WIPER1_ADDR		0x10
#define I2C_DIGIPOT_VOL_WIPER2_ADDR		0x60
#define I2C_DIGIPOT_VOL_WIPER3_ADDR		0x70

#define I2C_DIGIPOT_VOL_TCON0_ADDR		0x40
#define I2C_DIGIPOT_VOL_TCON1_ADDR		0xA0


#define SERVICE_I2C_INTERVAL				100			// ms
#define DIGIPOT_READ_RESULT_WAIT_TIMEOUT	2000		// ms
#define I2C_TRANSCEIVER_BUSY_TIMEOUT		3000		// ms

//extern unsigned char I2C_messageBuf[TWI_BUFFER_SIZE];
extern unsigned char Send_I2C_Msg;
extern unsigned char I2C_Send_Msg_Size;
extern unsigned char I2C_Read_Req_Size;
extern unsigned char I2C_Read_Msg_Size;
extern unsigned char I2C_Read_Request;
extern unsigned char I2C_Locked;
extern unsigned long previous_millis_service_i2c;

void init_I2C_Master(void);
void Service_I2C_Master(void);
void I2C_Send_Msg(unsigned char Slave_Address, unsigned char command, unsigned char NumofDataBytes, unsigned long Data);
void I2C_Read_Msg(unsigned char Slave_Address, unsigned char command, unsigned char NumofDataBytes, unsigned long Data);
void I2C_SW_Reset(void);
void I2C_digipots_set_defaults(void);
void I2C_digipots_set_wiper(unsigned char WiperAddr, 
                            unsigned short MilliAmps);
unsigned short I2C_digipots_read(unsigned char DeviceMemAddress);

#endif

/* spi.c
*
* This file contains basic low level SPI funtions.
*
* History:
* =======
*
* + 	xx NOV 2012		Author: JTK Wong (XTRONTEC Limited)
*		Created.
*/

#include <avr/interrupt.h>
#include "pins_teensy.h"
#include "spi.h"

/***************************************************
* spi_init(void)
*
* Initialises SPI with parameters defined in spi.h
****************************************************/
void spi_init(void)
{
	/** SPCR Register **/
	SPCR = (SPI_INT_EN << SPIE) | (SPI_EN << SPE) | (SPI_DATA_ORDER << DORD) 
			| (SPI_MASTER_EN << MSTR) | (SPI_CLKPOL << CPOL) | (SPI_CLK_PHASE << CPHA) 
			| (SPI_CLK_PHASE << SPR1) | (SPI_CLKRATE0 << SPR0);
	
	SPSR |= (SPI_DBLRATE_EN << SPI2X);
}


/***************************************************
* spi_disable(void)
*
* Disables SPI.
****************************************************/
void spi_disable(void)
{
	SPCR &= ~(1 << SPE);
}


/***************************************************
* spi_SendByte(unsigned char DataByte)
*
* DataByte:	Byte of data to be sent
*
* Sends single byte of data on the SPI bus and waits
* until SPIF is set (serial transfer complete).
****************************************************/
void spi_SendByte(unsigned char DataByte)
{
	SPDR = DataByte;
    while ( !(SPSR & (1 << SPIF)) );
}


/***************************************************
* spi_ReceiveByte(void)
*
* Return:	Byte of data.
*
* Receives a single byte of data from the SPI bus.
****************************************************/
unsigned char spi_ReceiveByte(void)
{
	SPDR = 0x00;
    while ( !(SPSR & (1 << SPIF)) );
    return SPDR;
}


/***************************************************
* spi_TransferByte(unsigned char DataByte)
*
* DataByte: Byte of data to be sent
* Return: 	Byte of data received.
*
* Sends a byte of data and receives a byte of data
* at the same time.
****************************************************/
unsigned char spi_TransferByte(unsigned char DataByte)
{
	SPDR = DataByte;
    while ( !(SPSR & (1 << SPIF)) );
    return SPDR;
}

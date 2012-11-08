/* spi.h
*
*
* History:
* =======
*
* + 	xx NOV 2012		Author: JTK Wong (XTRONTEC Limited)
*		Created.
*/

#ifndef SPI_H
#define SPI_H


/** SPCR Register Bits **/
#define SPI_INT_EN			0	// SPI Interrupt Enable
#define SPI_EN				1	// SPI Enable
#define SPI_DATA_ORDER		1	// Data Transmit Order; 0 = MSB first; 1 = LSB first;
#define SPI_MASTER_EN		1	// Master / Slave Select; 0 = Slave; 1 = Master;
#define SPI_CLKPOL			0	// Clock Polarity; 0 = SCK is low when idle;
								// 1 = SCK is high when idle;
#define SPI_CLK_PHASE 		1	// Clock Phase; 0 = Data sampled on leading edge;
								// 1 = Data sampled on trailing edge;
#define SPI_CLKRATE1		0	// SPI Clock Rate Select
#define SPI_CLKRATE0		0	// SPI2X	SPR1	SPR0	SCK Freq.
								// 0		0		0		Fosc / 4
								// 0		0		1		Fosc / 16
								// 0		1		0		Fosc / 64
								// 0		1		1		Fosc / 128
								// 1		0		0		Fosc / 2
								// 1		0		1		Fosc / 8
								// 1		1		0		Fosc / 32
								// 1		1		1		Fosc / 64


/** SPSR Register Bits **/
#define SPI_DBLRATE_EN		0	// SPI SCK Double Rate Enable (in Master mode)

/** Function Prototypes **/
void spi_init(void);
void spi_disable(void);
void spi_SendByte(unsigned char DataByte);
unsigned char spi_ReceiveByte(void);
unsigned char spi_TransferByte(unsigned char DataByte);

#endif
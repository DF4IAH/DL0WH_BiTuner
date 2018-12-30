/**
 * \file
 *
 * \brief DAC MCP4921 driver.
 *
 (c) 2018 Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms,you may use this software and
    any derivatives exclusively with Microchip products.It is your responsibility
    to comply with third party license terms applicable to your use of third party
    software (including open source software) that may accompany Microchip software.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 */

/**
 * \addtogroup doc_driver_dac_mcp4921
 *
 * \section doc_driver_dac_mcp4921_rev Revision History
 * - v0.0.0.1 Initial Commit
 *
 *@{
 */
#include <dac_mcp4921.h>

// Control register values
#define CTRL_DACA 0x00
#define CTRL_UNBUFFERED 0x00
#define CTRL_BUFFERED 0x40
#define CTRL_GAIN_1X 0x20
#define CTRL_GAIN_2X 0x00
#define CTRL_ENABLED 0x10
#define CTRL_DISABLED 0x00

void MCP4921_select(void);
void MCP4921_deselect(void);

void DAC_0_init()
{

	/* Enable SPI */
	PRR &= ~(1 << PRSPI);

	SPCR = 1 << SPE                     /* SPI module enable: enabled */
	       | 0 << DORD                  /* Data order: disabled */
	       | 1 << MSTR                  /* Master/Slave select: enabled */
	       | 0 << CPOL                  /* Clock polarity: disabled */
	       | 0 << CPHA                  /* Clock phase: disabled */
	       | 0 << SPIE                  /* SPI interrupt enable: disabled */
	       | (0 << SPR1) | (0 << SPR0); /* SPI Clock rate selection: fosc/4 */

	// SPSR = (0 << SPI2X); /* Disable double SPI speed */
}

void DAC_0_cs_low()
{
	MCP4921_select();
}

void DAC_0_cs_high()
{
	MCP4921_deselect();
}

void DAC_0_Set_Value(uint16_t dacValue)
{
	uint8_t ctrl_byte = CTRL_DACA | CTRL_BUFFERED | CTRL_GAIN_1X | CTRL_ENABLED;
	uint8_t byte0     = ((uint8_t)(dacValue >> 8) & 0x0F) | (ctrl_byte & 0xF0);
	uint8_t byte1     = (uint8_t)(dacValue & 0x00FF);

	DAC_0_cs_low();

	SPDR = byte0;
	while (!(SPSR & (1 << SPIF)))
		;

	SPDR = byte1;
	while (!(SPSR & (1 << SPIF)))
		;

	DAC_0_cs_high();
	return;
}

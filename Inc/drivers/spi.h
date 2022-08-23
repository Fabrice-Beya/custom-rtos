/**
 ******************************************************************************
 * @file           	: spi.h
 * @author         	: Fabrice Beya
 * @brief          	: SPI driver header file
 * @created			: 22 August 2022
 ******************************************************************************
 ******************************************************************************
*
**/

#ifndef DRIVERS_SPI_H_
#define DRIVERS_SPI_H_

#include "utils.h"

#define SPI1_PCLK_EN()					(RCC->APB2ENR |= 1U << 12)
#define SPI2_PCLK_EN()					(RCC->APB1ENR |= 1U << 14)
#define SPI3_PCLK_EN()					(RCC->APB1ENR |= 1U << 15)
#define SPI4_PCLK_EN()					(RCC->APB1ENR |= 1U << 13)

typedef struct
{
	/* Master or Slave mode */
	uint8_t SPI_Device_mode;

	/* Full duplex, Half duplex or simplex */
	uint8_t SPI_BusConfig;

	/* Data Frame Format: 8bit or 16bit */
	uint8_t SPI_DFF;

	/* Clock phase: Failing edge/Rising edge */
	uint8_t SPI_CPHA;

	/* Clock polarity: Idle on low or high bit */
	uint8_t SPI_CPOL;

	/* Software slave management */
	uint8_t SPI_SSM;

	/* Clock speed */
	uint8_t SPI_SPEED;
}SPI_Config_t;

typedef struct
{
	/* Handle to spi peripheral */
	SPI_TypeDef *pSPI_TypeDef;

	/* Handle to the spi peripheral configuration */
	SPI_Config_t *pSPI_Config_t;
}SPI_Handle_t;

SPI_Handle_t* SPI_Init(SPI_Config_t* pSPI_Config_t);
int SPI_Transmit(SPI_Handle_t* pSPI_Handle_t, char *data, int size);
int SPI_Recieve(SPI_Handle_t* pSPI_Handle_t, char *data);
int SPI_Enable(SPI_Handle_t* pSPI_Handle_t);
int SPI_Disable(SPI_Handle_t* pSPI_Handle_t);


#endif /* DRIVERS_SPI_H_ */

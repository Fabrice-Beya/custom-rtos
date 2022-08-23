/**
 ******************************************************************************
 * @file           	: spi.h
 * @author         	: Fabrice Beya
 * @brief          	: GPIO driver header file
 * @created			: 22 August 2022
 ******************************************************************************
 ******************************************************************************
*
**/

#ifndef DRIVERS_GPIO_H_
#define DRIVERS_GPIO_H_

#include "utils.h"

#define GPIOA_PCLK_EN() 		RCC->AHBENR |= (1U << 17);
#define GPIOB_PCLK_EN() 		RCC->AHBENR |= (1U << 18);
#define GPIOC_PCLK_EN() 		RCC->AHBENR |= (1U << 19);
#define GPIOD_PCLK_EN() 		RCC->AHBENR |= (1U << 20);
#define GPIOE_PCLK_EN() 		RCC->AHBENR |= (1U << 21);
#define GPIOF_PCLK_EN() 		RCC->AHBENR |= (1U << 22);
#define GPIOG_PCLK_EN() 		RCC->AHBENR |= (1U << 23);

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin possible numbers
 */
#define GPIO_PIN_NO_0  				0
#define GPIO_PIN_NO_1  				1
#define GPIO_PIN_NO_2  				2
#define GPIO_PIN_NO_3  				3
#define GPIO_PIN_NO_4  				4
#define GPIO_PIN_NO_5  				5
#define GPIO_PIN_NO_6  				6
#define GPIO_PIN_NO_7  				7
#define GPIO_PIN_NO_8  				8
#define GPIO_PIN_NO_9  				9
#define GPIO_PIN_NO_10  			10
#define GPIO_PIN_NO_11 				11
#define GPIO_PIN_NO_12  			12
#define GPIO_PIN_NO_13 				13
#define GPIO_PIN_NO_14 				14
#define GPIO_PIN_NO_15 				15

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN 				0
#define GPIO_MODE_OUT 				1
#define GPIO_MODE_ALTFN 			2
#define GPIO_MODE_ANALOG 			3
#define GPIO_MODE_IT_FT     		4
#define GPIO_MODE_IT_RT     		5
#define GPIO_MODE_IT_RFT    		6

#define GPIO_PIN_HIGH    			1
#define GPIO_PIN_LOW    			0

typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

typedef struct
{
	GPIO_TypeDef *pGPIO_Port;
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_PinHandle_t;

void GPIO_PortEnable(GPIO_TypeDef *pGPIO_Port);
void GPIO_PinInit(GPIO_PinHandle_t *pGPIO_PinHandle);
void GPIO_TogglePin(GPIO_PinHandle_t *pGPIO_PinHandle);
uint8_t GPIO_ReadPin(GPIO_PinHandle_t *pGPIO_PinHandle);
void GPIO_WritePin(GPIO_PinHandle_t *pGPIO_PinHandle, uint8_t val);

#endif /* DRIVERS_GPIO_H_ */



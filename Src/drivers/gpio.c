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
#include "gpio.h"

void GPIO_PortEnable(GPIO_TypeDef *pGPIO_Port)
{
	if(pGPIO_Port == GPIOA){
		GPIOA_PCLK_EN();
	}
	else if(pGPIO_Port == GPIOB)
	{
		GPIOB_PCLK_EN();
	}
	else if(pGPIO_Port == GPIOC)
	{
		GPIOC_PCLK_EN();
	}
	else if(pGPIO_Port == GPIOD)
	{
		GPIOD_PCLK_EN();
	}
	else if(pGPIO_Port == GPIOE)
	{
		GPIOE_PCLK_EN();
	}
	else if(pGPIO_Port == GPIOF)
	{
		GPIOF_PCLK_EN();
	}
	else if(pGPIO_Port == GPIOG)
	{
		GPIOG_PCLK_EN();
	}
}

void GPIO_ConfigMode(GPIO_PinHandle_t *pGPIO_PinHandle)
{
	if(pGPIO_PinHandle->GPIO_PinConfig->GPIO_PinMode == GPIO_MODE_IN)
	{
		/* Mode = 00 */
		pGPIO_PinHandle->pGPIO_Port->MODER &= ~(1U << (2 * pGPIO_PinHandle->GPIO_PinConfig->GPIO_PinNumber + 1));
		pGPIO_PinHandle->pGPIO_Port->MODER &= ~(1U << (2 * pGPIO_PinHandle->GPIO_PinConfig->GPIO_PinNumber));
	}
	else if(pGPIO_PinHandle->GPIO_PinConfig->GPIO_PinMode == GPIO_MODE_OUT)
	{
		/* Mode = 01 */
		pGPIO_PinHandle->pGPIO_Port->MODER &= ~(1U << (2 * pGPIO_PinHandle->GPIO_PinConfig->GPIO_PinNumber + 1));
		pGPIO_PinHandle->pGPIO_Port->MODER |= (1U <<  (2 * pGPIO_PinHandle->GPIO_PinConfig->GPIO_PinNumber));
	}
	else if(pGPIO_PinHandle->GPIO_PinConfig->GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		/* Mode = 10 */
		pGPIO_PinHandle->pGPIO_Port->MODER |= (1U << (2 * pGPIO_PinHandle->GPIO_PinConfig->GPIO_PinNumber + 1));
		pGPIO_PinHandle->pGPIO_Port->MODER &= ~(1U <<(2 * pGPIO_PinHandle->GPIO_PinConfig->GPIO_PinNumber));
	}
	else if(pGPIO_PinHandle->GPIO_PinConfig->GPIO_PinMode == GPIO_MODE_ANALOG)
	{
		/* Mode = 11 */
		pGPIO_PinHandle->pGPIO_Port->MODER |= (1U << (2 * pGPIO_PinHandle->GPIO_PinConfig->GPIO_PinNumber + 1));
		pGPIO_PinHandle->pGPIO_Port->MODER |= (1U << (2 * pGPIO_PinHandle->GPIO_PinConfig->GPIO_PinNumber));
	}
}

void GPIO_ConfigAlt_Func(GPIO_PinHandle_t *pGPIO_PinHandle)
{

	// TODO
	uint8_t temp1, temp2;

	temp1 = pGPIO_PinHandle->GPIO_PinConfig->GPIO_PinNumber / 8;
	temp2 = pGPIO_PinHandle->GPIO_PinConfig->GPIO_PinNumber  % 8;
	pGPIO_PinHandle->pGPIO_Port->AFR[temp1] &= ~(0xF << ( 4 * temp2 ) ); //clearing
	pGPIO_PinHandle->pGPIO_Port->AFR[temp1] |= (pGPIO_PinHandle->GPIO_PinConfig->GPIO_PinAltFunMode << ( 4 * temp2 ) );

}

void GPIO_PinInit(GPIO_PinHandle_t *pGPIO_PinHandle)
{
	/*  Enable clock access to the GPIO port */
	GPIO_PortEnable(pGPIO_PinHandle->pGPIO_Port);

	/*  Configure the pin mode */
	GPIO_ConfigMode(pGPIO_PinHandle);

	/*  Configure the pin alternative function if applicable */
	if(pGPIO_PinHandle->GPIO_PinConfig->GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		GPIO_ConfigAlt_Func(pGPIO_PinHandle);
	}
}

uint8_t GPIO_ReadPin(GPIO_PinHandle_t *pGPIO_PinHandle)
{
	uint8_t val;
	val = pGPIO_PinHandle->pGPIO_Port->IDR && (1U << pGPIO_PinHandle->GPIO_PinConfig->GPIO_PinNumber);
	return val;
}

void GPIO_WritePin(GPIO_PinHandle_t *pGPIO_PinHandle, uint8_t val)
{
	if(val == GPIO_PIN_HIGH)
	{
		pGPIO_PinHandle->pGPIO_Port->ODR |= (1U << (pGPIO_PinHandle->GPIO_PinConfig->GPIO_PinNumber));
	} else {
		pGPIO_PinHandle->pGPIO_Port->ODR &= ~(1U << (pGPIO_PinHandle->GPIO_PinConfig->GPIO_PinNumber));
	}
}

void GPIO_TogglePin(GPIO_PinHandle_t *pGPIO_PinHandle)
{
	if(GPIO_ReadPin(pGPIO_PinHandle)){
		GPIO_WritePin(pGPIO_PinHandle, GPIO_PIN_LOW);
	} else {
		GPIO_WritePin(pGPIO_PinHandle, GPIO_PIN_HIGH);
	}
}



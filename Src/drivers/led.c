/*
 * led.c
 *
 *  Created on: Aug 19, 2022
 *      Author: fabricebeya
 */

#include "led.h"

GPIO_PinHandle_t LedGPIO;

void LED_Init(void)
{
	LedGPIO.pGPIO_Port = GPIOA;
	LedGPIO.GPIO_PinConfig.GPIO_PinNumber = 5,
	LedGPIO.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	LedGPIO.GPIO_PinConfig.GPIO_PinOPType = 0;
	LedGPIO.GPIO_PinConfig.GPIO_PinPuPdControl = 0;
	LedGPIO.GPIO_PinConfig.GPIO_PinSpeed = 0;

	GPIO_PinInit(&LedGPIO);

}

void LED_On()
{
	GPIO_WritePin(&LedGPIO, GPIO_PIN_HIGH);
}

void LED_Off()
{
	GPIO_WritePin(&LedGPIO, GPIO_PIN_LOW);
}

void LED_Toggle(){
	GPIO_TogglePin(&LedGPIO);
}


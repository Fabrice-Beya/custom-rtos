/*
 * led.c
 *
 *  Created on: Aug 19, 2022
 *      Author: fabricebeya
 */

#include "led.h"

void LED_Init(void)
{
// 	Enable Port A
	RCC->AHBENR |= (1U << 17);

//	Configure PA5 for output
	GPIOA->MODER |= (1U << 10);
	GPIOA->MODER &= ~(1U << 11);

//	GPIO_PinHandle_t LedGPIO;
//
//	LedGPIO.pGPIO_Port = GPIOA;
//	LedGPIO.GPIO_PinNumber = 5,
//	LedGPIO.GPIO_PinMode = GPIO_MODE_OUT;
//	LedGPIO.GPIO_PinSpeed = 0;
//	LedGPIO.GPIO_PinPuPdControl = 0;
//	LedGPIO.GPIO_PinOPType = 0;
//	LedGPIO.GPIO_PinAltFunMode = 0;
//
//	GPIO_PinInit(&LedGPIO);

}

void LED_On()
{
	GPIOA->ODR |= (1U << 5);
}

void LED_OFF()
{
	GPIOA->ODR &= ~(1U << 5);
}


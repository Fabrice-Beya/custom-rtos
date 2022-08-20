/*
 * led.c
 *
 *  Created on: Aug 19, 2022
 *      Author: fabricebeya
 */

#include "led.h"

void led_init(void)
{
// 	Enable Port A
	RCC->AHBENR |= (1U << 17);

//	Configure PA5 for output
	GPIOA->MODER |= (1U << 10);
	GPIOA->MODER &= ~(1U << 11);

}

void led_on()
{
	GPIOA->ODR |= (1U << 5);
}

void led_off()
{
	GPIOA->ODR &= ~(1U << 5);
}


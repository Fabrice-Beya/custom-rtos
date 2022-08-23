/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Fabrice Beya
 * @brief          : Main program body
 ******************************************************************************
 ******************************************************************************
*
**/

#include "led.h"

void delay(void)
{
	for(uint32_t i = 0 ; i < 50000 ; i ++);
}

int main(void)
{
	LED_Init();

	while(1)
	{
		LED_Toggle();
		delay();
		LED_Toggle();
		delay();
	}
}

/*
 * uart.h
 *
 *  Created on: Aug 19, 2022
 *      Author: fabricebeya
 */

#ifndef UART_H_
#define UART_H_

#include "stm32f3xx.h"
#include <stdio.h>

#define ENABLE 						(|=)
#define DISABLE						(&=~)

#define GPIOA_CLOCK_EN				(1U << 17)
#define UART2_CLOCK_EN				(1U << 17)

#define GPIOA2_AF_0					(1U << 8)
#define GPIOA2_AF_1					(GPIOA2_AF_0 << 1)
#define GPIOA2_AF_2					(GPIOA2_AF_0 << 2)
#define GPIOA2_AF_3					(GPIOA2_AF_0 << 3)
#define GPIOA3_AF_0					(1U << 12)
#define GPIOA3_AF_1					(GPIOA3_AF_0 << 1)
#define GPIOA3_AF_2					(GPIOA3_AF_0 << 2)
#define GPIOA3_AF_3					(GPIOA3_AF_0 << 3)

#define GPIOA2_MODE_0				(1U << 4)
#define GPIOA2_MODE_1				(GPIOA2_MODE_0 << 1)
#define GPIOA3_MODE_0				(1U << 6)
#define GPIOA3_MODE_1				(GPIOA3_MODE_0 << 1)

#define BAUD_RATE_115200			115200
#define BAUD_RATE_9600				9600
#define UART2_CLK_SPEED				8000000	// Default clock speed

#define UART2_TX_EN					(1U << 3)
#define UART2_RX_EN					(1U << 2)

#define UART2_STOP_BITS_0			(1U << 12)
#define UART2_STOP_BITS_1			(1U << 13)

#define UART2_CTS_CTRL				(1U << 9)
#define UART2_RTS_CTRL				(1U << 10)

#define UART2_TX_READY				(1U << 7)
#define UART2_RX_READY				(1U << 5)

#define UART2_8BIT_WORD_M0			(1U << 12)
#define UART2_8BIT_WORD_M1			(1U << 28)

#define UART2_EN					(1U)


void uart_init(void);
int uart_write(int ch);
int uart_read(void);
void uart_test(void);

#endif /* UART_H_ */

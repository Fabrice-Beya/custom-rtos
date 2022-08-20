/*
 * uart.c
 *
 *  Created on: Aug 19, 2022
 *      Author: fabricebeya
 */

#include "uart.h"

void enable_gpio_clock(void);
void enable_uart_clock(void);
void configure_gpio(void);
void confgiure_uart(void);
void configure_baudrate_115200();
void configure_baudrate_9600();
uint16_t compute_baud_rate(uint32_t peripheral_clock, uint32_t baudrate);

/*
 * -	Enable clock access to Port A
 * -	Enable clock access to UART 2 peripheral
 * -	Configure pins PA2 and PA3 to be used for UART2 alternative function
 * -	Configure UART
 * -
 */
void uart_init(void)
{
	enable_gpio_clock();
	enable_uart_clock();
	configure_gpio();
	confgiure_uart();

	setvbuf( stdin, NULL, _IONBF, 0 );
}

/*
 *	Enable clock access to Port A
 * 	Port A is connected to AHB2 bus
 * 	Set bit 17 of RCC_AHBENR to 1
 */
void enable_gpio_clock(void)
{
	RCC->AHBENR |= GPIOA_CLOCK_EN;
}


/*
 *	Enable clock access to UART 2
 * 	UART 2 is connected to APB1
 * 	Set bit 17 of RCC_APB1ENR to 1
 *
 */
void enable_uart_clock(void)
{
	RCC->APB1ENR |= UART2_CLOCK_EN;
}

/*
 * 	Configure GPIOA mode to AF for PA2 and PA3
 * 	Set GPIOA_MODER MODER2[1:0] and MODER3[1:0] to 10
 *	Configure pins PA2 and PA3 to be used as alternative function
 * 	Set GPIOA_AFRL AFR2[3:0] and AFR3[3:0] to 0111(AF7) to set PA2 and PA3 to be used as TX and RX respectively
 *
 */
void configure_gpio(void)
{
	GPIOA->MODER &= ~GPIOA2_MODE_0;
	GPIOA->MODER |= GPIOA2_MODE_1;
	GPIOA->MODER &= ~GPIOA3_MODE_0;
	GPIOA->MODER |= GPIOA3_MODE_1;

	GPIOA->AFR[0] |= GPIOA2_AF_0;
	GPIOA->AFR[0] |= GPIOA2_AF_1;
	GPIOA->AFR[0] |= GPIOA2_AF_2;
	GPIOA->AFR[0] &= ~GPIOA2_AF_3;

	GPIOA->AFR[0] |= GPIOA3_AF_0;
	GPIOA->AFR[0] |= GPIOA3_AF_1;
	GPIOA->AFR[0] |= GPIOA3_AF_2;
	GPIOA->AFR[0] &= ~GPIOA3_AF_3;
}

/*
 *	Configure UART 2
 *	Set baud rate i.e the transfer rate to 115200
 *	Enable TX and RX mode in the control registers
 *	Set the word length to 8 bits
 *	Set the data size to be 8bits
 *	Set number of stop bits to 1: STOP[1:0]
 *	Disable hardware flow control
 *
 */
void confgiure_uart(void)
{
	configure_baudrate_115200();

	USART2->CR1 |= UART2_TX_EN;
	USART2->CR1 |= UART2_RX_EN;

	USART2->CR1 &= ~UART2_8BIT_WORD_M0;
	USART2->CR1 &= ~UART2_8BIT_WORD_M1;

	USART2->CR2 &= ~UART2_STOP_BITS_0;
	USART2->CR2 &= ~UART2_STOP_BITS_0;

	USART2->CR3 &= ~UART2_CTS_CTRL;
	USART2->CR3 &= ~UART2_RTS_CTRL;

	USART2->CR1 |= UART2_EN;
}

/*
 *	Configure baud rate register for 115200
 *
 */
void configure_baudrate_115200()
{
	USART2->BRR = compute_baud_rate(UART2_CLK_SPEED, BAUD_RATE_115200);
}

/*
 *	Configure baud rate register for 9600
 *
 */
void configure_baudrate_9600()
{
	USART2->BRR = compute_baud_rate(UART2_CLK_SPEED, BAUD_RATE_9600);
}

/*
 *	Configure baud rate value to pass into the baud rate register
 *	Given the clock speed of the peripheral and the desired baud
 *	rate we compute the value to pass into the register.
 *
 */
uint16_t compute_baud_rate(uint32_t peripheral_clock, uint32_t baudrate)
{
	return ((peripheral_clock + (baudrate/2U))/baudrate);
}

int uart_write(int ch)
{

	while(!(USART2->ISR & UART2_TX_READY)){}

	// write to the TX data register
	USART2->TDR = (ch & 0xFF);

	return ch;
}

int uart_read()
{

	while(!(USART2->ISR & UART2_RX_READY)){}

	return  USART2->RDR;
}


int __io_putchar(int ch)
{
	return uart_write(ch);
}

int __io_getchar(void)
{
	int c;
	c = uart_read();

	if (c == '\r') {
		uart_write(c);
		c = '\n';
	}

	uart_write(c);

	return c;
}

int val;
char str[80];

void uart_test(void)
{
	printf("Please enter a valid number: ");
	scanf("%d", &val);
	printf("The number entered is: %d\r\n", val);
	printf("Please type a character string: ");
	scanf("%s", str);
	printf("The character string entered: ");
	printf(str);
	printf("\r\n");

}




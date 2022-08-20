/*
 * os_kernel.h
 *
 *  Created on: 20 Aug 2022
 *      Author: fabricebeya
 */

#ifndef OS_KERNEL_H_
#define OS_KERNEL_H_

#include "stm32f3xx.h"
#include <stdint.h>

#define NUM_OF_THREADS 			3
#define STACKSIZE 				400
#define BUS_FREQ				8000000

#define STR_EN			(1U)
#define STR_INT			(1U << 1)
#define STR_CLK_SRC		(1U << 2)
#define STR_FLAG		(1U << 16)

#define ONE_SEC_LOAD	8000000

#define MAX_DELAY       0xFFFFFFFF

#define SYSTICK_RST     0

#define INT_CTRL		(*(volatile uint32_t *)0xE000ED04)
#define PENDSTSET		(1U << 26)


uint8_t os_kernel_add_threads(void(*task0)(void), void(*task1)(void), void(*task2)(void));
void os_kernel_init(void);
void os_kernel_launch(uint32_t quanta);
void os_yeild_thread(void);


#endif /* OS_KERNEL_H_ */

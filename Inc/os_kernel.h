/*
 * @File: os_kernel.h
 * @Brief:
 * A basic real time operating system kernel for the cortex-m4
 *
 * @Author: Fabrice Beya
 * @Created: 20 Aug 2022
 * @Last updated:  21 Aug 2022
 *
 */

#ifndef OS_KERNEL_H_
#define OS_KERNEL_H_

#include "utils.h"

// Maximum number of supported threads
#define NUM_OF_THREADS										10

// Total OS stack size
#define STACKSIZE 											400

// Default processor bus frequency
// Default stm32f3 is 8Mhz
#define BUS_FREQ											8000000

// Program Status Register T-Bit
// Used to configure the arm processor to use THUMB
// instruction set.
#define PSR_TBIT											bit24

// Enable SysTick clock
#define SYSTICK_CTRL_EN										bit0

// Enable SysTick Interrupt
#define SYSTICK_CTRL_INT									bit1

// Set the SysTick clock to use the internal clock
#define SYSTICK_CTRL_CLK									bit2

// SysTick timer flag i.e counter = 0
#define SYSTICK_CTRL_FLAG									bit16

// Pre-scaler used to set SysTick counter to 1 second
#define ONE_SEC_LOAD										8000000

// Maximum delay size = maximum 32 bit value
#define MAX_DELAY       									0xFFFFFFFF

// Value set when resting SysTick counter value
#define SYSTICK_CTRL_RST    	 							0

// Memory address of SysTick interrupt
#define INT_CTRL											(*(volatile uint32_t *)0xE000ED04)

// Used to programmatically trigger a SysTick interrupt
#define PENDSTSET											bit26

// Thread Control Block type definition
// Linked list node used manage collection of os threads
struct thread_control_block {
	// Pointer to first address of thread stack
	int32_t *stackPts;
	// Pointer to the first address of the next thread
	struct tcb *nextPt;
}typedef task_control_block_type;


task_control_block_type os_stack[NUM_OF_THREADS];
task_control_block_type *p_current_thread;


uint8_t Os_Kernel_Add_Threads(void(*task0)(void), void(*task1)(void), void(*task2)(void));
void Os_Kernel_Init(void);
void Os_Kernel_Launch(uint32_t quanta);
void Os_Yeild_Thread(void);


#endif /* OS_KERNEL_H_ */

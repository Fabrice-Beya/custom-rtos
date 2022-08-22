/**
 ******************************************************************************
 * @file           : os_kernel.h
 * @author         : Fabrice Beya
 * @brief          : OS header file
 ******************************************************************************
 ******************************************************************************
*
**/

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

// Dummy value used when initialising thread registers
#define THREAD_REG_INT_VAL									0xAAAAAAAA

// Maximum thread name size
#define THREAD_NAME_SIZE									24

// Used to keep track of a threads state
enum thread_state {
	READY = 0,
	BLOCKED,
	RUNNING
}typedef thread_state_type;

// Thread Control Block type definition
// Linked list node used manage collection of os threads
struct thread_control_block {
	// Pointer to first address of thread stack
	int32_t *stackPts;
	// Pointer to the first address of the next thread
	struct tcb *nextPt;
	// Index of the current thread
	uint32_t index;
	// Thread state
	thread_state_type state;
	// Priority weighting of the thread. [0 - 10]
	int priority;
	// Index of the current thread
	char *name;

}typedef thread_handle_t;


thread_handle_t* Os_Kernel_Create_Thread(char *name, int priority, void(*thread)(void));
void Os_Kernel_Init(void);
void Os_Kernel_Launch(uint32_t quanta);
void Os_Yeild_Thread(void);


#endif /* OS_KERNEL_H_ */

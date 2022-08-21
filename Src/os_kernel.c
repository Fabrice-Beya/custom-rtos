/*
 * os_kernel.c
 *
 *  Created on: 20 Aug 2022
 *      Author: fabricebeya
 */


/*
  * @Brief:
  * Each thread will have a stack size of 100 bytes.
	We need 16 * 4(32 bits) = 64 bytes to hold the all the 16
	processor registers. Only 8 are stored in the stack when
	context switching. xPSR, PC, LR, r12, r3, r2, r1, r0

	Register order:
	r16: PSR
	r15: PC
	r14: LR
	r12:
	r3:
	r2:
	r1:
	r0:
	r11:
	r10:
	r9:
	r8:
	r7:
	r6:
	r5:
	r4:

*/

#include "os_kernel.h"

void os_schedular_launch(void);
void os_kernel_stack_init(int thread_index);

// Launch kernel setting time constant quanta
uint32_t KERNEL_MILLIS_PRESCALER;

// Total number of create threads
volatile uint32_t thread_count;

struct tcb {
	int32_t *stackPts;
	struct tcb *nextPt;
}typedef tcbType;

tcbType tcbs[NUM_OF_THREADS];
tcbType *currentPt;


int32_t TCB_STACK[NUM_OF_THREADS][STACKSIZE];

/**
  *@brief
  *
  *
  *@param a parameter a
  *@param b parameter b
  *@return  return sum
  *
  */
void Os_Kernel_Init(void)
{
	KERNEL_MILLIS_PRESCALER = (BUS_FREQ/1000);
	thread_count = 0;
}

/**
  *@brief
  * The thread stack is initialized from the bottom up, this is due to the processor exception frame.
  * The exception frame stores registers from PSR to R0:
  *	Stack Pointer: 16
  *
  *
  *
  * holds the thread context which is stored by the processor when performing a context switch. Thus we initialize the
  * thread stack from the bottom up i.e [STACKSIZE - {Register position}]
  *
  *@param void
  *@return NA
  *
  */
void init_thread_stack()
{
	/* Set the thread stack pointer to the location of where the Program Counter is stored.
	 * Program Counter is located at 16th word from the top of the stack
	 * The register is initialize, and the set the thread stack point to this location.
	 */
	TCB_STACK[thread_index][STACKSIZE - 16] 	= 0xAAAAAAAA;
	os_stack[thread_count].stackPts = &os_stack[thread_count][STACKSIZE - 16]; // Program counter

	// Stored during context switching aka stack frame
	TCB_STACK[thread_index][STACKSIZE - 3]  	= 0xAAAAAAAA; // r14: LR(Link register)
	TCB_STACK[thread_index][STACKSIZE - 4] 		= 0xAAAAAAAA; // r12:
	TCB_STACK[thread_index][STACKSIZE - 5] 		= 0xAAAAAAAA; // r3:
	TCB_STACK[thread_index][STACKSIZE - 6] 		= 0xAAAAAAAA; // r2:
	TCB_STACK[thread_index][STACKSIZE - 7] 		= 0xAAAAAAAA; // r1:
	TCB_STACK[thread_index][STACKSIZE - 8] 		= 0xAAAAAAAA; // r0:

	/* Store auxiliary register
	 * @Brief:
	 * 	These registers are not pushed by the processor when an exception is call
	 * 	we manually store them in the stack frame, so we have the full register
	 * 	context in the thread stack.
	 */
	TCB_STACK[thread_index][STACKSIZE - 15] 	= 0xAAAAAAAA; // r5:
	TCB_STACK[thread_index][STACKSIZE - 9] 		= 0xAAAAAAAA; // r11:
	TCB_STACK[thread_index][STACKSIZE - 10] 	= 0xAAAAAAAA; // r10:
	TCB_STACK[thread_index][STACKSIZE - 11] 	= 0xAAAAAAAA; // r9:
	TCB_STACK[thread_index][STACKSIZE - 12] 	= 0xAAAAAAAA; // r8:
	TCB_STACK[thread_index][STACKSIZE - 13] 	= 0xAAAAAAAA; // r7:
	TCB_STACK[thread_index][STACKSIZE - 14]	 	= 0xAAAAAAAA; // r6:




	// The program counter needs to hold the address of the thread function



	/**
	 * Set the T-Bit of the Program Status Register to 1.
	 * @Brief:
	 * 	The register in the stack holds the Program Status Register.
	 * 	The T-Bit(bit 24) configures the processor to use the thumb instructions set
	 */
	os_stack[thread_count][STACKSIZE - 1] = PSR_TBIT;


}

/**
  *@brief
  *
  *
  *@param a parameter a
  *@param b parameter b
  *@return  return sum
  *
  */
uint8_t Os_Kernel_Add_Thread(void(*task)(void))
{
	// Disable global interrupts
	__disable_irq();

	// Increase thread count
	thread_count++;

	// Allocate os stack space for the new task
	init_thread_stack();

	// Add new
	tcbs[0].nextPt = &tcbs[1];
	tcbs[1].nextPt = &tcbs[2];
	tcbs[2].nextPt = &tcbs[0];

	// Initialize stack for each task
	// Set the PC to point the the address of each of the associated tasks
	os_kernel_stack_init(0);
	TCB_STACK[0][STACKSIZE - 2]  = (int32_t)(task0);

	os_kernel_stack_init(1);
	TCB_STACK[1][STACKSIZE - 2]  = (int32_t)(task1);

	os_kernel_stack_init(2);
	TCB_STACK[2][STACKSIZE - 2]  = (int32_t)(task2);

	// Set current stack to task0
	currentPt = &tcbs[0];

	// Enable global interrupts
	__enable_irq();

	return 1;
}

/**
  *@brief
  *
  *
  *@param a parameter a
  *@param b parameter b
  *@return  return sum
  *
  */
uint8_t Os_Kernel_Add_Threads(void(*task0)(void), void(*task1)(void), void(*task2)(void))
{
	// Disable global interrupts
	__disable_irq();

	// Connect
	tcbs[0].nextPt = &tcbs[1];
	tcbs[1].nextPt = &tcbs[2];
	tcbs[2].nextPt = &tcbs[0];

	// Initialize stack for each task
	// Set the PC to point the the address of each of the associated tasks
	os_kernel_stack_init(0);
	TCB_STACK[0][STACKSIZE - 2]  = (int32_t)(task0);

	os_kernel_stack_init(1);
	TCB_STACK[1][STACKSIZE - 2]  = (int32_t)(task1);

	os_kernel_stack_init(2);
	TCB_STACK[2][STACKSIZE - 2]  = (int32_t)(task2);

	// Set current stack to task0
	currentPt = &tcbs[0];

	// Enable global interrupts
	__enable_irq();

	return 1;
}


/**
  *@brief
  *
  *
  *@param quanta: The desired maximum thread period in milliseconds
  *@param b parameter b
  *@return  return sum
  *
  */
void Os_Kernel_Launch(uint32_t quanta)
{
	// reset systick value
	SysTick->CTRL = SYSTICK_CTRL_RST;
	// clear systick by writing any value
	SysTick->VAL = 0;
	// Load quanta size
	SysTick->LOAD = (quanta * KERNEL_MILLIS_PRESCALER) - 1;
	// Set systick priority to low so that other interrupts will be
	// So that hardware interrupts wont be blocked by the systick interrupts
	NVIC_SetPriority(SysTick_IRQn, 15);
	// Select sytick clock to internal clock | Enable systick Interrupts || Enable systick
	SysTick->CTRL |= SYSTICK_CTRL_CLK | SYSTICK_CTRL_INT | SYSTICK_CTRL_EN;

	// Launch scheduler
	// os scheduler launch
	os_schedular_launch();
}

/**
  *@brief
  *
  *
  *@param a parameter a
  *@param b parameter b
  *@return  return sum
  *
  */
void os_kernel_stack_init(int thread_index)
{

	tcbs[thread_index].stackPts = &TCB_STACK[thread_index][STACKSIZE - 16]; // Program counter

	// The first 4 bytes(32bits) in the stack hold the xPSR register
	// bit 21 indicates which instruction sets to use either thumb or arms
	// The cortex m4 uses thumb instruction set thus this bit needs to be set high
	TCB_STACK[thread_index][STACKSIZE - 1] = (1U << 24); // PSR(program status) register bit 21(T-Bit) set high

	// The program counter needs to hold the address of the thread function

	// Stored during context switching aka stack frame
	TCB_STACK[thread_index][STACKSIZE - 3]  	= 0xAAAAAAAA; // r14: LR(Link register)
	TCB_STACK[thread_index][STACKSIZE - 4] 		= 0xAAAAAAAA; // r12:
	TCB_STACK[thread_index][STACKSIZE - 5] 		= 0xAAAAAAAA; // r3:
	TCB_STACK[thread_index][STACKSIZE - 6] 		= 0xAAAAAAAA; // r2:
	TCB_STACK[thread_index][STACKSIZE - 7] 		= 0xAAAAAAAA; // r1:
	TCB_STACK[thread_index][STACKSIZE - 8] 		= 0xAAAAAAAA; // r0:

	// Not stored when context switching
	TCB_STACK[thread_index][STACKSIZE - 9] 		= 0xAAAAAAAA; // r11:
	TCB_STACK[thread_index][STACKSIZE - 10] 	= 0xAAAAAAAA; // r10:
	TCB_STACK[thread_index][STACKSIZE - 11] 	= 0xAAAAAAAA; // r9:
	TCB_STACK[thread_index][STACKSIZE - 12] 	= 0xAAAAAAAA; // r8:
	TCB_STACK[thread_index][STACKSIZE - 13] 	= 0xAAAAAAAA; // r7:
	TCB_STACK[thread_index][STACKSIZE - 14]	 	= 0xAAAAAAAA; // r6:
	TCB_STACK[thread_index][STACKSIZE - 15] 	= 0xAAAAAAAA; // r5:
	TCB_STACK[thread_index][STACKSIZE - 16] 	= 0xAAAAAAAA; // r4:
}

/**
  *@brief
  *
  *
  *@param a parameter a
  *@param b parameter b
  *@return  return sum
  *
  */
void os_kernel_stack_init(int thread_index)
{

	tcbs[thread_index].stackPts = &TCB_STACK[thread_index][STACKSIZE - 16]; // Program counter

	// The first 4 bytes(32bits) in the stack hold the xPSR register
	// bit 21 indicates which instruction sets to use either thumb or arms
	// The cortex m4 uses thumb instruction set thus this bit needs to be set high
	TCB_STACK[thread_index][STACKSIZE - 1] = (1U << 24); // PSR(program status) register bit 21(T-Bit) set high

	// The program counter needs to hold the address of the thread function

	// Stored during context switching aka stack frame
	TCB_STACK[thread_index][STACKSIZE - 3]  	= 0xAAAAAAAA; // r14: LR(Link register)
	TCB_STACK[thread_index][STACKSIZE - 4] 		= 0xAAAAAAAA; // r12:
	TCB_STACK[thread_index][STACKSIZE - 5] 		= 0xAAAAAAAA; // r3:
	TCB_STACK[thread_index][STACKSIZE - 6] 		= 0xAAAAAAAA; // r2:
	TCB_STACK[thread_index][STACKSIZE - 7] 		= 0xAAAAAAAA; // r1:
	TCB_STACK[thread_index][STACKSIZE - 8] 		= 0xAAAAAAAA; // r0:

	// Not stored when context switching
	TCB_STACK[thread_index][STACKSIZE - 9] 		= 0xAAAAAAAA; // r11:
	TCB_STACK[thread_index][STACKSIZE - 10] 	= 0xAAAAAAAA; // r10:
	TCB_STACK[thread_index][STACKSIZE - 11] 	= 0xAAAAAAAA; // r9:
	TCB_STACK[thread_index][STACKSIZE - 12] 	= 0xAAAAAAAA; // r8:
	TCB_STACK[thread_index][STACKSIZE - 13] 	= 0xAAAAAAAA; // r7:
	TCB_STACK[thread_index][STACKSIZE - 14]	 	= 0xAAAAAAAA; // r6:
	TCB_STACK[thread_index][STACKSIZE - 15] 	= 0xAAAAAAAA; // r5:
	TCB_STACK[thread_index][STACKSIZE - 16] 	= 0xAAAAAAAA; // r4:
}

// __atrribute__((naked)) prevents stack from being changed during the execution of the systick handler
// GNU compiler documentation using GNU assembler asm syntax different on arm keil.
// Save current thread and load the next thread.

/**
  *@brief
  *
  *
  *@param a parameter a
  *@param b parameter b
  *@return  return sum
  *
  */
__attribute__((naked))  void SysTick_Handler(void)
{
	// Run each quata

	// suspend current thread
	// disable global interrupts
	__asm("CPSID	I");
	// explicitly save registers not saved by processor
	// r4 - r11
	__asm("PUSH	{R4-R11}");
	// Load address of current pointer(thread) into r0
	__asm("LDR R0, =currentPt");
	// load current point into r1
	__asm("LDR R1,[R0]");
	// store cortex m4 stack pointer at address r1 i.e store stack pointer into tcb stackPt
	__asm("STR SP,[R1]");

	// choose next thread
	// load r1 from a location +4 bytes above address r1 i.e r1 = currentPt->next
	__asm("LDR R1,[R1,#4]");
	// Load currentPt->next into r1 store r1 into r0
	__asm("STR R1,[R0]");
	// Load stack pointer with current->next stack pointer into cortex-m4 stack pointer
	__asm("LDR SP,[R1]");
	// restore push register r4-r11
	__asm("POP {R4-R11}");

	// enable global interrupts
	__asm("CPSIE	I");

	// return from sub-routine/exception and restore saved registers
	// regsiters are automatically restored when we return.
	__asm("BX   LR");

}

/**
  *@brief
  *
  *
  *@param a parameter a
  *@param b parameter b
  *@return  return sum
  *
  */
void os_schedular_launch(void)
{
	// load address of currentPt into R0
	__asm("LDR R0,=currentPt");
	// Load r2 from address r2 with r0 ie r2 = currentPt
	__asm("LDR R2,[R0]");
	// Load r2 into cortex-m4 stack pointer ie making currentPt = cortex-m4 stack pointer
	__asm("LDR SP,[R2]");
	// restore push register r4-r11
	__asm("POP {R4-R11}");
	// retor the link register r12
	__asm("POP {R12}");
	// restore r0,r1,r2,r3
	__asm("POP {R0-R3}");
	// skip
	__asm("ADD SP,SP,#4");
	// create a new start location by popping lr
	__asm("POP {LR}");
	// skip PSR by adding 4
	__asm("ADD SP,SP,#4");

	// enable global interrupts
	__asm("CPSIE	I");
	// return from exception.
	__asm("BX    LR");
}


/**
  *@brief
  *
  *
  *@param a parameter a
  *@param b parameter b
  *@return  return sum
  *
  */
void Os_Yeild_Thread(void)
{
	// clear systick value
	SysTick->VAL = 0;
	// trigger a systick interrupt
	INT_CTRL |= PENDSTSET;
}



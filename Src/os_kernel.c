/*
 * os_kernel.c
 *
 *  Created on: 20 Aug 2022
 *      Author: fabricebeya
 */
#include "os_kernel.h"

void os_schedular_launch(void);
void os_kernel_stack_init(int thread_index);
// launch kernel setting time constant quanta
uint32_t KERNEL_MILLIS_PRESCALER;
struct tcb {
	int32_t *stackPts;
	struct tcb *nextPt;
}typedef tcbType;

tcbType tcbs[NUM_OF_THREADS];
tcbType *currentPt;

 /*
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
int32_t TCB_STACK[NUM_OF_THREADS][STACKSIZE];


/*
 * We skip the first 4 bytes this points to the stack pointer r13
 *
 * */

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

uint8_t os_kernel_add_threads(void(*task0)(void), void(*task1)(void), void(*task2)(void))
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

void os_kernel_init(void)
{
	KERNEL_MILLIS_PRESCALER = (BUS_FREQ/1000);
}

void os_kernel_launch(uint32_t quanta)
{
	// reset systick value
	SysTick->CTRL = SYSTICK_RST;
	// clear systick by writing any value
	SysTick->VAL = 0;
	// Load quanta size
	SysTick->LOAD = (quanta * KERNEL_MILLIS_PRESCALER) - 1;
	// Set systick priority to low so that other interrupts will be
	// So that hardware interrupts wont be blocked by the systick interrupts
	NVIC_SetPriority(SysTick_IRQn, 15);
	// Select sytick clock to internal clock | Enable systick Interrupts || Enable systick
	SysTick->CTRL |= STR_CLK_SRC | STR_INT | STR_EN;

	// Launch scheduler
	// os scheduler launch
	os_schedular_launch();
}

// __atrribute__((naked)) prevents stack from being changed during the execution of the systick handler
// GNU compiler documentation using GNU assembler asm syntax different on arm keil.
// Save current thread and load the next thread.
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

void os_yeild_thread(void)
{
	// clear systick value
	SysTick->VAL = 0;
	// trigger a systick interrupt
	INT_CTRL |= PENDSTSET;
}



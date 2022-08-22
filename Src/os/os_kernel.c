/**
 ******************************************************************************
 * @file           : os_kernel.c
 * @author         : Fabrice Beya
 * @brief          : Operating System kernel
 ******************************************************************************
 ******************************************************************************
*
**/

#include "os_kernel.h"

void os_schedular_launch(void);
void init_thread_stack(int32_t thread_address);

// Total number of created threads
volatile uint32_t os_thread_count = 0;

// Holds the computed pre-scalar value used to convert the specified os Quanta in milliseconds
volatile uint32_t KERNEL_MILLIS_PRESCALER;

// Total os stack space
int32_t os_stack[NUM_OF_THREADS][STACKSIZE];

// Total list of all create threads
thread_handle_t threads[NUM_OF_THREADS];

// Pointer to the current thread being executed
thread_handle_t *p_current_thread;

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
	os_thread_count = 0;
}

/**
  *@brief
  * The thread stack is initialized from the bottom up, this is due to the processor exception frame.
  * The exception frame stores registers from PSR to R0:
  *	: 16
  *
  * holds the thread context which is stored by the processor when performing a context switch. Thus we initialize the
  * thread stack from the bottom up i.e [STACKSIZE - {Register position}]
  *
  *@param void
  *@return NA
  *
  */
void init_thread_stack(int32_t thread_address)
{
	/**
	 * xPRS Register
	 * Set the T-Bit of the Program Status Register to 1.
	 * @Brief:
	 * 	The register in the stack holds the Program Status Register.
	 * 	The T-Bit(bit 24) configures the processor to use the thumb instructions set
	 */
	os_stack[os_thread_count][STACKSIZE - 1] = PSR_TBIT;

	/**
	 * PC Register
	 * Initialise the program counter register
	 * @Brief:
	 * 	The program counter register must be initialise to point the address of the
	 * 	thread function. This is done in the
	 */
	os_stack[os_thread_count][STACKSIZE - 2] = thread_address;

	/**
	 * Link Register
	 * Initialise the link register to dummy value THREAD_REG_INT_VAL
	 * @Brief:
	 * 	Register is stored as part of exception stack frame.
	 */
	os_stack[os_thread_count][STACKSIZE - 3] = THREAD_REG_INT_VAL;

	/**
	 * R12
	 * Initialise the R12 to dummy value THREAD_REG_INT_VAL
	 * @Brief:
	 * 	Register is stored as part of exception stack frame.
	 */
	os_stack[os_thread_count][STACKSIZE - 4] = THREAD_REG_INT_VAL;

	/**
	 * R3
	 * Initialise the R3 to dummy value THREAD_REG_INT_VAL
	 * @Brief:
	 * 	Register is stored as part of exception stack frame.
	 */
	os_stack[os_thread_count][STACKSIZE - 5] = THREAD_REG_INT_VAL;

	/**
	 * R2
	 * Initialise the R2 to dummy value THREAD_REG_INT_VAL
	 * @Brief:
	 * 	Register is stored as part of exception stack frame.
	 */
	os_stack[os_thread_count][STACKSIZE - 6] = THREAD_REG_INT_VAL;

	/**
	 * R1
	 * Initialise the R1 to dummy value THREAD_REG_INT_VAL
	 * @Brief:
	 * 	Register is stored as part of exception stack frame.
	 */
	os_stack[os_thread_count][STACKSIZE - 7] = THREAD_REG_INT_VAL;

	/**
	 * R0
	 * Initialise the R0 to dummy value THREAD_REG_INT_VAL
	 * @Brief:
	 * 	Register is stored as part of exception stack frame.
	 */
	os_stack[os_thread_count][STACKSIZE - 8] = THREAD_REG_INT_VAL;

	/**
	 * R11 -> R4
	 * Initialise R11->R4 to dummy value THREAD_REG_INT_VAL
	 * @Brief:
	 * 	These register values are not automatically stored, but we keep a record of them
	 * 	in our thread stack frame.
	 */
	os_stack[os_thread_count][STACKSIZE - 9]  = THREAD_REG_INT_VAL; // R11
	os_stack[os_thread_count][STACKSIZE - 10] = THREAD_REG_INT_VAL; // R10
	os_stack[os_thread_count][STACKSIZE - 11] = THREAD_REG_INT_VAL; // R9
	os_stack[os_thread_count][STACKSIZE - 12] = THREAD_REG_INT_VAL; // R8
	os_stack[os_thread_count][STACKSIZE - 13] = THREAD_REG_INT_VAL; // R7
	os_stack[os_thread_count][STACKSIZE - 14] = THREAD_REG_INT_VAL; // R6
	os_stack[os_thread_count][STACKSIZE - 15] = THREAD_REG_INT_VAL; // R5
	os_stack[os_thread_count][STACKSIZE - 16] = THREAD_REG_INT_VAL; // R4

	/**
	 * @Brief:
	 * 	Set the new threads stack pointer to the address at the top of the thread stack
	 */
	threads[os_thread_count].stackPts = &os_stack[os_thread_count][STACKSIZE - 16];


	/**
	 * Note:
	 * 	Register R13 is the Stack Pointer. This holds the address of the top
	 * 	of the stack frame i.e when an exception occurs r13 = os_stack[thread_count][STACKSIZE - 1]
	 */

}

/**
  *@brief
  *
  *
  *@param thread: A new thread function.
  *@return: A pointer to stack frame of the new thread.
  *
  */
thread_handle_t* Os_Kernel_Create_Thread(char *name, int priority, void(*thread)(void))
{
	// Disable global interrupts
	__disable_irq();

	// Initialise os stack space for the new task
	init_thread_stack((int32_t)thread);

	//Initialise thread meta data
	threads[os_thread_count].name = name;
	threads[os_thread_count].index = os_thread_count;
	threads[os_thread_count].state = READY;
	threads[os_thread_count].priority = priority;

	// Increase thread counter.
	os_thread_count++;

	// Enable global interrupts
	__enable_irq();

	return &threads[os_thread_count];
}

/**
  *@Brief
  *	Implement scheduler priority
  *
  */
void round_robin(void)
{
	if (os_thread_count == 0)
	{
		return;
	}

	for(int i = 0; i < os_thread_count; i++)
	{
		if (i == (os_thread_count - 1)){
			threads[i].nextPt = &threads[0];
		} else {
			threads[i].nextPt = &threads[i + 1];
		}
	}

	// Set current stack to the first thread
	p_current_thread = &threads[0];
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


// __atrribute__((naked)) prevents stack from being changed during the execution of the systick handler
// GNU compiler documentation using GNU assembler asm syntax different on arm keil.
// Save current thread and load the next thread.

/**
 * SysTick Exception Handler
  *@brief
  *	Called each time a SysTick interrupt has occurred. This is where we perform a context switch
  *	i.e we stop the execution of the current thread and replace it with the next thread in our
  *	scheduler priority queue.
  *
  *	__atrribute__((naked)) prevents stack from being changed during the execution of the systick handler
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
	__asm("LDR R0, =p_current_thread");
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
	// Registers are automatically restored when we return.
	__asm("BX   LR");

}

/**
 * Launch the os with the current thread
  *@brief
  *	This is the first task is loaded upon os launch.
  *
  */
void os_schedular_launch(void)
{
	// Apply round robin schedule
	round_robin();
	// load address of currentPt into R0
	__asm("LDR R0,=p_current_thread");
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
 * Yield control back to the os to run the next thread
  *@brief
  *	This allows a thread to end its execution and allow the next thread in the queue to run.
  *	This is achieved by triggering a SysTick interrupt which will preempt the current thread
  *	and move onto to running the next thread in the schedulers priority queue.
  *
  */
void Os_Yeild_Thread(void)
{
	// clear systick value
	SysTick->VAL = 0;
	// trigger a systick interrupt
	INT_CTRL |= PENDSTSET;
}



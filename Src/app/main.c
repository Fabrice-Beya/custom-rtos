/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Fabrice Beya
 * @brief          : Main program body
 ******************************************************************************
 ******************************************************************************
*
**/

#include "uart.h"
#include "os_kernel.h"

typedef uint32_t TaskProfiler;

TaskProfiler Task0_Profiler, Task1_Profiler, Task2_Profiler;
thread_handle_t* task0_handle;
thread_handle_t* task1_handle;
thread_handle_t* task2_handle;

#define QUANTA 		10

void motor_on(void);
void motor_off(void);
void valve_open(void);
void valve_closed(void);

void task0(void)
{
	while(1)
	{
		Task0_Profiler++;
	}
}

void task1(void)
{
	while(1)
	{
		Task1_Profiler++;
	}
}

void task2(void)
{
	while(1)
	{
		Task2_Profiler++;
	}
}


int main(void)
{
	// Initialise uart
	uart_init();

	// Initialise kernel
	Os_Kernel_Init();

	// Create three tasks
	task0_handle = Os_Kernel_Create_Thread("Task 0", 0, &task0);
	task1_handle = Os_Kernel_Create_Thread("Task 1", 0, &task1);
	task2_handle = Os_Kernel_Create_Thread("Task 2", 0, &task2);

	// Set time quanta to 10ms
	Os_Kernel_Launch(QUANTA);

}

void motor_on(void)
{
	printf("Motor is running... \n\r");
}

void motor_off(void)
{
	printf("Motor is stopping... \n\r");
}

void valve_open(void)
{
	printf("Valve is opening... \n\r");
}

void valve_closed(void)
{
	printf("Valve is closing... \n\r");
}

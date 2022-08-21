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
	// init uart
	uart_init();
	// Init kernel
	Os_Kernel_Init();
	// Add threads
	Os_Kernel_Add_Thread(&task0);
	Os_Kernel_Add_Thread(&task1);
	Os_Kernel_Add_Thread(&task2);
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

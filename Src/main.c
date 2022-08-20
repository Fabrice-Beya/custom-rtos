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
		os_yeild_thread();
	}
}

void task1(void)
{
	while(1)
	{
		Task1_Profiler++;
		valve_open();
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
	os_kernel_init();
	// Add threads
	os_kernel_add_threads(&task0, &task1, &task2);
	// Set time quanta to 10ms
	os_kernel_launch(QUANTA);

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

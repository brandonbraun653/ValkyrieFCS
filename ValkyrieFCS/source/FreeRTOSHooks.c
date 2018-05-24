#include "FreeRTOS.h"
#include "task.h"

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
	for (;;){}
}

void vApplicationMallocFailedHook( void )
{
	for(;;){}
}
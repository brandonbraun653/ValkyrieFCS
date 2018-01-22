/* C/C++ Includes */
#include <stdint.h>
#include <stdlib.h>
#include <string>

/* Thor Includes */
#include "include/thor.h"
#include "include/uart.h"
#include "include/print.h"

/* FreeRTOS Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Project Includes */
#include "fcsConfig.hpp"
#include "bluetooth.hpp"



void bluetoothTask(void* argument)
{
	/* Initialize serial */
	UARTClass_sPtr uart = uart5;
 	uart->begin(115200);
 	uart->setTxModeDMA();
 	uart->setRxModeDMA();
	
	const char* msg = "I'm alive!!!";
	
	TickType_t lastTimeWoken = xTaskGetTickCount();
	for (;;)
	{
		/* Run asynchronously and only push data at the specified update frequency */
		
		//uart->write(msg);
		
		vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(50));
	}
}
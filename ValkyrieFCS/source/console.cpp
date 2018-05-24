/* C/C++ Includes */
#include <stdint.h>
#include <stdlib.h>
#include <string>

/* Thor Includes */
#include <Thor/include/thor.hpp>
#include <Thor/include/uart.hpp>
#include <Thor/include/print.hpp>

/* FreeRTOS Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Project Includes */
#include <ValkyrieFCS/include/fcsConfig.hpp>
#include <ValkyrieFCS/include/console.hpp>
#include <ValkyrieFCS/include/dataTypes.hpp>
#include <ValkyrieFCS/include/radio.hpp>

const int updateRate_mS = (1.0 / CONSOLE_UPDATE_FREQ_HZ) * 1000.0;

void consoleTask(void* argument)
{
	/* Initialize serial */
	//UARTClass_sPtr uart = uart5;
 	//uart->begin(115200);
 	//uart->setTxModeDMA();
 	//uart->setRxModeDMA();
	
	//The console is connected to USART1 through the CP2102 bridge...need USART Lib...*sigh...
	
	
	TickType_t lastTimeWoken = xTaskGetTickCount();
	for (;;)
	{
		/* Run asynchronously and only push data at the specified update frequency */
		
		vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(updateRate_mS));
	}
}

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
#include "console.hpp"
#include "dataTypes.hpp"
#include "radio.hpp"


void consoleTask(void* argument)
{
	/* Initialize serial */
	UARTClass_sPtr uart = uart4;
 	uart->begin(115200);
 	uart->setTxModeDMA();
 	uart->setRxModeDMA();
	
	Radio_Control controlData;
	
	std::string throttle, roll, pitch, yaw, msg;
	std::string t("Throttle: "), r("Roll: "), p("Pitch: "), y("Yaw: ");
	
	TickType_t lastTimeWoken = xTaskGetTickCount();
	for (;;)
	{
		//Will need to handling outgoing log messages too...or heck, stream data to Matlab
		
		
		//Block until we get some new data 
		xQueueReceive(qRadio_Control, &controlData, portMAX_DELAY);
		
		throttle = float2String(controlData.THROTTLE);
		roll = float2String(controlData.ROLL);
		pitch = float2String(controlData.PITCH);
		yaw = float2String(controlData.YAW);
		
		msg = t + throttle + ' ' + r + roll + ' ' + p + pitch + ' ' + y + yaw + '\n';
		
		uart->write(msg);
	}
}
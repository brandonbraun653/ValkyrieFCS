
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
#include "console.hpp"
#include "dataTypes.hpp"
#include "radio.hpp"

const int updateRate_mS = (1.0 / CONSOLE_UPDATE_FREQ_HZ) * 1000.0;

void consoleTask(void* argument)
{
	/* Initialize serial */
	UARTClass_sPtr uart = uart4;
 	uart->begin(115200);
 	uart->setTxModeDMA();
 	uart->setRxModeDMA();
	
	
	TickType_t lastTimeWoken = xTaskGetTickCount();
	for (;;)
	{
		/* Run asynchronously and only push data at the specified update frequency */
		
		vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(updateRate_mS));
	}
}

//Block until we get some new data 
//		Radio_Control controlData;
//		std::string throttle, roll, pitch, yaw, msg;
//		std::string t("Throttle: "), r("Roll: "), p("Pitch: "), y("Yaw: ");
// 		xQueueReceive(qRadio_Control, &controlData, portMAX_DELAY);
// 		
// 		throttle = float2String(controlData.THROTTLE);
// 		roll = float2String(controlData.ROLL);
// 		pitch = float2String(controlData.PITCH);
// 		yaw = float2String(controlData.YAW);
// 		
// 		msg = t + throttle + ' ' + r + roll + ' ' + p + pitch + ' ' + y + yaw + '\n';
// 		
// 		uart->write(msg);
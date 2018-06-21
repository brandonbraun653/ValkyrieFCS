/* C/C++ Includes */
#include <stdint.h>
#include <stdlib.h>

/* Chimera Includes */
#include <Chimera/logging.hpp>
#include <Chimera/gpio.hpp>

/* FreeRTOS Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Project Includes */
#include <ValkyrieFCS/include/dataTypes.hpp>
#include <ValkyrieFCS/include/threads.hpp>
#include <ValkyrieFCS/include/led.hpp>

using namespace Chimera::GPIO;
using namespace Chimera::Logging;

namespace FCS_LED
{
	void parseTaskNotification(uint32_t notification)
	{
	}

	void ledStatus(void* argument)
	{
		Console.log(Level::INFO, "LED Thread: Initializing\r\n");

		GPIOClass armPin(PORTC, 8);
		GPIOClass modePin(PORTC, 7);
		GPIOClass errorPin(PORTC, 6);
		
		armPin.mode(OUTPUT_PUSH_PULL);
		armPin.write(LOW);

		modePin.mode(OUTPUT_PUSH_PULL);
		modePin.write(LOW);

		errorPin.mode(OUTPUT_PUSH_PULL);
		errorPin.write(LOW);
		
		
		Console.log(Level::INFO, "LED Thread: Initialization Complete\r\n");
		Chimera::Threading::signalThreadSetupComplete();
		Console.log(Level::INFO, "LED Thread: Running\r\n");

		TickType_t lastTimeWoken = xTaskGetTickCount();

		#ifdef DEBUG
		volatile UBaseType_t stackHighWaterMark_LEDSTATUS = uxTaskGetStackHighWaterMark(NULL);
		Console.log(Level::DBG, "Led Thread: Remaining stack size after init is %d bytes\r\n", stackHighWaterMark_LEDSTATUS);
		#endif

		for (;;)
		{
			armPin.write(HIGH);
			vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(150));
			armPin.write(LOW);
			vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(1000));
		}
	}
}

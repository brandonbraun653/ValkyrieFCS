/* C/C++ Includes */
#include <stdint.h>
#include <stdlib.h>

/* Chimera Includes */
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

namespace FCS_LED
{
	void parseTaskNotification(uint32_t notification)
	{
	}

	void ledStatus(void* argument)
	{
		#ifdef DEBUG
		volatile UBaseType_t stackHighWaterMark_LEDSTATUS = 0;
		#endif

		GPIOClass armPin(PORTC, 8);
		GPIOClass modePin(PORTC, 7);
		GPIOClass errorPin(PORTC, 6);
		

		/* Set up the GPIO Led */
		armPin.mode(OUTPUT_PUSH_PULL);
		armPin.write(LOW);

		modePin.mode(OUTPUT_PUSH_PULL);
		modePin.write(LOW);

		errorPin.mode(OUTPUT_PUSH_PULL);
		errorPin.write(LOW);
		
		
		Chimera::Threading::signalThreadSetupComplete();
		
		TickType_t lastTimeWoken = xTaskGetTickCount();
		for (;;)
		{
			#ifdef DEBUG
			activeTask = LED_STATUS_TASK;
			stackHighWaterMark_LEDSTATUS = uxTaskGetStackHighWaterMark(NULL);
			#endif
			
			armPin.write(HIGH);
			vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(150));
			armPin.write(LOW);
			vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(1000));
		}
	}
}

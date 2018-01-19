

/* C/C++ Includes */
#include <stdint.h>
#include <stdlib.h>

/* Thor Includes */
#include "include/thor.h"


/* FreeRTOS Includes */
#include "FreeRTOS.h"
#include "task.h"

/* Project Includes */
#include "fcsConfig.hpp"
#include "motors.hpp"

const int updateRate_mS = (1.0 / MOTOR_UPDATE_FREQ_HZ) * 1000.0;

void motorTask(void* argument)
{
	
	TickType_t lastTimeWoken = xTaskGetTickCount();
	
	for (;;)
	{
		
		
		vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(updateRate_mS));
	}
}


/* C/C++ Includes */
#include <stdint.h>
#include <stdlib.h>

/* Thor Includes */
#include "include/thor.h"
#include "include/gpio.h"
#include "include/exti.h"
#include "include/interrupt.h"

/* Boost Includes */
#include <boost/smart_ptr.hpp>
#include <boost/make_shared.hpp>

/* FreeRTOS Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Project Includes */
#include "dataTypes.hpp"
#include "threading.hpp"
#include "led.hpp"

using namespace ThorDef::GPIO;
GPIOClass_sPtr armPin;
GPIOClass_sPtr modePin;
GPIOClass_sPtr errorPin;

/*----------------------------------
* Private Function Forward Declarations 
*----------------------------------*/
void parseTaskNotification(uint32_t notification);

void ledStatus(void* argument)
{
	armPin = boost::make_shared<GPIOClass>(GPIOC, PIN_8, ULTRA_SPD, NOALTERNATE);
	modePin = boost::make_shared<GPIOClass>(GPIOC, PIN_7, ULTRA_SPD, NOALTERNATE); 
	errorPin = boost::make_shared<GPIOClass>(GPIOC, PIN_6, ULTRA_SPD, NOALTERNATE); 
	
	/* Set up the GPIO Led */
	armPin->mode(OUTPUT_PP);		armPin->write(LOW);
	modePin->mode(OUTPUT_PP);		modePin->write(LOW);
	errorPin->mode(OUTPUT_PP);		errorPin->write(LOW);
	
	
	
	TickType_t lastTimeWoken = xTaskGetTickCount();
	for (;;)
	{
		/* Check for some new notifications before continuing */
		parseTaskNotification(ulTaskNotifyTake(pdTRUE, 0));
		
		
		armPin->write(HIGH);
		vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(150));
		armPin->write(LOW);
		vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(1000));
	}
}


/*----------------------------------
* Private Functions 
*----------------------------------*/
void parseTaskNotification(uint32_t notification)
{
	if (notification == (LED_RED | LED_STATIC_ON))
		errorPin->write(HIGH);
}
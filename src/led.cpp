#include "led.hpp"

using namespace ThorDef::GPIO;

void ledStatus(void* argument)
{
	
	GPIOClass_sPtr armPin = boost::make_shared<GPIOClass>(GPIOC, PIN_8, ULTRA_SPD, NOALTERNATE);
	GPIOClass_sPtr modePin = boost::make_shared<GPIOClass>(GPIOC, PIN_7, ULTRA_SPD, NOALTERNATE); 
	GPIOClass_sPtr errorPin = boost::make_shared<GPIOClass>(GPIOC, PIN_6, ULTRA_SPD, NOALTERNATE); 
	
	/* Set up the GPIO Led */
	armPin->mode(OUTPUT_PP);
	modePin->mode(OUTPUT_PP);
	errorPin->mode(OUTPUT_PP);
	
	armPin->write(LOW);
	modePin->write(HIGH);
	errorPin->write(HIGH);
	
	TickType_t lastTimeWoken = xTaskGetTickCount();
	for (;;)
	{
		armPin->write(HIGH);
		vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(150));
		armPin->write(LOW);
		vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(1000));
	}
}
#include "led.hpp"

using namespace ThorDef::GPIO;

void ledStatus(void* argument)
{
	
	GPIOClass_sPtr ledPin = boost::make_shared<GPIOClass>(GPIOA, PIN_5, ULTRA_SPD, NOALTERNATE); /* LED Status Signal */
	
	/* Set up the GPIO Led */
	ledPin->mode(OUTPUT_PP);
	ledPin->write(LOW);
	
	CMDData_t pkt;
	
	TickType_t lastTimeWoken = xTaskGetTickCount();
	for (;;)
	{
		ledPin->write(HIGH);
		vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(150));
		ledPin->write(LOW);
		vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(1000));
	}
}
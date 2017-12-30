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
		/* Block this task until the queue has a new command for us. */
		xQueueReceive(qCommandBuffer, &pkt, portMAX_DELAY);
		
		if (pkt.rawPacket[0] == '2')
			ledPin->toggle();
		
		if (pkt.rawPacket[0] == '1')
			ledPin->write(HIGH);
		
		if (pkt.rawPacket[0] == '0')
			ledPin->write(LOW);
		
	}
}
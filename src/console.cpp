#include "console.hpp"



void consoleTask(void* argument)
{
	/* Initialize serial */
	UARTClass_sPtr uart = uart4;
 	uart->begin(115200);
 	uart->setTxModeDMA();
 	uart->setRxModeDMA();
	
	const char* NO_RX_MSG = "No cmd received\n";
	
	/* Packet read buffer */
	uint8_t readArr[ThorDef::UART::UART_BUFFER_SIZE];
	memset(readArr, 0, ThorDef::UART::UART_BUFFER_SIZE);
	
	/* Inform the Interrupt Task Manager that we want to be notified 
	 * of incoming RX packets */
	SemaphoreHandle_t rxPktRcvd = xSemaphoreCreateCounting(ThorDef::UART::UART_PACKET_QUEUE_SIZE, 0);
	EXTI0_TaskMGR->logEventConsumer(Interrupt::SRC_UART, 4, &rxPktRcvd);
	
	TickType_t lastTimeWoken = xTaskGetTickCount();
	for (;;)
	{
		
		if (xSemaphoreTake(rxPktRcvd, 0) == pdPASS)
		{
			/* Echo the packet back */
			if (uart->availablePackets() > 0)
			{
				memset(readArr, 0, ThorDef::UART::UART_BUFFER_SIZE);
				size_t pktSize = uart->nextPacketSize();

				uart->readPacket(readArr, ThorDef::UART::UART_BUFFER_SIZE);

				uart->write(readArr, pktSize);
			}
		}
		else
			uart->write(NO_RX_MSG);
		
		vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(500));	
	}
}
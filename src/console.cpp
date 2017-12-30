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
	uint8_t pkt[ThorDef::UART::UART_BUFFER_SIZE];
	memset(pkt, 0, ThorDef::UART::UART_BUFFER_SIZE);
	
	/* Inform the Interrupt Task Manager that we want to be notified 
	 * of incoming RX packets */
	SemaphoreHandle_t rxPktRcvd = xSemaphoreCreateCounting(ThorDef::UART::UART_PACKET_QUEUE_SIZE, 0);
	EXTI0_TaskMGR->logEventConsumer(Interrupt::SRC_UART, 4, &rxPktRcvd);
	
	bool parsePacket = false;
	CMDData_t cmdPkt;
	
	TickType_t lastTimeWoken = xTaskGetTickCount();
	for (;;)
	{
		/* Look for a new incoming command packet! */
		if (xSemaphoreTake(rxPktRcvd, 0) == pdPASS)
		{
			if (uart->availablePackets())
			{
				memset(pkt, 0, ThorDef::UART::UART_BUFFER_SIZE);
				size_t pktSize = uart->nextPacketSize();

				uart->readPacket(pkt, ThorDef::UART::UART_BUFFER_SIZE);
			
				/* We should look at the preamble to decide where to send it. */

				cmdPkt.priority = 0;
				cmdPkt.rawPacket = pkt;
				cmdPkt.rawPacketSize = pktSize;
				cmdPkt.rxTimeStamp = 0.0;

				xQueueSendToBack(qCommandBuffer, &cmdPkt, 0);
			}
		}
		
		vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(500));	
	}
}
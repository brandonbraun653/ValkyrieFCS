#include "sdcard.hpp"

/* C/C++ Includes */
#include <stdint.h>
#include <stdlib.h>

/* Thor Includes */
#include "include/thor.h"

/* ValkyrieRTX Includes */
#include "rtx.hpp"

/* FreeRTOS Includes */
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"



void radioTask(void* argument)
{
	/* Initialize serial */
	UARTClass_sPtr uart = uart5;
	uart->begin(115200);
	uart->setTxModeDMA();
	uart->setRxModeDMA();
	
	/* Inform the Interrupt Task Manager that we want to be notified of incoming RX packets */
	SemaphoreHandle_t rxPktRcvd = xSemaphoreCreateCounting(ThorDef::UART::UART_PACKET_QUEUE_SIZE, 0);
	EXTI0_TaskMGR->logEventConsumer(Interrupt::SRC_UART, 5, &rxPktRcvd);
	
	/* Raw serial input data */
	uint8_t rawPkt[ThorDef::UART::UART_BUFFER_SIZE];
	memset(rawPkt, 0, ThorDef::UART::UART_BUFFER_SIZE);
	
	/* Actual packet data */
	packetData* pkt;
	size_t pktSize = sizeof(pkt);
	uint8_t dummyBuffer[pktSize];
	
	memset((void*)&pkt, 0, pktSize);
	
	
	for(;;)	
	{
		//NOTE: Eventually I am going to have to provide some sort of error handler for missed packets
		//		while debugging this software. It's not like the radio quits transmitting.
		
		
		/* Wake up the thread when there is new data to process from the XBEE */
		xSemaphoreTake(rxPktRcvd, portMAX_DELAY);
		
		if (uart->availablePackets())
		{
			memset(rawPkt, 0, ThorDef::UART::UART_BUFFER_SIZE);
			size_t rawPktSize = uart->nextPacketSize();

			uart->readPacket(rawPkt, rawPktSize);
			
			/* Check the first byte of the received packet to determine what kind of data it is */
			switch (rawPkt[0])
			{
			case PACKET_CONTROLS:
				
				memcpy(dummyBuffer, rawPkt, pktSize);				/* Copy the correct amount of data */
				pkt = reinterpret_cast<packetData*>(dummyBuffer);	/* Reinterpret the data as a packet */
				
				//now I need to copy the data into a queue for transmission over the other serial port
				
				break;
				
			default:
				//Eventually log an error through the console
				break;
			}
			
			//xQueueSendToBack(qCommandBuffer, &cmdPkt, 0);
		}
	}
}
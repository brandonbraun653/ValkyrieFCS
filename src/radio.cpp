#include "sdcard.hpp"

/* C/C++ Includes */
#include <stdint.h>
#include <string.h>

/* Thor Includes */
#include "include/thor.h"
#include "include/uart.h"

/* ValkyrieRTX Includes */
#include "rtx.hpp"

/* FreeRTOS Includes */
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

/* Project Includes */
#include "dataTypes.hpp"

#define RADIO_QUEUE_SIZE 10
QueueHandle_t qRadio_Command = xQueueCreate(RADIO_QUEUE_SIZE, sizeof(Radio_Command));
QueueHandle_t qRadio_Control = xQueueCreate(RADIO_QUEUE_SIZE, sizeof(Radio_Control));
QueueHandle_t qRadio_Request = xQueueCreate(RADIO_QUEUE_SIZE, sizeof(Radio_Request));

void radioTask(void* argument)
{
	/*------------------------------
	* Initialize Serial Comms 
	*-----------------------------*/
	UARTClass_sPtr uart = uart5;
	uart->begin(115200);
	uart->setTxModeDMA();
	uart->setRxModeDMA();
	
	/* Make sure we are notified of incoming RX events */
	SemaphoreHandle_t rxPktRcvd = xSemaphoreCreateCounting(ThorDef::UART::UART_PACKET_QUEUE_SIZE, 0);
	EXTI0_TaskMGR->logEventConsumer(Interrupt::SRC_UART, 5, &rxPktRcvd);
	
	/*------------------------------
	* Initialize local buffers  
	*-----------------------------*/
	uint8_t rawPkt[ThorDef::UART::UART_BUFFER_SIZE];	/* Generic receive buffer */
	memset(rawPkt, 0, sizeof(rawPkt));
	
	packetData pkt;										/* Formatted packet that is RX'd */
	memset(&pkt, 0, sizeof(pkt));
	
	
	for(;;)	
	{		
		xSemaphoreTake(rxPktRcvd, portMAX_DELAY);	/* Wake up ONLY when there is new XBEE data */
		
		if (uart->availablePackets())
		{
			memset(rawPkt, 0, sizeof(rawPkt));					/* Reset the buffer to 0 */
			uart->readPacket(rawPkt, uart->nextPacketSize());	/* Read in all the data */
			memcpy(&pkt, rawPkt, sizeof(pkt));					/* Copy into the packet struct */
			
			//Note: Probably should start handling multiple packet types rather than a giant all
			//encompassing one...Discretization is my friend here. I have a variable length async
			//uart library, may as well use it. 
			
			if (rawPkt[0] == PACKET_CONTROLS)
			{
				Radio_Control temp;
				temp.THROTTLE	= pkt.THROTTLE;
				temp.ROLL		= pkt.ROLL;
				temp.PITCH		= pkt.PITCH;
				temp.YAW		= pkt.YAW;
				
				/* Data is actually copied (see documentation) and not referenced. Don't wait
				 * for confirmation that data was queued successfully. Something is very very 
				 * wrong with the main control loop if it cannot keep up with the radio rates. */
				xQueueSendToBack(qRadio_Control, &temp, 0);
			}
			
			else if (rawPkt[0] == PACKET_COMMAND)
			{
				//This one does a lot...need to figure out what kind of commands will be supported.
				
				/* 1. Arm
				 * 2. Flying Modes
				 *		a. Acro
				 *		b. Auto-level
				 * 3. Data Record 
				 * 4. ?
				 * */
			}
			
			else if (rawPkt[0] == PACKET_REQUEST)
			{
				//Asks the main thread for some data...main thread should handle this asynchronously rather
				//than assigning a specific task to it. Very very low priority.
			}
			else
			{
				//log some error to console
			}
			
		}
	}
}
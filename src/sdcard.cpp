#include "sdcard.hpp"

using namespace ThorDef::SPI;
using namespace ThorDef::GPIO;



void sdCardTask(void* argument)
{
	/* Assign the object pointers */
	SPIClass_sPtr sd_spi = spi3;
	SDCard_sPtr sd = boost::make_shared<SDCard>(sd_spi);
	GPIOClass_sPtr SD_SSPin = boost::make_shared<GPIOClass>(GPIOA, PIN_15, ULTRA_SPD, NOALTERNATE); /* SD Slave Select */
	
	
	/* Set up SPI */
	sd_spi->attachPin(SD_SSPin);
	sd_spi->setSSMode(SS_MANUAL_CONTROL);
	sd_spi->begin(EXTERNAL_SLAVE_SELECT);
	
	/* Initialize the SD Card */
	FRESULT error = FR_OK;
	portENTER_CRITICAL();
	error = sd->initialize();//true, FM_FAT32);
	portEXIT_CRITICAL();
	
	IMUData_t logData;
	size_t logDataSize = sizeof(logData);
	uint32_t bytesWritten = 0, dummyWrite = 0;
	
	const char* testMessage = "Hello from the Valkyrie FCS board!!!";
	
	//error = sd->fopen("dataFile", FA_CREATE_ALWAYS | FA_WRITE);
	error = sd->fopen("FCSMessage.txt", FA_CREATE_ALWAYS | FA_WRITE);
	error = sd->write((uint8_t*)testMessage, logDataSize, bytesWritten);
	error = sd->fclose();
	
	TickType_t xLastWakeTime = xTaskGetTickCount();
	for(;;)	
	{
//		if (error == FR_OK)
//		{		
//			/* Indefinitely block the task if no data is ready...I think this is right? */
//			xQueueReceive(qIMUFlightData, &logData, portMAX_DELAY);
//			
//			/* Unfortunately this takes FOREVER to send a block of data...2-3mS AND it blocks
//			 * all the other tasks from running!! */
//			portENTER_CRITICAL();
//			error = sd->write(reinterpret_cast<uint8_t*>(&logData), logDataSize, bytesWritten);
//			portEXIT_CRITICAL();
//		}
		
		/* Let others run */
		taskYIELD();
		
		/* Error condition visual confirm fast led blink*/
//		else
//		{
//			ledPin->write(HIGH);
//			vTaskDelay(pdMS_TO_TICKS(150));
//			ledPin->write(LOW);
//			vTaskDelay(pdMS_TO_TICKS(150));
//			vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
//		}
		
		/* Send the status of SD read/writes out to another task that solely handles 
		 * blinking leds for the user. */
		
	}
}
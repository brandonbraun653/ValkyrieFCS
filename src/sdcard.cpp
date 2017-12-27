#include "sdcard.hpp"

using namespace ThorDef::SPI;
using namespace ThorDef::GPIO;

/* Try setting up some local variables for use in the task */
SPIClass_sPtr sd_spi;
SDCard_sPtr sd;
GPIOClass_sPtr SD_SSPin;
GPIOClass_sPtr ledPin;

const char* msg = "Hello from the sdCardTask thread.";
uint32_t bw;

void sdCardTaskInit()
{
	/* Assign the object pointers */
	sd_spi = spi2;
	sd = boost::make_shared<SDCard>(sd_spi);
	SD_SSPin = boost::make_shared<GPIOClass>(GPIOB, PIN_12, ULTRA_SPD, NOALTERNATE); /* SD Slave Select */
	ledPin = boost::make_shared<GPIOClass>(GPIOA, PIN_5, ULTRA_SPD, NOALTERNATE); /* LED Status Signal */
	
	/* Set up SPI */
	sd_spi->attachPin(SD_SSPin);
	sd_spi->setSSMode(SS_MANUAL_CONTROL);
	sd_spi->begin(EXTERNAL_SLAVE_SELECT);
	
	/* Initialize the SD Card */
	sd->initialize();
}

void sdCardTask(void* argument)
{
	
	sdCardTaskInit();
	
	TickType_t xLastWakeTime = xTaskGetTickCount();
	
	for(;;)	
	{
		sd->fopen("thw.txt", FA_CREATE_ALWAYS | FA_WRITE);
		sd->write((uint8_t*)msg, strlen(msg), bw);
		sd->fclose();
	
		ledPin->write(HIGH);
		vTaskDelay(pdMS_TO_TICKS(250));
		ledPin->write(LOW);
		vTaskDelay(pdMS_TO_TICKS(250));
		
		
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(2000));
	}
	
}
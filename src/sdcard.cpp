#include "sdcard.hpp"

using namespace ThorDef::SPI;
using namespace ThorDef::GPIO;

/* Fake set of data that might get logged. Only the length 
 * and data type is important at the moment.*/
double data[] = {
	1.3423,
	7.4554,
	8.3224, 
	1.3423,
	7.4554,
	8.3224,
	1.3423,
	7.4554,
	8.3224,
};

uint32_t bw;

void sdCardTask(void* argument)
{
	/* Assign the object pointers */
	SPIClass_sPtr sd_spi = spi2;
	SDCard_sPtr sd = boost::make_shared<SDCard>(sd_spi);
	GPIOClass_sPtr SD_SSPin = boost::make_shared<GPIOClass>(GPIOB, PIN_12, ULTRA_SPD, NOALTERNATE); /* SD Slave Select */
	GPIOClass_sPtr ledPin = boost::make_shared<GPIOClass>(GPIOA, PIN_5, ULTRA_SPD, NOALTERNATE); /* LED Status Signal */
	
	/* Set up the GPIO Led */
	ledPin->mode(OUTPUT_PP);
	ledPin->write(LOW);
	
	/* Set up SPI */
	sd_spi->attachPin(SD_SSPin);
	sd_spi->setSSMode(SS_MANUAL_CONTROL);
	sd_spi->begin(EXTERNAL_SLAVE_SELECT);
	
	/* Initialize the SD Card */
	FRESULT error = FR_OK;
	portENTER_CRITICAL();
	error = sd->initialize(true, FM_FAT32);
	portEXIT_CRITICAL();
	
	TickType_t xLastWakeTime = xTaskGetTickCount();
	error = sd->fopen("dataFile", FA_CREATE_ALWAYS | FA_WRITE);
	
	for(;;)	
	{
		if (error == FR_OK)
		{
			/* Test out how quickly a write can occur for 1 unit log.
			 * Results:
			 * 1st write == ~4-5mS
			 * all other == ~60uS
			 * 
			 * It would appear that only the first write takes a long time.
			 * All subsequent writes to that particular file take a very miniscule
			 * amount of time. This is great news. 
			 * */
			portENTER_CRITICAL();
			error = sd->write((uint8_t*)data, sizeof(data)/sizeof(data[0]), bw);
			portEXIT_CRITICAL();
			
			ledPin->write(HIGH);
			vTaskDelay(pdMS_TO_TICKS(100));
			ledPin->write(LOW);
			vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
		}
		
		/* Error condition visual confirm fast led blink*/
		else
		{
			ledPin->write(HIGH);
			vTaskDelay(pdMS_TO_TICKS(150));
			ledPin->write(LOW);
			vTaskDelay(pdMS_TO_TICKS(150));
			vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
		}
	}
}
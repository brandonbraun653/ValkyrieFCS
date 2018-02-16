/* C/C++ Includes */
#include <stdint.h>
#include <stdlib.h>

/* Thor Includes */
#include "include/thor.h"
#include "include/spi.h"
#include "include/gpio.h"

/* External Library Includes */
#include "sd.h"

/* Boost Includes */
#include <boost/smart_ptr.hpp>
#include <boost/make_shared.hpp>

/* FreeRTOS Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Project Includes */
#include "dataTypes.hpp"
#include "threading.hpp"
#include "sdcard.hpp"

using namespace ThorDef::SPI;
using namespace ThorDef::GPIO;

uint32_t taskNotification = 0;

uint32_t writeCount = 0;
uint32_t maxWriteCount = 10000; //5 seconds


void parseAHRSData(AHRSData_t& ahrs, SD_LOG_AHRS_t& sdlog);


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
	
	AHRSData_t rawAHRSData;
	SD_LOG_AHRS_t ahrsDataToLog;

	ahrsDataToLog.euler_deg_pitch = 0.0f;
	ahrsDataToLog.euler_deg_roll = 1.0f;
	ahrsDataToLog.euler_deg_yaw = 2.0f;
	ahrsDataToLog.accel_x = 3.0f;
	ahrsDataToLog.accel_y = 4.0f;
	ahrsDataToLog.accel_z = 5.0f;
	ahrsDataToLog.gyro_x = 6.0f;
	ahrsDataToLog.gyro_y = 7.0f;
	ahrsDataToLog.gyro_z = 8.0f;
	ahrsDataToLog.mag_x = 9.0f;
	ahrsDataToLog.mag_y = 99.0f;
	ahrsDataToLog.mag_z = 87.0f;
		
	uint32_t bytesWritten = 0;
	
	error = sd->fopen("flightLog.dat", FA_CREATE_ALWAYS | FA_WRITE);
	
	size_t t = sizeof(SD_LOG_AHRS_t);
	
	TickType_t xLastWakeTime = xTaskGetTickCount();
	for(;;)	
	{
		/* Wake up only when we have new data */
		xQueueReceive(qSD_AHRSData, &rawAHRSData, portMAX_DELAY);
		
	
		
		if (error == FR_OK && (writeCount < maxWriteCount))
		{		
			parseAHRSData(rawAHRSData, ahrsDataToLog);
			
			error = sd->write(reinterpret_cast<uint8_t*>(&ahrsDataToLog), sizeof(SD_LOG_AHRS_t), bytesWritten);
			writeCount++;
			
			if (writeCount == maxWriteCount)
			{
				sd->fclose();
				sd->deInitialize();
				xTaskSendMessage(LED_STATUS_TASK, (LED_RED | LED_STATIC_ON));
			}
		}
	}
	
	
	/* Ensure a clean deletion of the task if we exit */
	vTaskDelete(NULL);
}


void parseAHRSData(AHRSData_t& ahrs, SD_LOG_AHRS_t& sdlog)
{
	sdlog.euler_deg_pitch = ahrs.pitch();
	sdlog.euler_deg_roll = ahrs.roll();
	sdlog.euler_deg_yaw = ahrs.yaw();
	
	sdlog.accel_x = ahrs.ax();
	sdlog.accel_y = ahrs.ay();
	sdlog.accel_z = ahrs.az();
	
	sdlog.gyro_x = ahrs.gx();
	sdlog.gyro_y = ahrs.gy();
	sdlog.gyro_z = ahrs.gz();
	
	sdlog.mag_x = ahrs.mx();
	sdlog.mag_y = ahrs.my();
	sdlog.mag_z = ahrs.mz();
}
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


namespace FCS_SD
{
	const char* ahrsLogFilename = "ahrsLogMinimal.dat";
	const char* motorLogFilename = "motorLog.dat";
	FIL ahrsLogFile;
	FIL motorLogFile;

	const int updateRate_mS = (1.0f / SDCARD_UPDATE_FREQ_HZ) * 1000.0f;


	uint32_t writeCount = 0;
	uint32_t maxWriteCount = 50000;

	bool loggerEnabled = false;

	void parseAHRSData(AHRSData_t& ahrs, SDLOG_AHRS_Full_t& sdlog)
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

	void parseTaskNotification(uint32_t notification)
	{
		if (notification == SD_CARD_ENABLE_IO)
			loggerEnabled = true;

		if (notification == SD_CARD_DISABLE_IO)
			loggerEnabled = false;
	}

	/* Assumes the file exists and can be appended to */
	template<typename qType>
	FRESULT writeQueueToFile(SDCard_sPtr& sd, FIL& file, QueueHandle_t queue, qType& data)
	{
		uint32_t temp = 0;
		FRESULT error;
		while (uxQueueMessagesWaiting(queue) > (UBaseType_t)0)
		{
			xQueueReceive(queue, &data, 0);
			error = sd->write(file, reinterpret_cast<uint8_t*>(&data), sizeof(data), temp);
		}

		return error;
	}



	void sdCardTask(void* argument)
	{
		/* Assign the object pointers */
		SPIClass_sPtr sd_spi = spi3;
		SDCard_sPtr sd = boost::make_shared<SDCard>(sd_spi);
		GPIOClass_sPtr SD_SSPin = boost::make_shared<GPIOClass>(GPIOA, PIN_15, ULTRA_SPD, NOALTERNATE);


		/* Set up SPI */
		sd_spi->attachPin(SD_SSPin);
		sd_spi->setSSMode(SS_MANUAL_CONTROL);
		sd_spi->begin(EXTERNAL_SLAVE_SELECT);

		/* Initialize the SD Card */
		FRESULT error = FR_OK;
		portENTER_CRITICAL();
		error = sd->initialize();//true, FM_FAT32);
		portEXIT_CRITICAL();

		SDLOG_AHRS_Full_t ahrsLogDataFull;
		SDLOG_AHRS_Minimal_t ahrsLogDataMinimal;
		SDLOG_Motors_t motorLogData;

		uint32_t bytesWritten = 0;

		/*-------------------------------------
		* Set up the various files 
		*------------------------------------*/
		error = sd->fopen(ahrsLogFilename, FA_CREATE_ALWAYS | FA_WRITE, ahrsLogFile);
		//Write the header argument here...
		

		error = sd->fopen(motorLogFilename, FA_CREATE_ALWAYS | FA_WRITE, motorLogFile);
		//Write the header argument here...


		TickType_t lastTimeWoken = xTaskGetTickCount();
		for (;;)
		{
			parseTaskNotification(ulTaskNotifyTake(pdTRUE, 0));

			if (loggerEnabled)
			{
				if (error == FR_OK && (writeCount < maxWriteCount))
				{
					error = writeQueueToFile(sd, ahrsLogFile, qSD_AHRSMinimal, ahrsLogDataMinimal);
					error = writeQueueToFile(sd, motorLogFile, qSD_Motor, motorLogData);

					writeCount++;
				}
				else
				{
					sd->fclose(ahrsLogFile);
					sd->fclose(motorLogFile);

					sd->deInitialize();
					xTaskSendMessage(LED_STATUS_TASK, (LED_RED | LED_STATIC_ON));
				}
			}	

			vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(updateRate_mS));
		}


		/* Ensure a clean deletion of the task if we exit */
		vTaskDelete(NULL);
	}

}

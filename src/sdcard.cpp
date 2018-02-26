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
	

	//const char* ahrsLogFilename = "ahrsLogMinimal.dat";
	const char* ahrsLogFilename = "ahrsLogFull.dat";
	const char* motorLogFilename = "motorLog.dat";
	const char* pidAngleSetpointLogFilename = "angleSetpoints.dat";
	const char* pidRateSetpointLogFilename = "rateSetpoints.dat";
	
	FIL ahrsLogFile;
	FIL motorLogFile;
	FIL pidAngleSetpointLogFile;
	FIL pidRateSetpointLogFile;
	
	const int updateRate_mS = (1.0f / SDCARD_UPDATE_FREQ_HZ) * 1000.0f;


	uint32_t writeCount = 0;
	uint32_t maxWriteCount = 1000;

	bool loggerEnabled = false;
	bool cleanlyExit = false;

	void parseTaskNotification(uint32_t notification)
	{
		if (notification == SD_CARD_ENABLE_IO)
			loggerEnabled = true;

		if (notification == SD_CARD_DISABLE_IO)
			loggerEnabled = false;
		
		if (notification == SD_CARD_SHUTDOWN)
		{
			loggerEnabled = false;
			cleanlyExit = true;
		}
	}

	/* Assumes the file exists and can be appended to */
	template<typename qType>
	FRESULT writeQueueToFile(SDCard_sPtr& sd, FIL& file, QueueHandle_t queue, qType& data)
	{
		uint32_t temp = 0;
		FRESULT error = FR_OK;
		while (uxQueueMessagesWaiting(queue) > (UBaseType_t)0)
		{
			xQueueReceive(queue, &data, 0);
			error = sd->write(file, reinterpret_cast<uint8_t*>(&data), sizeof(data), temp);
		}

		return error;
	}



	void sdCardTask(void* argument)
	{
		#ifdef DEBUG
		volatile UBaseType_t stackHighWaterMark_SDCARD = 0;
		#endif

		/* Assign the object pointers */
		SPIClass_sPtr sd_spi = spi3;
		SDCard_sPtr sd = boost::make_shared<SDCard>(sd_spi);
		GPIOClass_sPtr SD_SSPin = boost::make_shared<GPIOClass>(GPIOA, PIN_15, ULTRA_SPD, NOALTERNATE);


		/* Set up SPI */
		sd_spi->attachPin(SD_SSPin);
		sd_spi->setSSMode(SS_MANUAL_CONTROL);
		sd_spi->begin(EXTERNAL_SLAVE_SELECT);

		
		/* Initialize the SD Card */
		#ifdef DEBUG
		volatile FRESULT error = FR_OK;
		#else
		FRESULT error = FR_OK;
		#endif
	
		error = sd->initialize();//true, FM_FAT32);
		
		/*-------------------------------------
		* Open the log files for writing 
		*------------------------------------*/
		error = sd->fopen(ahrsLogFilename, FA_CREATE_ALWAYS | FA_WRITE, ahrsLogFile);
		error = sd->fopen(motorLogFilename, FA_CREATE_ALWAYS | FA_WRITE, motorLogFile);
		error = sd->fopen(pidAngleSetpointLogFilename, FA_CREATE_ALWAYS | FA_WRITE, pidAngleSetpointLogFile);
		error = sd->fopen(pidRateSetpointLogFilename, FA_CREATE_ALWAYS | FA_WRITE, pidRateSetpointLogFile);


		SDLOG_AHRS_Full_t ahrsLogDataFull;
		SDLOG_AHRS_Minimal_t ahrsLogDataMinimal;
		SDLOG_Motors_t motorLogData;
		SDLOG_PIDAngleInput_t pidAngleSetpoints;
		SDLOG_PIDRateInput_t pidRateSetpoints;

		uint32_t bytesWritten = 0;
		
		/* Tell init task that this thread's initialization is done and ok to run.
		 * Wait for init task to resume operation. */
		xTaskSendMessage(INIT_TASK, 1u);
		vTaskSuspend(NULL);
		taskYIELD();
		
		
		TickType_t lastTimeWoken = xTaskGetTickCount();
		for (;;)
		{
			#ifdef DEBUG
			activeTask = SDCARD_TASK;
			stackHighWaterMark_SDCARD = uxTaskGetStackHighWaterMark(NULL);
			#endif

			parseTaskNotification(ulTaskNotifyTake(pdTRUE, 0));

			if (loggerEnabled)
			{
				if (error == FR_OK && (writeCount < maxWriteCount))
				{
					error = writeQueueToFile(sd, ahrsLogFile, qSD_AHRSFull, ahrsLogDataFull);
					error = writeQueueToFile(sd, motorLogFile, qSD_Motor, motorLogData);
					error = writeQueueToFile(sd, pidAngleSetpointLogFile, qSD_PIDAngleSetpoint, pidAngleSetpoints);
					error = writeQueueToFile(sd, pidRateSetpointLogFile, qSD_PIDRateSetpoint, pidRateSetpoints);
			
					writeCount++;
				}
				else
				{
					sd->fclose(ahrsLogFile);
					sd->fclose(motorLogFile);
					sd->fclose(pidAngleSetpointLogFile);
					sd->fclose(pidRateSetpointLogFile);

					sd->deInitialize();
					xTaskSendMessage(LED_STATUS_TASK, (LED_RED | LED_STATIC_ON));
				}
			}	

			if (cleanlyExit)
			{
				sd->fclose(ahrsLogFile);
				sd->fclose(motorLogFile);
				sd->fclose(pidAngleSetpointLogFile);
				sd->fclose(pidRateSetpointLogFile);

				sd->deInitialize();
				xTaskSendMessage(LED_STATUS_TASK, (LED_RED | LED_STATIC_ON));
			}

			vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(updateRate_mS));
		}


		/* Ensure a clean deletion of the task if we exit */
		TaskHandle[SDCARD_TASK] = (void*)0;  	//Deletes our personal log of this task's existence
		vTaskDelete(NULL);						//Deletes the kernel's log of this task's existence
	}

}

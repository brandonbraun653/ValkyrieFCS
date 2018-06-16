/* C/C++ Includes */
#include <stdint.h>
#include <stdlib.h>

/* Boost Includes */
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

/* Chimera Includes */
#include <Chimera/gpio.hpp>

/* FreeRTOS Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* External Library Includes */
#include <SDCard/sd.h>

/* Project Includes */
#include <ValkyrieFCS/include/dataTypes.hpp>
#include <ValkyrieFCS/include/threads.hpp>
#include <ValkyrieFCS/include/sdcard.hpp>

#define SD_SPI_CHANNEL 3

using namespace SD;
using namespace Chimera::GPIO;

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
	uint32_t maxWriteCount = 200;

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
			error = sd->write(file, reinterpret_cast<uint8_t*>(&data), sizeof(qType), temp);
		}

		return error;
	}

	void sdCardTask(void* argument)
	{
		#ifdef DEBUG
		volatile UBaseType_t stackHighWaterMark_SDCARD = 0;
		#endif

		/* Assign the object pointers */
		SDCard_sPtr sd = boost::make_shared<SDCard>(SD_SPI_CHANNEL, PORTA, 15);

		/* Initialize the SD Card */
		volatile FRESULT error = FR_OK;
	
		error = sd->initialize();//true, FM_FAT32);
		
		/*-------------------------------------
		* Open the log files for writing 
		*------------------------------------*/
		error = sd->fopen(ahrsLogFilename, FA_CREATE_ALWAYS | FA_WRITE, ahrsLogFile);
		error = sd->fopen(motorLogFilename, FA_CREATE_ALWAYS | FA_WRITE, motorLogFile);
		error = sd->fopen(pidAngleSetpointLogFilename, FA_CREATE_ALWAYS | FA_WRITE, pidAngleSetpointLogFile);
		error = sd->fopen(pidRateSetpointLogFilename, FA_CREATE_ALWAYS | FA_WRITE, pidRateSetpointLogFile);

		uint8_t data[10];
		uint32_t temp;
		memset(data, 0xFF, 10);

		SDLOG_AHRS_Full_t ahrsLogDataFull;
		SDLOG_AHRS_Minimal_t ahrsLogDataMinimal;
		SDLOG_Motors_t motorLogData;
		SDLOG_PIDAngleInput_t pidAngleSetpoints;
		SDLOG_PIDRateInput_t pidRateSetpoints;

		uint32_t bytesWritten = 0;
	

		Chimera::Threading::signalThreadSetupComplete();

		//error = sd->write(ahrsLogFile, data, 10, temp);
		
		TickType_t lastTimeWoken = xTaskGetTickCount();
		for (;;)
		{
			#ifdef DEBUG
			activeTask = SDCARD_TASK;
			stackHighWaterMark_SDCARD = uxTaskGetStackHighWaterMark(NULL);
			#endif

			parseTaskNotification(ulTaskNotifyTake(pdTRUE, 0));

			if (true)
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
					//xTaskSendMessage(LED_STATUS_TASK, (LED_RED | LED_STATIC_ON));
				}
			}	

			if (cleanlyExit)
			{
				sd->fclose(ahrsLogFile);
				sd->fclose(motorLogFile);
				sd->fclose(pidAngleSetpointLogFile);
				sd->fclose(pidRateSetpointLogFile);

				sd->deInitialize();
				//xTaskSendMessage(LED_STATUS_TASK, (LED_RED | LED_STATIC_ON));
			}

			vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(updateRate_mS));
		}


		/* Ensure a clean deletion of the task if we exit */
		Chimera::Threading::deleteThread(NULL);
	}

}

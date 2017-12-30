#include "FreeRTOS.h"
#include "task.h"

#include "include/thor.h"

#ifdef DEBUG
#include "SysprogsProfiler.h"
#endif


/* Thread Task Includes */
#include "sdcard.hpp"
#include "ahrs.hpp"
#include "console.hpp"
#include "led.hpp"

using namespace ThorDef::GPIO;
using namespace ThorDef::SPI;

int main(void)
{
	HAL_Init();
	ThorInit();

	#ifdef DEBUG
	InitializeSamplingProfiler();
	//InitializeInstrumentingProfiler();
	#endif 
	
	xTaskCreate(sdCardTask, "sdTask", 2000, NULL, 1, NULL);
	xTaskCreate(ahrsTask, "ahrsTask", 2000, NULL, 1, NULL);
	xTaskCreate(consoleTask, "cmdListener", 2000, NULL, 1, NULL);
	xTaskCreate(ledStatus, "ledResponse", 2000, NULL, 1, NULL);
	
	vTaskStartScheduler();
	
	/* We will never reach here as the scheduler should have taken over */
	for (;;)
	{
	}
}
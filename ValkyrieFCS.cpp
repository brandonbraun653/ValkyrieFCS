#include "FreeRTOS.h"
#include "task.h"

#include "include/thor.h"

#ifdef DEBUG
#include "SysprogsProfiler.h"
#endif


/* Thread Task Includes */
#include "radio.hpp"
#include "sdcard.hpp"
#include "ahrs.hpp"
#include "console.hpp"
#include "led.hpp"

#include "fcsConfig.hpp"

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
	
	xTaskCreate(sdCardTask,		"sdTask",		2000,	NULL,	SDCARD_LOGGING_PRIORITY,	NULL);
	xTaskCreate(ahrsTask,		"ahrsTask",		2000,	NULL,	AHRS_UPDATE_PRIORITY,		NULL);
	xTaskCreate(consoleTask,	"cmdListener",	2000,	NULL,	CONSOLE_LOGGING_PRIORITY,	NULL);
	xTaskCreate(ledStatus,		"ledResponse",	512,	NULL,	STATUS_LEDS_PRIORITY,		NULL);
	xTaskCreate(radioTask,		"radio",		1000,	NULL,	RADIO_UPDATE_PRIORITY,		NULL);
	
	vTaskStartScheduler();
	
	/* We will never reach here as the scheduler should have taken over */
	for (;;)
	{
	}
}
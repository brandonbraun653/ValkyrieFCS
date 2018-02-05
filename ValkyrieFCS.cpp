#include "FreeRTOS.h"
#include "task.h"

#include "include/thor.h"

#ifdef DEBUG
#include "SysprogsProfiler.h"
#endif

/* Thread Task Includes */
#include "fcsConfig.hpp"
#include "radio.hpp"
#include "sdcard.hpp"
#include "sensor.hpp"
#include "ahrs.hpp"
#include "motors.hpp"
#include "console.hpp"
#include "led.hpp"
#include "bluetooth.hpp"
#include "threading.hpp"
#include "pid.hpp"

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
	
	
	xTaskCreate(sensorTask,		"sensor",		2000,	NULL,	SENSOR_UPDATE_PRIORITY,		&TaskHandle[SENSOR_TASK]);
	//xTaskCreate(sdCardTask,		"sdTask",		2000,	NULL,	SDCARD_LOGGING_PRIORITY,	&TaskHandle[SDCARD_TASK]);
	xTaskCreate(ahrsTask,		"ahrsTask",		2000,	NULL,	AHRS_UPDATE_PRIORITY,		&TaskHandle[AHRS_TASK]);
	//xTaskCreate(consoleTask,	"cmdListener",	2000,	NULL,	CONSOLE_LOGGING_PRIORITY,	&TaskHandle[CONSOLE_TASK]);
	//xTaskCreate(ledStatus,		"ledStatus",	512,	NULL,	STATUS_LEDS_PRIORITY,		&TaskHandle[LED_STATUS_TASK]);
	//xTaskCreate(radioTask,		"radio",		1000,	NULL,	RADIO_UPDATE_PRIORITY,		&TaskHandle[RADIO_TASK]);
	xTaskCreate(motorTask,		"motor",		1000,	NULL,	MOTOR_UPDATE_PRIORITY,		&TaskHandle[MOTOR_TASK]);
	//xTaskCreate(bluetoothTask,	"bluetooth",	1000,	NULL,	BLUETOOTH_PRIORITY,			&TaskHandle[BLUETOOTH_TASK]);
	xTaskCreate(pidTask,		"pid",			2000,	NULL,	PID_UPDATE_PRIORITY,		&TaskHandle[PID_TASK]);
	
	vTaskStartScheduler();
	
	/* We will never reach here as the scheduler should have taken over */
	for (;;)
	{
	}
}
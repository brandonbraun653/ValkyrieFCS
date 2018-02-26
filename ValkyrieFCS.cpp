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
#include "ahrs.hpp"
#include "motors.hpp"
#include "console.hpp"
#include "led.hpp"
#include "bluetooth.hpp"
#include "threading.hpp"
#include "pid.hpp"

using namespace ThorDef::GPIO;
using namespace ThorDef::SPI;

void init(void* parameter);

int main(void)
{
	HAL_Init();
	ThorInit();
	
	#ifdef DEBUG
	InitializeSamplingProfiler();
	InitializeInstrumentingProfiler();
	#endif 

	xTaskCreate(init, "init", 500, NULL, 1, &TaskHandle[INIT_TASK]);
	vTaskStartScheduler();
	
	/* We will never reach here as the scheduler should have taken over */
	for (;;)
	{
	}
}

void init(void* parameter)
{
	/* Initialize the pointers to all task handles (except for INIT_TASK). This 
	 * prevents errors in calling xTaskSendMessage should a task no longer exist.
	 * */
	for (int task = 1; task < TOTAL_TASK_SIZE; task++)
		TaskHandle[task] = (TaskHandle_t)0;

	volatile BaseType_t error = pdPASS;
	TickType_t lastTimeWoken = xTaskGetTickCount();
	
	/* Create a task and then wait until its initialization sequence has finished before continuing.
	 * The tasks MUST be created in this order:
	 * 1. LedStatus
	 * 2. SDCard
	 * 3. PID
	 * 4. Motor (This depends on [2,3] being started) 
	 * 5. AHRS
	 * */
	
	error = xTaskCreate(FCS_LED::ledStatus, "ledTask", 350, NULL, STATUS_LEDS_PRIORITY, &TaskHandle[LED_STATUS_TASK]);
	while (!ulTaskNotifyTake(pdTRUE, 0))
		vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(10));
	
	
	error = xTaskCreate(FCS_SD::sdCardTask, "sdTask", 350, NULL, SDCARD_LOGGING_PRIORITY, &TaskHandle[SDCARD_TASK]);
	while (!ulTaskNotifyTake(pdTRUE, 0))
		vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(10));
	
	
	error = xTaskCreate(FCS_PID::pidTask, "pidTask", 350, NULL, PID_UPDATE_PRIORITY, &TaskHandle[PID_TASK]);
	while (!ulTaskNotifyTake(pdTRUE, 0))
		vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(10));
	
	
	error = xTaskCreate(FCS_MOTOR::motorTask, "motorTask", 350, NULL, MOTOR_UPDATE_PRIORITY, &TaskHandle[MOTOR_TASK]);
	while (!ulTaskNotifyTake(pdTRUE, 0))
		vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(10));
	
	
	error = xTaskCreate(FCS_AHRS::ahrsTask, "ahrsTask", 8000, NULL, AHRS_UPDATE_PRIORITY, &TaskHandle[AHRS_TASK]);
	while (!ulTaskNotifyTake(pdTRUE, 0))
		vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(10));
	
	
	//xTaskCreate(consoleTask,	"cmdListener",	2000,	NULL,	CONSOLE_LOGGING_PRIORITY,	&TaskHandle[CONSOLE_TASK]);
	//xTaskCreate(radioTask,		"radio",		1000,	NULL,	RADIO_UPDATE_PRIORITY,		&TaskHandle[RADIO_TASK]);
	//xTaskCreate(bluetoothTask,	"bluetooth",	1000,	NULL,	BLUETOOTH_PRIORITY,			&TaskHandle[BLUETOOTH_TASK]);


	#ifdef DEBUG
	volatile TaskHandle_t hSD		= TaskHandle[SDCARD_TASK];
	volatile TaskHandle_t hAhrs		= TaskHandle[AHRS_TASK];
	volatile TaskHandle_t hLed		= TaskHandle[LED_STATUS_TASK];
	volatile TaskHandle_t hMotor	= TaskHandle[MOTOR_TASK];
	volatile TaskHandle_t hPid		= TaskHandle[PID_TASK];
	volatile size_t bytesRemaining	= xPortGetFreeHeapSize();
	#endif
	
	
	if (error == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY)
	{
		/* If you hit this point, one of the above tasks tried to allocate more heap space 
		 * than was available. Run in a Debug build and check the bytesRemaining variable. */
		for (;;) {}
	}
	
	
	/* Resume all tasks in the correct order */
	vTaskResume(TaskHandle[LED_STATUS_TASK]);
	vTaskResume(TaskHandle[SDCARD_TASK]);
	vTaskResume(TaskHandle[PID_TASK]);
	vTaskResume(TaskHandle[MOTOR_TASK]);
	vTaskResume(TaskHandle[AHRS_TASK]);
	
	/* Ensure a clean deletion of the task upon exit */
	TaskHandle[INIT_TASK] = (void*)0;	//Deletes our personal log of this task's existence
	vTaskDelete(NULL);					//Deletes the kernel's log of this task's existence
}
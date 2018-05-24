#pragma once
#ifndef MESSAGING_HPP_
#define MESSAGING_HPP_

/* Boost Includes */
#include <boost/container/vector.hpp>

/* FreeRTOS Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Thor Includes */
#include <Thor/include/threading.hpp>

/* Project Includes */
#include <ValkyrieFCS/include/fcsConfig.hpp>
#include <ValkyrieFCS/include/dataTypes.hpp>

/*----------------------------------
* Queues
*----------------------------------*/
extern QueueHandle_t qIMU;		/* Buffers raw data from IMU for the AHRS algorithm */
extern QueueHandle_t qAHRS;		/* Buffers processed AHRS data for the PID controller */

#if USING_PID_CONTROL
extern QueueHandle_t qPID;		/* Buffers PID output data for the Motor controller */
#endif
#if USING_LQR_CONTROL
extern QueueHandle_t qLQR;		/* Buffers LQR output data for the Motor controller */
#endif

extern QueueHandle_t qSD_AHRSFull;
extern QueueHandle_t qSD_AHRSMinimal;
extern QueueHandle_t qSD_Motor;
extern QueueHandle_t qSD_PIDAngleSetpoint;
extern QueueHandle_t qSD_PIDRateSetpoint;


/*----------------------------------
* Semaphores and Mutexes
*----------------------------------*/
extern SemaphoreHandle_t ahrsBufferMutex; /* Protects data passing between AHRS and PID loops when
											 running at differing frequencies */

extern SemaphoreHandle_t pidBufferMutex;  /* Protects data passing between PID and Motor control loops */

extern SemaphoreHandle_t lqrBufferMutex;  /* Protects data passing between LQR and Motor control loops*/


enum TaskIndex
{
	INIT_TASK,
	AHRS_TASK,
	SENSOR_TASK,
	MOTOR_TASK,
	SDCARD_TASK,
	CONSOLE_TASK,
	LED_STATUS_TASK,
	RADIO_TASK,
	BLUETOOTH_TASK,
	CTRL_TASK,
	
	TOTAL_TASK_SIZE
};

extern TaskIndex activeTask; /* A simple debugging flag to check which task is currently running */


#endif
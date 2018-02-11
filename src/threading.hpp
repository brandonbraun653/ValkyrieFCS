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

/* Project Includes */
#include "fcsConfig.hpp"
#include "dataTypes.hpp"

/*----------------------------------
* Queues
*----------------------------------*/
extern QueueHandle_t qIMU;		/* Buffers raw data from IMU for the AHRS algorithm */
extern QueueHandle_t qAHRS;		/* Buffers processed AHRS data for the PID controller */
extern QueueHandle_t qPID;		/* Buffers PID output data for the Motor controller */

extern QueueHandle_t qCommandBuffer;	/* Handles external serial command inputs */

/*----------------------------------
* Semaphores and Mutexes
*----------------------------------*/
extern SemaphoreHandle_t ahrsBufferMutex; /* Protects data passing between AHRS and PID loops when
											 running at differing frequencies */

extern SemaphoreHandle_t pidBufferMutex;  /* Protects data passing between PID and Motor control loops */

enum TaskIndex
{
	AHRS_TASK,
	SENSOR_TASK,
	MOTOR_TASK,
	SDCARD_TASK,
	CONSOLE_TASK,
	LED_STATUS_TASK,
	RADIO_TASK,
	BLUETOOTH_TASK,
	PID_TASK,
	
	TOTAL_TASK_SIZE
};
extern boost::container::vector<TaskHandle_t> TaskHandle;

extern TaskIndex activeTask; /* A simple debugging flag to check which task is currently running */

/* Allows sending a notification message to any task from anywhere */
extern BaseType_t xSendTaskMessage(const TaskIndex, const uint32_t);
extern BaseType_t xSendTaskMessageFromISR(const TaskIndex, const uint32_t);

#endif
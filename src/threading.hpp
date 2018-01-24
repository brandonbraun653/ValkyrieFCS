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

extern QueueHandle_t qIMURawData;		/* Passes raw IMU data to AHRS */

extern QueueHandle_t qCommandBuffer;	/* Handles external serial command inputs */


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
	
	TOTAL_TASK_SIZE
};
extern boost::container::vector<TaskHandle_t> TaskHandle;


/* Allows sending a notification message to any task from anywhere */
extern BaseType_t xSendTaskMessage(const TaskIndex, const uint32_t);
extern BaseType_t xSendTaskMessageFromISR(const TaskIndex, const uint32_t);

#endif
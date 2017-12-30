#pragma once
#ifndef MESSAGING_HPP_
#define MESSAGING_HPP_

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

#include "dataTypes.hpp"

extern QueueHandle_t qIMUFlightData;	/* Handles passing of raw IMU data */
extern QueueHandle_t qCommandBuffer;	/* Handles external serial command inputs */

extern SemaphoreHandle_t sLedCmdTrigger; /* Testing Semphr used for toggling an led based on serial input */

#endif
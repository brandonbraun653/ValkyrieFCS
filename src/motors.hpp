#pragma once
#ifndef MOTORS_HPP_
#define MOTORS_HPP_

/* C/C++ Includes */
#include <stdint.h>
#include <stdlib.h>

/* FreeRTOS Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "dataTypes.hpp"

struct MotorCommands
{
	uint16_t motor1 = 0;
	uint16_t motor2 = 0;
	uint16_t motor3 = 0;
	uint16_t motor4 = 0;
};

extern Radio_Control latestMotorCommand; //not actually radio control type!! hacky fix for bare bones testing!
extern SemaphoreHandle_t mc_mutex;

extern void motorTask(void* argument);


#endif /* MOTORS_HPP_ */
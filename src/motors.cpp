/* C/C++ Includes */
#include <stdint.h>
#include <stdlib.h>

/* Thor Includes */
#include "include/thor.h"
#include "include/gpio.h"
#include "libraries/ESC/esc_quad.hpp"

/* Boost Includes */
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

/* FreeRTOS Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Project Includes */
#include "dataTypes.hpp"
#include "fcsConfig.hpp"
#include "threading.hpp"
#include "motors.hpp"


#define ESC_ARM 800
#define ESC_MIN_THROTTLE 1060
#define ESC_MAX_THROTTLE 1860

const uint16_t ARM_COMMAND = 0xFFFF;

PIDData_t pidInput;
MotorState MotorControllerState = MOTOR_STATE_OFF;


void motorTask(void* argument)
{	
	ESCQuad_Init initStruct;
	initStruct.armCMD_uS = ESC_ARM;
	initStruct.minThrottleCMD_uS = ESC_MIN_THROTTLE;
	initStruct.maxThrottleCMD_uS = ESC_MAX_THROTTLE;
	initStruct.outputFrequency = 55.0;
	
	initStruct.motor1 = boost::make_shared<GPIOClass>(GPIOB, PIN_6, ULTRA_SPD, GPIO_AF2_TIM4);
	initStruct.motor1_timer_channel = TIM_CHANNEL_1;

	initStruct.motor2 = boost::make_shared<GPIOClass>(GPIOB, PIN_7, ULTRA_SPD, GPIO_AF2_TIM4);
	initStruct.motor2_timer_channel = TIM_CHANNEL_2;

	initStruct.motor3 = boost::make_shared<GPIOClass>(GPIOB, PIN_8, ULTRA_SPD, GPIO_AF2_TIM4);
	initStruct.motor3_timer_channel = TIM_CHANNEL_3;

	initStruct.motor4 = boost::make_shared<GPIOClass>(GPIOB, PIN_9, ULTRA_SPD, GPIO_AF2_TIM4);
	initStruct.motor4_timer_channel = TIM_CHANNEL_4;
	

	
	ESCQuad motorController;
	motorController.initialize(initStruct);
	
	
	
	TickType_t lastTimeWoken = xTaskGetTickCount();
	for (;;)
	{
		/* Block indefinitely until a new PID command has been issued */
		xQueueReceive(qPID, &pidInput, portMAX_DELAY);

		if (MotorControllerState == MOTOR_STATE_ARM)
		{
			if (!motorController.isArmed())
				motorController.arm();
			
			//blah blah blah continue on as usual here
			
		}

	}
}
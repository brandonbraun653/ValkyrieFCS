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

/* Project Includes */
#include "fcsConfig.hpp"
#include "motors.hpp"
#include "radio.hpp"	//TODO: remove this after initial testing of motors!



const int updateRate_mS = (1.0 / MOTOR_UPDATE_FREQ_HZ) * 1000.0;
const uint16_t ARM_COMMAND = 0xFFFF;

Radio_Control latestMotorCommand;
SemaphoreHandle_t mc_mutex = xSemaphoreCreateMutex();

void motorTask(void* argument)
{
	GPIOClass_sPtr m1_pin = boost::make_shared<GPIOClass>(GPIOB, PIN_6, ULTRA_SPD, GPIO_AF2_TIM4);
	GPIOClass_sPtr m2_pin = boost::make_shared<GPIOClass>(GPIOB, PIN_7, ULTRA_SPD, GPIO_AF2_TIM4);
	GPIOClass_sPtr m3_pin = boost::make_shared<GPIOClass>(GPIOB, PIN_8, ULTRA_SPD, GPIO_AF2_TIM4);
	GPIOClass_sPtr m4_pin = boost::make_shared<GPIOClass>(GPIOB, PIN_9, ULTRA_SPD, GPIO_AF2_TIM4);
	
	ESCQuad_Init initStruct;
	initStruct.minInput = 0;
	initStruct.maxInput = 2048;
	initStruct.outputFrequency = 55.0;
	
	initStruct.motor1 = m1_pin; initStruct.motor1_timer_channel = TIM_CHANNEL_1;
	initStruct.motor2 = m2_pin; initStruct.motor2_timer_channel = TIM_CHANNEL_2;
	initStruct.motor3 = m3_pin; initStruct.motor3_timer_channel = TIM_CHANNEL_3;
	initStruct.motor4 = m4_pin; initStruct.motor4_timer_channel = TIM_CHANNEL_4;
	
	MotorCommands motorCMD;
	motorCMD.motor1 = ARM_COMMAND;
	motorCMD.motor2 = ARM_COMMAND;
	motorCMD.motor3 = ARM_COMMAND;
	motorCMD.motor4 = ARM_COMMAND;
	
	ESCQuad motorController;
	motorController.initialize(initStruct);
	
	motorController.enable();
	motorController.updateSetPoint(motorCMD.motor1, motorCMD.motor2, motorCMD.motor3, motorCMD.motor4);
	
	TickType_t lastTimeWoken = xTaskGetTickCount();
	vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(5000));
	for (;;)
	{
		if (xQueueReceive(qRadio_Control, &latestMotorCommand, 0) == pdPASS)
		{
			motorCMD.motor1 = (uint16_t)(latestMotorCommand.THROTTLE * 300.0);
			motorCMD.motor2 = motorCMD.motor1;
			motorCMD.motor3 = motorCMD.motor1;
			motorCMD.motor4 = motorCMD.motor1;
		}
		
		motorController.updateSetPoint(motorCMD.motor1, motorCMD.motor2, motorCMD.motor3, motorCMD.motor4);
		
		vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(updateRate_mS));
	}
}
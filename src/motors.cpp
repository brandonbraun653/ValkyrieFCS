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


/*-------------------------------------------------
 *				Motor Orientation
 * Motor 1: TOP RIGHT		-- CCW
 * Motor 2: BOTTOM LEFT		-- CCW
 * Motor 3: TOP LEFT		-- CW
 * Motor 4: BOTTOM RIGHT	-- CW
 * 
 *
 *			M1  M2  M3  M4
 * +Pitch:	+	-	+	-
 * -Pitch:	-	+	-	+
 * +Roll:	-	+	+	-
 * -Roll:	+	-	-	+
 * 
 **/


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
	initStruct.outputFrequency = 450.0;
	
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
	//motorController.throttleCalibration();
	
	motorController.arm();
	
	//const float throttleRange = ESC_MAX_THROTTLE - ESC_MIN_THROTTLE;
	const float throttleRange = 265;
	
	int dPitch = 0;
	int dRoll = 0;
	int dYaw = 0;
	
	int baseThrottle = ESC_MIN_THROTTLE + 25;
	
	uint16_t m1 = baseThrottle;
	uint16_t m2 = baseThrottle;
	uint16_t m3 = baseThrottle;
	uint16_t m4 = baseThrottle;
	
	TickType_t lastTimeWoken = xTaskGetTickCount();
	for (;;)
	{
		/* Block indefinitely until a new PID command has been issued */
		xQueueReceive(qPID, &pidInput, portMAX_DELAY);
		
		/* There is a chance a lot of packets were missed, so make sure the latest is received.
		 * TODO: In the pid loop, Change the queue size to 1 and always overwrite...*/
		if (uxQueueMessagesWaiting(qPID) > 0)
		{
			while (uxQueueMessagesWaiting(qPID) > 0){ xQueueReceive(qPID, &pidInput, 0); }
		}

		/*-------------------------------------
		 * Command Input Processing 
		 *------------------------------------*/
		dPitch = (int)(pidInput.pitchControl);
		dRoll = (int)(pidInput.rollControl);
		dYaw = (int)(pidInput.yawControl);
		
		m1 = (uint16_t)(baseThrottle - dPitch + dRoll);
		m2 = (uint16_t)(baseThrottle + dPitch - dRoll);
		m3 = (uint16_t)(baseThrottle - dPitch - dRoll);
		m4 = (uint16_t)(baseThrottle + dPitch + dRoll);
		
		m1 = motorController.limit(m1);
		m2 = motorController.limit(m2);
		m3 = motorController.limit(m3);
		m4 = motorController.limit(m4);
		
		/*-------------------------------------
		 * Write Motors
		 *------------------------------------*/
		motorController.updateSetPoint(m1, m2, m3, m4);
	}
}
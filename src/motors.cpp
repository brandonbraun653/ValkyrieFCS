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

PIDData_t pidInput;				/* Input struct for receiving pid commands */
SDLOG_Motors_t sd_motorLog;		/* Output struct for logging motor commands */


MotorState MotorControllerState = MOTOR_STATE_OFF;

const int updateRate_mS = (1.0f / PID_UPDATE_FREQ_HZ) * 1000.0f;


/* Low pass filtering constants for pitch cmd input */
float dt = (1.0f / PID_UPDATE_FREQ_HZ);
float tau = 0.01f;
float alpha_lp = dt / tau;

void motorTask(void* argument)
{	
	ESCQuad_Init initStruct;
	initStruct.armCMD_uS = ESC_ARM;
	initStruct.minThrottleCMD_uS = ESC_MIN_THROTTLE;
	initStruct.maxThrottleCMD_uS = 1650;
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
	
	/* Arm the motors and inform the PID loop to start sending data */
	motorController.arm();
	xTaskSendMessage(PID_TASK, PID_ENABLE);
	xTaskSendMessage(SDCARD_TASK, SD_CARD_ENABLE_IO);
	
	
	int baseThrottle = initStruct.minThrottleCMD_uS + 100;
	
	uint16_t m1 = baseThrottle;
	uint16_t m2 = baseThrottle;
	uint16_t m3 = baseThrottle;
	uint16_t m4 = baseThrottle;
	
	Eigen::Matrix<float, 4, 1> dummyVec1, dummyVec2, commandInputRaw, commandInputFiltered, motorOutput;
	Eigen::Matrix4f motorMix;
	
	motorMix << 
		1.0f,  1.0f,  1.0f, 0.0f,	//M1
		1.0f, -1.0f, -1.0f, 0.0f,	//M2
		1.0f,  1.0f, -1.0f, 0.0f,	//M3
		1.0f, -1.0f,  1.0f, 0.0f;	//M4
	
	
	TickType_t lastTimeWoken = xTaskGetTickCount();
	for (;;)
	{
		activeTask = MOTOR_TASK;
		
		/* Grab the latest PID command outputs */
		if (xSemaphoreTake(pidBufferMutex, 0) == pdPASS)
		{
			xQueueReceive(qPID, &pidInput, 0);
			xSemaphoreGive(pidBufferMutex);
			
			/* Low pass filter the input */
			commandInputRaw << baseThrottle, pidInput.pitchControl, pidInput.rollControl, pidInput.yawControl;
			commandInputFiltered += alpha_lp * (commandInputRaw - commandInputFiltered);
		}
		
	
		/*-------------------------------------
		 * Things
		 *------------------------------------*/
		motorOutput = motorMix*commandInputFiltered;
		
		m1 = motorController.limit((uint16_t)motorOutput(0));
		m2 = motorController.limit((uint16_t)motorOutput(1));
		m3 = motorController.limit((uint16_t)motorOutput(2));
		m4 = motorController.limit((uint16_t)motorOutput(3));
		
		/*-------------------------------------
		 * Write Motors
		 *------------------------------------*/
		motorController.updateSetPoint(m1, m2, m3, m4);
		

		/*----------------------------
		* Data Post Processing
		*---------------------------*/
		sd_motorLog.tickTime = (uint32_t)xTaskGetTickCount();
		sd_motorLog.m1 = m1;
		sd_motorLog.m2 = m2;
		sd_motorLog.m3 = m3;
		sd_motorLog.m4 = m4;

		xQueueSendToBack(qSD_Motor, &sd_motorLog, 0);

		vTaskDelayUntil(&lastTimeWoken, updateRate_mS);
	}
}
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

namespace FCS_MOTOR
{
	const uint16_t ARM_COMMAND = 0xFFFF;
	const int updateRate_mS = (1.0f / CTRL_UPDATE_FREQ_HZ) * 1000.0f;
	const int ESC_ARM = 800;
	
	/* Low pass filtering constants for pitch cmd input */
	const float dt = (1.0f / CTRL_UPDATE_FREQ_HZ);
	const float tau = 0.01f;
	const float alpha_lp = dt / tau;

	void motorTask(void* argument)
	{	
		#ifdef DEBUG
		volatile UBaseType_t stackHighWaterMark_MOTOR = 0;
		volatile float pitch_cmd = 0.0;
		volatile float roll_cmd = 0.0;
		volatile uint16_t m1_cmd = 0;
		volatile uint16_t m2_cmd = 0;
		volatile uint16_t m3_cmd = 0;
		volatile uint16_t m4_cmd = 0;
		#endif
	
		ESCQuad_Init initStruct;
		initStruct.armCMD_uS = ESC_ARM;
		initStruct.minThrottleCMD_uS = ESC_MIN_THROTTLE;
		initStruct.maxThrottleCMD_uS = 1450;
		initStruct.outputFrequency = 450.0;
	
		initStruct.motor1 = boost::make_shared<GPIOClass>(GPIOB, PIN_6, ULTRA_SPD, GPIO_AF2_TIM4);
		initStruct.motor1_timer_channel = TIM_CHANNEL_1;

		initStruct.motor2 = boost::make_shared<GPIOClass>(GPIOB, PIN_7, ULTRA_SPD, GPIO_AF2_TIM4);
		initStruct.motor2_timer_channel = TIM_CHANNEL_2;

		initStruct.motor3 = boost::make_shared<GPIOClass>(GPIOB, PIN_8, ULTRA_SPD, GPIO_AF2_TIM4);
		initStruct.motor3_timer_channel = TIM_CHANNEL_3;

		initStruct.motor4 = boost::make_shared<GPIOClass>(GPIOB, PIN_9, ULTRA_SPD, GPIO_AF2_TIM4);
		initStruct.motor4_timer_channel = TIM_CHANNEL_4;
	
		uint16_t baseThrottle = initStruct.minThrottleCMD_uS + 100;
	
		uint16_t m1 = baseThrottle;
		uint16_t m2 = baseThrottle;
		uint16_t m3 = baseThrottle;
		uint16_t m4 = baseThrottle;
	
		Eigen::Matrix<float, 4, 1> commandInputRaw, commandInputFiltered, motorOutput;
		Eigen::Matrix4f motorMix;
	
		motorMix << 
			1.0f, 1.0f, 1.0f, 0.0f, 	//M1
			1.0f, -1.0f, -1.0f, 0.0f, 	//M2
			1.0f, 1.0f, -1.0f, 0.0f, 	//M3
			1.0f, -1.0f, 1.0f, 0.0f; 	//M4
	
		ESCQuad motorController;
		motorController.initialize(initStruct);
		motorController.arm();
		//motorController.throttleCalibration();
	
		/* Inform the controller loop and SD card logger to start doing their thing */
		xTaskSendMessage(SDCARD_TASK, SD_CARD_ENABLE_IO);
		xTaskSendMessage(CTRL_TASK, CTRL_ENABLE);
		
	
		PIDData_t pidInput;
		LQRData_t lqrInput;
		SDLOG_Motors_t sd_motorLog;
	
		
		/* Tell init task that this thread's initialization is done and ok to run.
		 * Wait for init task to resume operation. */
		xTaskSendMessage(INIT_TASK, 1u);
		vTaskSuspend(NULL);
		taskYIELD();
		
		TickType_t lastTimeWoken = xTaskGetTickCount();
		for (;;)
		{
			#ifdef DEBUG
			activeTask = MOTOR_TASK;
			stackHighWaterMark_MOTOR = uxTaskGetStackHighWaterMark(NULL);
			#endif
		
			/* Grab the latest CTRL command outputs */
			#if USING_PID_CTRL
			if (xSemaphoreTake(pidBufferMutex, 0) == pdPASS)
			{
				xQueueReceive(qPID, &pidInput, 0);
				xSemaphoreGive(pidBufferMutex);
			
				/* Low pass filter the input */
				commandInputRaw << (float)baseThrottle, pidInput.pitchControl, pidInput.rollControl, pidInput.yawControl;
				commandInputFiltered += alpha_lp * (commandInputRaw - commandInputFiltered);
			}
			motorOutput = motorMix*commandInputFiltered;
			#endif
			
			#if USING_LQR_CONTROL
			if (xSemaphoreTake(lqrBufferMutex, 0) == pdPASS)
			{
				xQueueReceive(qLQR, &lqrInput, 0);
				xSemaphoreGive(lqrBufferMutex);	
			}
			motorOutput << lqrInput.m1, lqrInput.m2, lqrInput.m3, lqrInput.m4;
			#endif 

			/*-------------------------------------
			 * Write Motors
			 *------------------------------------*/
			m1 = motorController.limit((uint16_t)motorOutput(0));
			m2 = motorController.limit((uint16_t)motorOutput(1));
			m3 = motorController.limit((uint16_t)motorOutput(2));
			m4 = motorController.limit((uint16_t)motorOutput(3));
			
			motorController.updateSetPoint(m1, m2, m3, m4);
			
//			uint16_t thr = ESC_MIN_THROTTLE + 150;
//			motorController.updateSetPoint(thr, thr, thr, thr);
		
			#ifdef DEBUG
			pitch_cmd = commandInputFiltered(1);
			roll_cmd = commandInputFiltered(2);
			m1_cmd = m1;
			m2_cmd = m2;
			m3_cmd = m3;
			m4_cmd = m4;
			#endif

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
}


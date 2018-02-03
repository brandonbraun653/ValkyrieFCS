/* C/C++ Includes */


/* FreeRTOS Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue"

/* Library Includes */
#include "MiniPID.h"

/* Project Includes */
#include "fcsConfig.hpp"
#include "threading.hpp"
#include "pid.hpp"
#include "radio.hpp"

const int updateRate_mS = (1.0 / PID_UPDATE_FREQ_HZ) * 1000.0;

AHRSDataDeg_t ahrs;	/* Input data from the AHRS algorithm */
PIDData_t pidCMD;	/* Output data from this thread */

//TODO: Replace these with variables from the motor esc class...
const float minPWR = 1060.0f;
const float maxPWR = 1860.0f;
const float outRng = maxPWR - minPWR;

const float rollAngleSetPoint = 0.0;	//deg
const float pitchAngleSetPoint = 0.0;	//deg
const float yawAngleSetPoint = 0.0;		//deg => eventually put this in the Radio task (process signals)

#define KP 1.0f
#define KI 1.0f
#define KD 1.0f

#define PID_OUTPUT_RANGE 256.0f

void pidTask(void* argument)
{

	/* Ensure we don't send the PID controllers ridiculous values upon init */
	ahrs.pitch = 0.0;
	ahrs.roll = 0.0;
	ahrs.yaw = 0.0;

	MiniPID pitchAngleController	= MiniPID(KP, KI, KD);
	MiniPID rollAngleController		= MiniPID(KP, KI, KD);
	MiniPID yawAngleController		= MiniPID(KP, KI, KD);
	
	/* Operate as normalized controllers for numerical stability */
	pitchAngleController.setOutputLimits(-PID_OUTPUT_RANGE, PID_OUTPUT_RANGE);
	rollAngleController.setOutputLimits(-PID_OUTPUT_RANGE, PID_OUTPUT_RANGE);
	yawAngleController.setOutputLimits(-PID_OUTPUT_RANGE, PID_OUTPUT_RANGE);
	
	//pitchAngleController.setOutputRampRate(0.1f);
	//rollAngleController.setOutputRampRate(0.1f);
	
	pitchAngleController.setDirection(true); //Negative fb
	rollAngleController.setDirection(true); //Negative fb

	TickType_t lastTimeWoken = xTaskGetTickCount();
	for (;;)
	{
		/* Check for an update from the AHRS thread */
		xSemaphoreTake(ahrsBufferMutex, 0);
		while (uxQueueMessagesWaiting(qAHRS) > 0){ xQueueReceive(qAHRS, &ahrs, 0); }
		xSemaphoreGive(ahrsBufferMutex);

		//TODO:
		/* Check for an update from the radio for new set points */


		pidCMD.pitchControl = pitchAngleController.getOutput(ahrs.pitch, pitchAngleSetPoint);
		pidCMD.rollControl = rollAngleController.getOutput(ahrs.roll, rollAngleSetPoint);
		pidCMD.yawControl = yawAngleController.getOutput(ahrs.yaw, yawAngleSetPoint);


		/* Send data over to the motor controller thread without waiting for confirmation of success */
		xQueueSendToBack(qPID, &pidCMD, 0);
		vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(updateRate_mS));
	}

}
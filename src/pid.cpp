/* C/C++ Includes */


/* FreeRTOS Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue"

/* Library Includes */
#include "MiniPID.h"
#include "PID_v1.h"

/* Project Includes */
#include "fcsConfig.hpp"
#include "threading.hpp"
#include "pid.hpp"
#include "radio.hpp"

const int updateRate_mS = (1.0f / PID_UPDATE_FREQ_HZ) * 1000.0f;

AHRSDataDeg_t ahrs;	/* Input data from the AHRS algorithm */
PIDData_t pidCMD;	/* Output data from this thread */

//TODO: Replace these with variables from the motor esc class...
const float minPWR = 1060.0f;
const float maxPWR = 1860.0f;
const float outRng = maxPWR - minPWR;

float rollAngleSetPoint = 0.0;	//deg
float pitchAngleSetPoint = 0.0;	//deg
float yawAngleSetPoint = 0.0;	//deg => eventually put this in the Radio task (process signals)

float pitchRateSetPoint = 0.0;

#define ANGLE_KP 1.0f
#define ANGLE_KI 0.0f
#define ANGLE_KD 0.0f

#define RATE_KP 15.0f
#define RATE_KI 0.0f
#define RATE_KD 0.1f

#define MOTOR_OUTPUT_RANGE 256.0f
#define GYRO_SENSITIVITY 2000 //dps

void pidTask(void* argument)
{	
	float pAngIn = 0.0;			//Current pitch angle from AHRS (deg)
	
	
	

	
	float dP = 0.0;				//Current pitch angle rate of change from gyro (dps)
	float dR = 0.0;
	
	float rRateDesired = 0.0;
	float pRateDesired = 0.0; 	//Desired pitch angle to achieve (deg)
	
	float pMotorCmd = 0.0; 		//Output motor command signal delta
	float rMotorCmd = 0.0;
	
	/* Input: Current angle as measured from AHRS in degrees
	 * Output: Desired rotation rate in dps*/
//	PID pAngCtrl(&pAngIn, &pRateDesired, &pitchAngleSetPoint, ANGLE_KP, ANGLE_KI, ANGLE_KD, P_ON_E, REVERSE);
//	
//	pAngCtrl.SetMode(0);
//	pAngCtrl.SetOutputLimits(-GYRO_SENSITIVITY, GYRO_SENSITIVITY);
//	pAngCtrl.SetSampleTime(updateRate_mS);
	
	/* Input: Desired rotation rate in dps from pitch angle controller
	 * Output: Motor command delta */
	PID pRateCtrl(&dP, &pMotorCmd, &pRateDesired, RATE_KP, RATE_KI, RATE_KD, P_ON_E, REVERSE);
	
	pRateCtrl.SetMode(0);
	pRateCtrl.SetOutputLimits(-MOTOR_OUTPUT_RANGE, MOTOR_OUTPUT_RANGE);
	pRateCtrl.SetSampleTime(updateRate_mS);
	
	PID rRateCtrl(&dP, &rMotorCmd, &rRateDesired, RATE_KP, RATE_KI, RATE_KD, P_ON_E, REVERSE);
	
	rRateCtrl.SetMode(0);
	rRateCtrl.SetOutputLimits(-MOTOR_OUTPUT_RANGE, MOTOR_OUTPUT_RANGE);
	rRateCtrl.SetSampleTime(updateRate_mS);
	
	TickType_t lastTimeWoken = xTaskGetTickCount();
	for (;;)
	{
		activeTask = PID_TASK;

		/* Check for an update from the AHRS thread. This will always pull the latest information. */
		if (xSemaphoreTake(ahrsBufferMutex, 0) == pdPASS)
		{
			xQueueReceive(qAHRS, &ahrs, 0);
			xSemaphoreGive(ahrsBufferMutex);
		}
		
		//TODO:
		/* Check for an update from the radio for new set points */

		
		/* Calculate the desired rotation rate */
		pRateDesired = pitchRateSetPoint;
		dP = ahrs.gyro(0);
		dR = ahrs.gyro(1);
		
		pRateCtrl.Compute();
		rRateCtrl.Compute();
		
		pidCMD.pitchControl = pMotorCmd;
		pidCMD.rollControl = rMotorCmd;
		
		/* Send data over to the motor controller thread without waiting for confirmation of success */
		xQueueSendToBack(qPID, &pidCMD, 0);
		vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(updateRate_mS));
	}

}
/* C/C++ Includes */


/* FreeRTOS Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue"

/* Library Includes */
#include "PID_v1.h"

/* Project Includes */
#include "fcsConfig.hpp"
#include "threading.hpp"
#include "pid.hpp"
#include "radio.hpp"

const int updateRate_mS = (1.0f / PID_UPDATE_FREQ_HZ) * 1000.0f;

AHRSData_t ahrs;	/* Input data from the AHRS algorithm */
PIDData_t pidCMD;	/* Output data from this thread */

//TODO: Replace these with variables from the motor esc class...
const float minPWR = 1060.0f;
const float maxPWR = 1860.0f;
const float outRng = maxPWR - minPWR;

float rollAngleSetPoint = 0.0;	//deg
float pitchAngleSetPoint = 0.0;	//deg
float yawAngleSetPoint = 0.0;	//deg => eventually put this in the Radio task (process signals)

float pitchRateSetPoint = 0.0;
float rollRateSetPoint = 0.0;

bool pidEnabled = false;

#define ANGLE_KP 2.5f
#define ANGLE_KI 3.0f
#define ANGLE_KD 0.01f

#define RATE_KP 0.9f
#define RATE_KI 4.0f
#define RATE_KD 0.05f

#define MOTOR_OUTPUT_RANGE 500.0f
#define GYRO_SENSITIVITY 2000 //dps


namespace FCSPID
{
	void parseTaskNotification(uint32_t notification)
	{
		if (notification == PID_ENABLE)
			pidEnabled = true;

		if (notification == PID_DISABLE)
			pidEnabled = false;
	}

	void pidTask(void* argument)
	{
		SDLOG_PIDAngleInput_t angleControllerLog;
		SDLOG_PIDRateInput_t rateControllerLog;
		
		
		float pAngIn = 0.0;			//Current pitch angle from AHRS (deg)
		float rAngIn = 0.0;

		float dP = 0.0;				//Current pitch angle rate of change from gyro (dps)
		float dR = 0.0;

		float rRateDesired = 0.0;	//Desired pitch angle to achieve (deg)
		float pRateDesired = 0.0;

		float pMotorCmd = 0.0; 		//Output motor command signal delta
		float rMotorCmd = 0.0;


		/*-------------------------------------------------------
		* ANGLE PID CONTROLLER
		* Input: Current angle from the AHRS algorithm
		* Output: Rotation Rate
		*
		*
		*------------------------------------------------------*/
		PID pAngCtrl(&pAngIn, &pRateDesired, &pitchAngleSetPoint, ANGLE_KP, ANGLE_KI, ANGLE_KD, P_ON_E, REVERSE);
		PID rAngCtrl(&rAngIn, &rRateDesired, &rollAngleSetPoint, ANGLE_KP, ANGLE_KI, ANGLE_KD, P_ON_E, DIRECT);

		pAngCtrl.SetMode(0);
		pAngCtrl.SetOutputLimits(-100.0f, 100.0f);
		pAngCtrl.SetSampleTime(updateRate_mS);

		rAngCtrl.SetMode(0);
		rAngCtrl.SetOutputLimits(-100.0f, 100.0f);
		rAngCtrl.SetSampleTime(updateRate_mS);


		/*-------------------------------------------------------
		* RATE PID CONTROLLER
		* Input: Desired rotation rate in dps from pitch angle controller
		* Output: Motor command delta
		*
		* Positive error signal yields negative output when in DIRECT mode
		*------------------------------------------------------*/
		PID pRateCtrl(&dP, &pMotorCmd, &pRateDesired, RATE_KP, RATE_KI, RATE_KD, P_ON_E, DIRECT);
		PID rRateCtrl(&dR, &rMotorCmd, &rRateDesired, RATE_KP, RATE_KI, RATE_KD, P_ON_E, DIRECT);


		pRateCtrl.SetMode(0);
		pRateCtrl.SetOutputLimits(-MOTOR_OUTPUT_RANGE, MOTOR_OUTPUT_RANGE);
		pRateCtrl.SetSampleTime(updateRate_mS);

		rRateCtrl.SetMode(0);
		rRateCtrl.SetOutputLimits(-MOTOR_OUTPUT_RANGE, MOTOR_OUTPUT_RANGE);
		rRateCtrl.SetSampleTime(updateRate_mS);

		TickType_t lastTimeWoken = xTaskGetTickCount();
		for (;;)
		{
			activeTask = PID_TASK;
			parseTaskNotification(ulTaskNotifyTake(pdTRUE, 0));

			if (pidEnabled)
			{
				/* Check for an update from the AHRS thread. This will always pull the latest information. */
				if (xSemaphoreTake(ahrsBufferMutex, 0) == pdPASS)
				{
					xQueueReceive(qAHRS, &ahrs, 0);
					xSemaphoreGive(ahrsBufferMutex);
				}

				//TODO:
				/* Check for an update from the radio for new set points */

				/*--------------------------------------
				* Calculate the desired rotation rate
				*--------------------------------------*/
				//Update the inputs
				pAngIn = ahrs.eulerAngles(0);
				rAngIn = ahrs.eulerAngles(1);

				pAngCtrl.Compute();
				rAngCtrl.Compute();

				#ifdef DEBUG
				float pRateOutput = pRateDesired;
				float rRateOutput = rRateDesired;
				#endif

				/*--------------------------------------
				* Calculate the motor command outputs
				*--------------------------------------*/
				dP = ahrs.gyro(1);	/* Pitch Rotation Axis: Sensor Y-Axis
									* Increasing Body Pitch Angle ==> ++Pitch Rate (CW Rotation About Sensor Y)*/

				dR = -ahrs.gyro(0); /* Roll Rotation Axis: Sensor X-Axis
									* Increasing Body Roll Angle ==> --Roll Rate (CCW Rotation About Sensor X)
									*
									* FIX: Multiply by -1 so that increasing roll angle yields increasing rotation rate */

				pRateCtrl.Compute();
				rRateCtrl.Compute();

				pidCMD.pitchControl = pMotorCmd;
				pidCMD.rollControl = rMotorCmd;

				/* Send data over to the motor controller thread without waiting for confirmation of success */
				xSemaphoreTake(pidBufferMutex, 2);
				xQueueOverwrite(qPID, &pidCMD);
				xSemaphoreGive(pidBufferMutex);
				
				
				/* Push log data to the SDCard task */
				angleControllerLog.tickTime = (uint32_t)xTaskGetTickCount();
				angleControllerLog.pitch_angle_setpoint = (uint8_t)pitchAngleSetPoint;
				angleControllerLog.roll_angle_setpoint = (uint8_t)rollAngleSetPoint;
				angleControllerLog.yaw_angle_setpoint = (uint8_t)0;
				xQueueSendToBack(qSD_PIDAngleSetpoint, &angleControllerLog, 0);
				
				rateControllerLog.tickTime = (uint32_t)xTaskGetTickCount();
				rateControllerLog.pitch_rate_setpoint = (uint16_t)pRateDesired;
				rateControllerLog.roll_rate_setpoint = (uint16_t)rRateDesired;
				rateControllerLog.yaw_rate_setpoint = (uint16_t)0;
				xQueueSendToBack(qSD_PIDRateSetpoint, &rateControllerLog, 0);
			}

			vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(updateRate_mS));
		}


	}
}


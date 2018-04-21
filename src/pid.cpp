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

bool pidEnabled = false;

#define MOTOR_OUTPUT_RANGE 500.0f
#define GYRO_SENSITIVITY 2000 //dps


namespace FCS_PID
{
	/* PID SETTINGS */
	const float ANGLE_KP_PITCH = 1.5f;
	const float ANGLE_KI_PITCH = 3.0f;
	const float ANGLE_KD_PITCH = 0.01f;

	const float RATE_KP_PITCH = 0.9f;
	const float RATE_KI_PITCH = 4.0f; 
	const float RATE_KD_PITCH = 0.05f;

	const float ANGLE_KP_ROLL = 1.5f;
	const float ANGLE_KI_ROLL = 3.0f;
	const float ANGLE_KD_ROLL = 0.01f;

	const float RATE_KP_ROLL = 0.9f;
	const float RATE_KI_ROLL = 4.0f;
	const float RATE_KD_ROLL = 0.05f;


	void parseTaskNotification(uint32_t notification)
	{
		if (notification == PID_ENABLE)
			pidEnabled = true;

		if (notification == PID_DISABLE)
			pidEnabled = false;
	}

	void pidTask(void* argument)
	{
		#ifdef DEBUG
		volatile UBaseType_t stackHighWaterMark_PID = 0;
		#endif
		
		/* Uncomment these for 5 degree stepping: UNCOUPLED */
// 		const float rollStep[] = {	
// 			0.0, 5.0, 10.0, 15.0, 20.0, 15.0, 10.0, 5.0, 0.0, -5.0, -10.0, -15.0, -20.0, -15.0, -10.0, -5.0, 0.0,  
// 			0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0, 0.0, 0.0,  0.0,   0.0,   0.0,   0.0,   0.0,   0.0,  0.0, 0.0 };
// 
// 		const float pitchStep[] = { 
// 			0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0, 0.0, 0.0,  0.0,   0.0,   0.0,   0.0,   0.0,   0.0,  0.0, 0.0,
// 			0.0, 5.0, 10.0, 15.0, 20.0, 15.0, 10.0, 5.0, 0.0, -5.0, -10.0, -15.0, -20.0, -15.0, -10.0, -5.0, 0.0 };

		/* Uncomment these for 10 degree stepping: UNCOUPLED */
		const float rollStep[] =  { 0.0, 10.0, 0.0, 10.0, 0.0, -10.0, 0.0, -10.0, 0.0,
									0.0,  0.0, 0.0,  0.0, 0.0,   0.0, 0.0,   0.0, 0.0 };

		const float pitchStep[] = { 0.0,  0.0, 0.0,  0.0, 0.0,   0.0, 0.0,   0.0, 0.0,
									0.0, 10.0, 0.0, 10.0, 0.0, -10.0, 0.0, -10.0, 0.0 };

		/* Uncomment these for 15 degree stepping: UNCOUPLED */
// 		const float rollStep[]  = { 0.0, 15.0, 0.0, 15.0, 0.0, -15.0, 0.0, -15.0, 0.0,  0.0,   0.0,   0.0, 0.0,   0.0,   0.0,   0.0, 0.0 };
// 		const float pitchStep[] = { 0.0,  0.0, 0.0,  0.0, 0.0,   0.0, 0.0,   0.0, 0.0, 15.0,   0.0,  15.0, 0.0, -15.0,   0.0, -15.0, 0.0 };

		/* Uncomment these for 5 degree stepping: COUPLED */
// 		const float rollStep[]  = { 0.0, 5.0, 10.0, 15.0, 10.0, 5.0, 0.0, -5.0, -10.0, -15.0, -10.0, -5.0, 0.0 };
// 		const float pitchStep[] = { 0.0, 5.0, 10.0, 15.0, 10.0, 5.0, 0.0, -5.0, -10.0, -15.0, -10.0, -5.0, 0.0 };

		/* Uncomment these for 10 degree stepping: COUPLED */
// 		const float rollStep[]  = { 0.0, 10.0, 0.0, 10.0, 0.0, -10.0, 0.0, -10.0, 0.0 };
// 		const float pitchStep[] = { 0.0, 10.0, 0.0, 10.0, 0.0, -10.0, 0.0, -10.0, 0.0 };

		/* Uncomment these for 15 degree stepping: COUPLED */
// 		const float rollStep[]  = { 0.0, 15.0, 0.0, 15.0, 0.0, -15.0, 0.0, -15.0, 0.0 };
// 		const float pitchStep[] = { 0.0, 15.0, 0.0, 15.0, 0.0, -15.0, 0.0, -15.0, 0.0 };

		/* Uncomment these for manual poking (super scientific, I know) */
// 		const float rollStep[] =  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
// 		const float pitchStep[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };


		const int numSteps = sizeof(rollStep) / sizeof(rollStep[0]);
		uint32_t lastTime = 0;
		int currentStep = 0;
		bool steppingEnabled = true;
		bool signalRecordingStop = false;
		int STEP_DELAY_TIME_MS = 1000;


		SDLOG_PIDAngleInput_t angleControllerLog;
		SDLOG_PIDRateInput_t rateControllerLog;
		
		
		float pAngleDesired = 0.0;	//Desired pitch angle to achieve (deg)
		float rAngleDesired = 0.0;

		float pRateDesired = 0.0;	//Desired pitch rotation rate to achieve (deg)
		float rRateDesired = 0.0;

		float pAngFB = 0.0;			//Current pitch angle from AHRS (deg)
		float rAngFB = 0.0;

		float pRateFB = 0.0;		//Current pitch angle rate of change from gyro (dps)
		float rRateFB = 0.0;

		float pMotorCmd = 0.0; 		//Output motor command signal delta
		float rMotorCmd = 0.0;
		

		/*-------------------------------------------------------
		* ANGLE PID CONTROLLER
		* Input: Current angle from the AHRS algorithm
		* Output: Rotation Rate
		*
		*
		*------------------------------------------------------*/
		PID pAngCtrl(&pAngFB, &pRateDesired, &pAngleDesired, ANGLE_KP_PITCH, ANGLE_KI_PITCH, ANGLE_KD_PITCH, P_ON_E, REVERSE);
		PID rAngCtrl(&rAngFB, &rRateDesired, &rAngleDesired, ANGLE_KP_ROLL, ANGLE_KI_ROLL, ANGLE_KD_ROLL, P_ON_E, DIRECT);

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
		PID pRateCtrl(&pRateFB, &pMotorCmd, &pRateDesired, RATE_KP_PITCH, RATE_KI_PITCH, RATE_KD_PITCH, P_ON_E, DIRECT);
		PID rRateCtrl(&rRateFB, &rMotorCmd, &rRateDesired, RATE_KP_ROLL, RATE_KI_ROLL, RATE_KD_ROLL, P_ON_E, DIRECT);


		pRateCtrl.SetMode(0);
		pRateCtrl.SetOutputLimits(-MOTOR_OUTPUT_RANGE, MOTOR_OUTPUT_RANGE);
		pRateCtrl.SetSampleTime(updateRate_mS);

		rRateCtrl.SetMode(0);
		rRateCtrl.SetOutputLimits(-MOTOR_OUTPUT_RANGE, MOTOR_OUTPUT_RANGE);
		rRateCtrl.SetSampleTime(updateRate_mS);
		
		/* Tell init task that this thread's initialization is done and ok to run.
		 * Wait for init task to resume operation. */
		xTaskSendMessage(INIT_TASK, 1u);
		vTaskSuspend(NULL);
		taskYIELD();
		
		TickType_t lastTimeWoken = xTaskGetTickCount();
		lastTime = xTaskGetTickCount();
		for (;;)
		{
			#ifdef DEBUG
			activeTask = PID_TASK;
			stackHighWaterMark_PID = uxTaskGetStackHighWaterMark(NULL);
			#endif
			
			parseTaskNotification(ulTaskNotifyTake(pdTRUE, 0));

			if (pidEnabled)
			{
				/* Check for an update from the AHRS thread. This will always pull the latest information. */
				if (xSemaphoreTake(ahrsBufferMutex, 0) == pdPASS)
				{
					xQueueReceive(qAHRS, &ahrs, 0);
					xSemaphoreGive(ahrsBufferMutex);
				}

				/*--------------------------------------
				* Calculate the desired rotation rate
				*--------------------------------------*/
				/* Update the desired angle to achieve */
				if (steppingEnabled && (((uint32_t)xTaskGetTickCount() - lastTime) >= STEP_DELAY_TIME_MS))
				{
					lastTime = (uint32_t)xTaskGetTickCount();

					rAngleDesired = rollStep[currentStep];
					pAngleDesired = pitchStep[currentStep];


					++currentStep;


					if (currentStep == numSteps)
					{
						currentStep = 0;
						rAngleDesired = 0.0;
						pAngleDesired = 0.0;
						steppingEnabled = false;
						signalRecordingStop = true;
					}
				}

				if (signalRecordingStop && (((uint32_t)xTaskGetTickCount() - lastTime) >= STEP_DELAY_TIME_MS))
				{
					signalRecordingStop = false;
					xTaskSendMessage(SDCARD_TASK, SD_CARD_SHUTDOWN);
				}
				

				/* Update the inputs to the angle control loop*/
				pAngFB = ahrs.eulerAngles(0);
				rAngFB = ahrs.eulerAngles(1);

				pAngCtrl.Compute();
				rAngCtrl.Compute();

				#ifdef DEBUG
				volatile float pangf = pAngFB;
				volatile float rangf = rAngFB;
				volatile float pitchAngleSetpoint = pAngleDesired;
				volatile float rollAngleSetpoint = rAngleDesired;
				volatile float pRateOutput = pRateDesired;
				volatile float rRateOutput = rRateDesired;
				#endif

				/*--------------------------------------
				* Calculate the motor command outputs
				*--------------------------------------*/
				pRateFB = ahrs.gyro(1);	/* Pitch Rotation Axis: Sensor Y-Axis
									* Increasing Body Pitch Angle ==> ++Pitch Rate (CW Rotation About Sensor Y)*/

				rRateFB = -ahrs.gyro(0); /* Roll Rotation Axis: Sensor X-Axis
									* Increasing Body Roll Angle ==> --Roll Rate (CCW Rotation About Sensor X)
									*
									* FIX: Multiply by -1 so that increasing roll angle yields increasing rotation rate */

				pRateCtrl.Compute();
				rRateCtrl.Compute();

				pidCMD.pitchControl = pMotorCmd;
				pidCMD.rollControl = rMotorCmd;

				/* Send data over to the motor controller thread without waiting for confirmation of success */
				if (xSemaphoreTake(pidBufferMutex, 0) == pdPASS)
				{
					xQueueOverwrite(qPID, &pidCMD);
					xSemaphoreGive(pidBufferMutex);
				}
				
				/* Push log data to the SDCard task */
				angleControllerLog.tickTime = (uint32_t)xTaskGetTickCount();
				angleControllerLog.pitch_angle_setpoint = pAngleDesired;
				angleControllerLog.roll_angle_setpoint = rAngleDesired;
				angleControllerLog.yaw_angle_setpoint = 0.0f;
				xQueueSendToBack(qSD_PIDAngleSetpoint, &angleControllerLog, 0);
				
				rateControllerLog.tickTime = (uint32_t)xTaskGetTickCount();
				rateControllerLog.pitch_rate_setpoint = pRateDesired;
				rateControllerLog.roll_rate_setpoint = rRateDesired;
				rateControllerLog.yaw_rate_setpoint = 0.0f;
				xQueueSendToBack(qSD_PIDRateSetpoint, &rateControllerLog, 0);
			}

			vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(updateRate_mS));
		}


	}
}


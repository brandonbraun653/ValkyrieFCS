/* C/C++ Includes */
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include <Eigen/Eigen>

/* FreeRTOS Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Project Includes */
#include <ValkyrieFCS/include/fcsConfig.hpp>
#include <ValkyrieFCS/include/dataTypes.hpp>
#include <ValkyrieFCS/include/threads.hpp>
#include <ValkyrieFCS/include/lqr.hpp>

//#define PHI_ANG 0.0
//#define THE_ANG 0.0
//#define PSI_ANG 0.0
//
//#define MOTOR_ARM_LEN_MM 230
//#define MOTOR_ANG_WRT_BODY_X (45.0*(180.0/M_PI))
//#define MOTOR_ANG_WRT_BODY_Y (90.0*(180.0/M_PI) - MOTOR_ANG_WRT_BODY_X)
//
//#define LQR_TEST_BASE_THROTTLE 1200
//
//namespace FCS_LQR
//{
//	const int updateRate_mS = (1.0f / CTRL_UPDATE_FREQ_HZ) * 1000.0f;
//	
//	// Steady Wing Level Flight Angles
//	static const Eigen::Matrix<float, 1, 3> SWLFAngles = (Eigen::Matrix<float, 1, 3>() << 0.0, 0.0, 0.0).finished();
//	
//	/* Input data from the AHRS algorithm */
//	AHRSData_t ahrsData; 
//	
//	/* Output to the motor controller thread */
//	LQRData_t lqrCmd;
//	
//	/* Some coefficients for motor speed calculation */
//	const float d = 1e-6;											//Motor/Prop Drag coefficient
//	const float b = 1.0;											//Motor/Prop Thrust coefficient 
//	const float lx = MOTOR_ARM_LEN_MM*sin(MOTOR_ANG_WRT_BODY_X);	//Effective torque arm length for roll
//	const float ly = MOTOR_ARM_LEN_MM*sin(MOTOR_ANG_WRT_BODY_Y);	//Effective torque arm length for pitch
//	
//	/* This matrix, when multiplied by the idealized input 'U', yields the desired motor 
//	 * commands in (rad/s)^2 only for an X configuration quadcopter with motors rotating 
//	 * like this: https://goo.gl/3jRukg
//	 * 
//	 *	[ 1/(4*b), -1/(4*b*lx),  1/(4*b*ly),  1/(4*d)]
//	 *	[ 1/(4*b),  1/(4*b*lx), -1/(4*b*ly),  1/(4*d)]
//	 *	[ 1/(4*b),  1/(4*b*lx),  1/(4*b*ly), -1/(4*d)]
//	 *	[ 1/(4*b), -1/(4*b*lx), -1/(4*b*ly), -1/(4*d)]	
//	 *	
//	 * Note that before taking the square root, the negative terms that might arise will need
//	 * to be zeroed out. The negative means a reversal of motor spin direction, which is impractical.
//	 * Sometimes the negative term will appear due to a low total thrust 'ft' in the input.*/
//	
//	static const Eigen::Matrix<float, 4, 4> uT = (Eigen::Matrix<float, 4, 4>() << 
//		(1.0f / (4.0f*b)), ( 1.0f / (4.0f*b*lx)),  ( 1.0f / (4.0f*b*ly)),  (0.0f),
//	 	(1.0f / (4.0f*b)), ( 1.0f / (4.0f*b*lx)),  (-1.0f / (4.0f*b*ly)),  (0.0f),
//	 	(1.0f / (4.0f*b)), (-1.0f / (4.0f*b*lx)),  (-1.0f / (4.0f*b*ly)),  (0.0f),
//		(1.0f / (4.0f*b)), (-1.0f / (4.0f*b*lx)),  ( 1.0f / (4.0f*b*ly)),  (0.0f)).finished();
//	
//	
//	static const Eigen::Matrix<float, 4, 4> motorMix = (Eigen::Matrix<float, 4, 4>() << 
//		1.0f, -1.0f,  1.0f,  1.0f,
//		1.0f,  1.0f, -1.0f,  1.0f,
//		1.0f,  1.0f,  1.0f, -1.0f,
//		1.0f, -1.0f, -1.0f, -1.0f).finished();
//	
//	bool lqrEnabled = false;
//	
//	void parseTaskNotification(uint32_t notification)
//	{
//		if (notification == CTRL_ENABLE)
//			lqrEnabled = true;
//
//		if (notification == CTRL_DISABLE)
//			lqrEnabled = false;
//	}
//	
//	void lqrController(void* argument)
//	{
//		#ifdef DEBUG
//		volatile UBaseType_t stackHighWaterMark_LQR = 0;
//		#endif
//		
//		// Placeholders for the current & desired states
//		Eigen::Matrix<float, 3, 1>  currStateAngle, currStateRate, angleError,
//									desStateAngle, desStateRate, rateError;
//		
//		// LQR gains calculated from Matlab (this particular one is quite aggressive)
//		Eigen::Matrix<float, 4, 6> K;
//		K << 
//			0.0,	0.0,	0.0,	0,		0.0,	0.0,
//			0.6667,	0.0,	0.0,	0.1202, 0.0,	0.0,
//			0.0,	1.0,	0.0,	0.0,	0.1394,	0.0,
//			0.0,	0.0,	1.0,	0.0,	0.0,	0.1745;
//		
//		// State Matrix
//		Eigen::Matrix<float, 6, 1> State;
//		
//		// Idealized control signal input [ft, tau_x, tau_y, tau_z]
//		// Where ft -> total thrust, tau_p -> moment about an axis 'p'
//		Eigen::Matrix<float, 4, 1> U;
//		
//		// Calculated Motor RPM Values in rad/s
//		Eigen::Matrix<float, 4, 1> motorDelta;
//		
//		// Scales the errors between state and desired [Phi, Theta, Psi]
//		Eigen::Matrix<float, 3, 1> angleErrorGains;
//		angleErrorGains << 5.0, 5.0, 5.0;
//		
//		// Scales the errors between state and desired [dPhi, dTheta, dPsi] <=> [P,Q,R]
//		Eigen::Matrix<float, 3, 1> rateErrorGains;
//		rateErrorGains << 5.0, 5.0, 5.0;
//		
//		
//		
//		/* Tell init task that this thread's initialization is done and ok to run.
//		 * Wait for init task to resume operation. */
//		xTaskSendMessage(INIT_TASK, 1u);
//		vTaskSuspend(NULL);
//		taskYIELD();
//		
//		TickType_t lastTimeWoken = xTaskGetTickCount();
//		for (;;)
//		{
//			#ifdef DEBUG
//			activeTask = CTRL_TASK;
//			stackHighWaterMark_LQR = uxTaskGetStackHighWaterMark(NULL);
//			#endif
//			
//			/* Check for start/stop signals from an external thread */
//			parseTaskNotification(ulTaskNotifyTake(pdTRUE, 0));
//			
//			if (lqrEnabled)
//			{
//				/* Check for an update from the AHRS thread. This will always pull the latest information. */
//				if (xSemaphoreTake(ahrsBufferMutex, 0) == pdPASS)
//				{
//					xQueueReceive(qAHRS, &ahrsData, 0);
//					xSemaphoreGive(ahrsBufferMutex);
//					
//					// Angles: [roll, pitch, yaw]
//					//currStateAngle << ahrsData.roll(), ahrsData.pitch(), ahrsData.yaw();
//					currStateAngle << ahrsData.roll(), ahrsData.pitch(), 0.0f;
//					
//					/* Rates: [p, q, r]
//					p -> Increasing gy corresponds to pitch up
//					q -> Increasing gx corresponds to roll right 
//					r -> Increasing gz corresponds to rotate left
//					*/
//					//currStateRate << ahrsData.gy(), ahrsData.gx(), ahrsData.gz(); 
//					currStateRate << ahrsData.gy(), ahrsData.gx(), 0.0f; 
//				}
//				
//				/* Calculate the Angle Error signal */
//				angleError = desStateAngle - currStateAngle;
//				angleError = (angleError.array() * angleErrorGains.array()).matrix();
//				
//				
//				/* Calculate the Rate Error signal, using the current angle error as the rate reference */
//				rateError = angleError - currStateRate;
//				rateError = (rateError.array() * rateErrorGains.array()).matrix();
//				
//				
//				/* Combine the two signals into a [6x1] state deviation error vector */
//				State << angleError, rateError;
//				
//				/* Calculate the idealized control signal U: [ft, tau_x, tau_y, tau_z] 
//				 * Normally this is -K*x, but the negative was brought into the error calculations */
//				U = -K*State;
//				
//				/* Manually calculate the desired thrust vector 'ft' because the system model
//				 * has no concept of height. It is only stabilizing angles and rotation rates. */
//				U(0) = LQR_TEST_BASE_THROTTLE;
//				
//				/* Convert 'U' into motor commands */
//				motorDelta = motorMix*U;
//				
//				
//				lqrCmd.m1 = (uint16_t)(motorDelta(0));
//				lqrCmd.m2 = (uint16_t)(motorDelta(1));
//				lqrCmd.m3 = (uint16_t)(motorDelta(2));
//				lqrCmd.m4 = (uint16_t)(motorDelta(3));
//				
//				
//				/* Post commands to the motor thread */
//				if (xSemaphoreTake(lqrBufferMutex, 0) == pdPASS)
//				{
//					xQueueOverwrite(qLQR, &lqrCmd);
//					xSemaphoreGive(lqrBufferMutex);
//				}
//			}
//			
//			vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(updateRate_mS));
//		}
//	}
//}	
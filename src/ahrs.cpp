/* C/C++ Includes */
#include <stdint.h>
#include <stdlib.h>

/* Thor Includes */
#include "include/thor.h"
#include "include/spi.h"
#include "include/gpio.h"

/* Boost Includes */
#include <boost/smart_ptr.hpp>
#include <boost/make_shared.hpp>

/* FreeRTOS Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Project Includes */
#include "ahrs.hpp"
#include "fcsConfig.hpp"
#include "dataTypes.hpp"
#include "threading.hpp"

/* Signal Processing Includes */
#include "libraries/DspFilters/Filter.h"
#include "libraries/DspFilters/RBJ.h"
#include "libraries/DspFilters/ChebyshevI.h"

/* Madgwick Filter */
#include "madgwick.hpp"

IMUData_t imuData;		/* Input struct to read from the IMU thread */
AHRSDataDeg_t ahrsData;	/* Output struct to push to the motor controller thread */

#define NUM_SAMPLES 1

void ahrsTask(void* argument)
{
	float beta = 2.5; //2.5 seems to work fairly well. 10.0 has a fantastic response time but is horrendously noisy.
	float roll, pitch, yaw;
	Eigen::Vector3f accel, gyro, mag, eulerDeg;
	
	MadgwickFilter ahrs((AHRS_UPDATE_RATE_MULTIPLIER * SENSOR_UPDATE_FREQ_HZ), beta);
	
//	Dsp::SimpleFilter <Dsp::RBJ::LowPass> f;
//	f.setup(SENSOR_UPDATE_FREQ_HZ, // sample rate Hz
//	         50, // cutoff frequency Hz
//	         1);  // "Q" (resonance)
	
	Dsp::SimpleFilter <Dsp::ChebyshevI::BandStop <3>, 2> f;
	f.setup(3, // order
	         SENSOR_UPDATE_FREQ_HZ, // sample rate
	         25, // center frequency
	         50,  // band width
	         1);  // ripple dB
	
	
	float* rawData[9];
	for (int i = 0; i < 9; i++)
		rawData[i] = new float[NUM_SAMPLES];
	
	
	for (;;)
	{
		/* Indefinitely wait until new sensor data has become available */
		xQueueReceive(qIMU, &imuData, portMAX_DELAY);
		
		accel << imuData.ax, imuData.ay, imuData.az;
		gyro  << imuData.gx, imuData.gy, imuData.gz;
		mag   << imuData.mx, imuData.my, imuData.mz;
		
		
		/*----------------------------
		* Filtering 
		*---------------------------*/
//		rawData[0][0] = imuData.ax;
//		rawData[1][0] = imuData.ay;
//		rawData[2][0] = imuData.az;
//		
//		rawData[3][0] = imuData.gx;
//		rawData[4][0] = imuData.gy;
//		rawData[5][0] = imuData.gz;
//		
//		rawData[6][0] = imuData.mx;
//		rawData[7][0] = imuData.my;
//		rawData[8][0] = imuData.mz;
//		
//		f.process(NUM_SAMPLES, rawData);
//		
//		accel << rawData[0][0], rawData[1][0], rawData[2][0];
//		gyro << rawData[3][0], rawData[4][0], rawData[5][0];
//		mag << rawData[6][0], rawData[7][0], rawData[8][0];
		
		
		/*----------------------------
		 * AHRS Algorithm 
		 *---------------------------*/
		/* The Madgwick filter needs to run between 3-5 times as fast IMU measurements
		 * to achieve decent convergence to a stable value. This thread only runs when 
		 * new data has arrived from the IMU, so frequency multiplication is as simple 
		 * as looping 3-5 times here. */
		for (int i = 0; i < AHRS_UPDATE_RATE_MULTIPLIER; i++)
			ahrs.update(accel, gyro, mag);
		
		ahrs.getEulerDeg(eulerDeg);		//Generate Euler angles from internal quaternion data
		ahrsData(eulerDeg);				//Copy the angles into the data struct assuming [PITCH, ROLL, YAW] structure
		
		#ifdef DEBUG
		float pitch = eulerDeg(0);
		float roll = eulerDeg(1);
		float yaw = eulerDeg(2);
		#endif

		/* Send data over to the PID thread without waiting for confirmation of success */
		xSemaphoreTake(ahrsBufferMutex, 2);
		xQueueSendToBack(qAHRS, &ahrsData, 0);
		xSemaphoreGive(ahrsBufferMutex);
	}
}
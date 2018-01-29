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
//#include "libraries/DspFilters/Filter.h"
//#include "libraries/DspFilters/ChebyshevI.h"

/* Madgwick Filter */
#include "madgwick.hpp"

IMUData_t imuData;		/* Input struct to read from the IMU thread */
AHRSDataDeg_t ahrsData;	/* Output struct to push to the motor controller thread */

void ahrsTask(void* argument)
{
	float beta = 2.5; //2.5 seems to work fairly well. 10.0 has a fantastic response time but is horrendously noisy.
	float roll, pitch, yaw;
	Eigen::Vector3f accel, gyro, mag, eulerDeg;
	
	MadgwickFilter ahrs((AHRS_UPDATE_RATE_MULTIPLIER * SENSOR_UPDATE_FREQ_HZ), beta);
	
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
		//Currently don't need any signal processing. Implemented on IMU chip.
		
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

		/* Send data over to the PID thread without waiting for confirmation of success */
		xQueueSendToBack(qAHRS, &ahrsData, 0);
	}
}
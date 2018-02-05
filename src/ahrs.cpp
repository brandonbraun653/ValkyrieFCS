/* C/C++ Includes */
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

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
//#include "libraries/DspFilters/RBJ.h"
//#include "libraries/DspFilters/ChebyshevI.h"

/* Madgwick Filter */
#include "madgwick.hpp"

IMUData_t imuData;		/* Input struct to read from the IMU thread */
AHRSDataDeg_t ahrsData;	/* Output struct to push to the motor controller thread */

#define NUM_SAMPLES 1

#define DEADBAND_ACCEL 0.5f
#define DEADBAND_GYRO 0.5f

void ahrsTask(void* argument)
{
	float beta = 10.0; //2.5 seems to work fairly well. 10.0 has a fantastic response time but is horrendously noisy.
	float roll, pitch, yaw;
	Eigen::Vector3f accel_raw, gyro_raw, mag_raw;
	Eigen::Vector3f accel_filtered, gyro_filtered, mag_filtered, eulerDeg;
	
	MadgwickFilter ahrs((AHRS_UPDATE_RATE_MULTIPLIER * SENSOR_UPDATE_FREQ_HZ), beta);	
	
	float lastX = 0.0f, newX = 0.0f;
	float dt = (1.0f / SENSOR_UPDATE_FREQ_HZ);
	float tau = 0.1f;
	float alpha_lp = dt / tau;
	
	float fc = 400.0f;
	float alpha_hp = (1.0f / (2.0f*3.14159f*dt*fc + 1));
	
	accel_filtered << 0.0, 0.0, 0.0;
	gyro_filtered << 0.0, 0.0, 0.0;
	mag_filtered << 0.0, 0.0, 0.0;
	
	accel_raw << 0.0, 0.0, 0.0;
	gyro_raw << 0.0, 0.0, 0.0;
	mag_raw << 0.0, 0.0, 0.0;
	
	Eigen::Vector3f gyro_raw_last;
	gyro_raw_last << 0.0, 0.0, 0.0;
	
	for (;;)
	{
		activeTask = AHRS_TASK;
		
		/* Indefinitely wait until new sensor data has become available */
		xQueueReceive(qIMU, &imuData, portMAX_DELAY);
		
		gyro_raw_last = gyro_raw;
		
		accel_raw << imuData.ax, imuData.ay, imuData.az;
		gyro_raw  << imuData.gx, imuData.gy, imuData.gz;
		mag_raw   << imuData.mx, imuData.my, imuData.mz;
		
		
		/*----------------------------
		* Filtering 
		*---------------------------*/
		accel_filtered += alpha_lp * (accel_raw - accel_filtered);
		
		//gyro_filtered = alpha_hp * (gyro_filtered + gyro_raw - gyro_raw_last);
		
		mag_filtered = mag_raw;
		
		/*----------------------------
		 * AHRS Algorithm 
		 *---------------------------*/
		/* The Madgwick filter needs to run between 3-5 times as fast IMU measurements
		 * to achieve decent convergence to a stable value. This thread only runs when 
		 * new data has arrived from the IMU, so frequency multiplication is as simple 
		 * as looping 3-5 times here. */
		taskENTER_CRITICAL();
		
		for (int i = 0; i < AHRS_UPDATE_RATE_MULTIPLIER; i++)
			ahrs.update(accel_filtered, gyro_filtered, mag_filtered);
		
		ahrs.getEulerDeg(eulerDeg);		//Generate Euler angles from internal quaternion data
		ahrsData(eulerDeg);				//Copy the angles into the data struct assuming [PITCH, ROLL, YAW] structure
		
		#ifdef DEBUG
		float pitch = eulerDeg(0);
		float roll = eulerDeg(1);
		float yaw = eulerDeg(2);
		
		float ax = accel_filtered(0);
		float ay = accel_filtered(1);
		float az = accel_filtered(2);
		
		float gx = gyro_filtered(0);
		float gy = gyro_filtered(1);
		float gz = gyro_filtered(2);
		#endif
		taskEXIT_CRITICAL();
		
		
		/* Send data over to the PID thread without waiting for confirmation of success */
		xSemaphoreTake(ahrsBufferMutex, 2);
		xQueueSendToBack(qAHRS, &ahrsData, 0);
		xSemaphoreGive(ahrsBufferMutex);
		
		
	}
}
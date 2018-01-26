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

const int convergenceIterations = AHRS_UPDATE_FREQ_HZ / SENSOR_UPDATE_FREQ_HZ;
IMUData_t rawIMUData;


void ahrsTask(void* argument)
{
	float beta = 10.0;
	float roll, pitch, yaw;
	Eigen::Vector3f accel, gyro, mag, eulerDeg;
	
	MadgwickFilter ahrs(AHRS_UPDATE_FREQ_HZ, beta);
	
	for (;;)
	{
		/* Indefinitely wait until new sensor data has become available */
		xQueueReceive(qIMURawData, &rawIMUData, portMAX_DELAY);
		
		accel << rawIMUData.ax, rawIMUData.ay, rawIMUData.az;
		gyro  << rawIMUData.gx, rawIMUData.gy, rawIMUData.gz;
		mag   << rawIMUData.mx, rawIMUData.my, rawIMUData.mz;
		
		
		/*----------------------------
		 * Filtering 
		 *---------------------------*/
		
		
		/*----------------------------
		 * AHRS Algorithm 
		 *---------------------------*/
		for(int i = 0 ; i < convergenceIterations ; i++)
			ahrs.update(accel, gyro, mag);
		
		ahrs.getEulerDeg(eulerDeg);
		
		pitch = eulerDeg(0);
		roll = eulerDeg(1);
		yaw = eulerDeg(2);
		
		/*----------------------------
		 * Update Queues and other post processing things (TODO: rename this crap)
		 *---------------------------*/
		//Update the SD card task with new data
		//Update console logger??
		//Update the motor control algorithm 
		
	}
}
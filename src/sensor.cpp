/* C/C++ Includes */
#include <stdint.h>
#include <stdlib.h>

/* Thor Includes */
#include "include/thor.h"
#include "include/spi.h"
#include "include/gpio.h"
#include "include/exceptions.h"

/* Boost Includes */
#include <boost/smart_ptr.hpp>
#include <boost/make_shared.hpp>

/* FreeRTOS Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Project Includes */
#include "fcsConfig.hpp"
#include "sensor.hpp"
#include "LSM9DS1.hpp"
#include "dataTypes.hpp"
#include "threading.hpp"


const int updateRate_mS = (1.0 / SENSOR_UPDATE_FREQ_HZ) * 1000.0;
const int magMaxUpdateRate_mS = (1.0 / LSM9DS1_M_MAX_BW) * 1000.0;

IMUData_t sensorData;

void sensorTask(void* argument)
{
	/* Object allocation */
	GPIOClass_sPtr lsm_ss_xg = boost::make_shared<GPIOClass>(GPIOC, PIN_4, ULTRA_SPD, NOALTERNATE);
	GPIOClass_sPtr lsm_ss_m = boost::make_shared<GPIOClass>(GPIOC, PIN_3, ULTRA_SPD, NOALTERNATE);
	SPIClass_sPtr lsm_spi = spi2;
	
	LSM9DS1 imu(lsm_spi, lsm_ss_xg, lsm_ss_m);
	
	
	/* Force halt of the device if the IMU cannot be reached */
	if (imu.begin() == 0)
		BasicErrorHandler("The IMU WHO_AM_I registers did not return valid readings");
	
	imu.calibrate(true);		/* "true" forces an automatic software subtraction of the calculated bias from all further data */
	imu.calibrateMag(true);		/* "true" writes the offest into the mag sensor hardware for automatic subtraction in results */
	
	int count = 0;
	TickType_t lastTimeWoken = xTaskGetTickCount();
	for (;;)
	{
		activeTask = SENSOR_TASK;
		/* Update Accel & Gyro Data at whatever frequency set by user. Max bandwidth on
		 * chip is 952Hz which will saturate FreeRTOS if sampled that often.*/
		imu.readAccel(); 
		imu.readGyro(); 
		
		/* Update Mag Data at a max frequency set by LSM9DS1_M_MAX_BW (~75Hz) */
		#if (SENSOR_UPDATE_FREQ_HZ > LSM9DS1_M_MAX_BW)
		if (count > magMaxUpdateRate_mS)
		{
			imu.readMag();
			count = 0;
		}
		else
			count += updateRate_mS;
		#else
		imu.readMag();
		#endif
			
		/* Convert raw data from chip into meaningful data */
		imu.calcAccel(); imu.calcGyro(); imu.calcMag();
		
		
		/* Acceleration in m/s^2 */
		sensorData.ax = imu.aRaw[0];	
		sensorData.ay = imu.aRaw[1];
		sensorData.az = imu.aRaw[2];
		
		/* Rotation Rate in dps */
		sensorData.gx = imu.gRaw[0];	
		sensorData.gy = imu.gRaw[1];
		sensorData.gz = imu.gRaw[2];
		
		/* Mag Field Strength in Gauss */
		sensorData.mx = -1.0f * imu.mRaw[1];		//Align mag x with accel x
		sensorData.my = -1.0f * imu.mRaw[0];		//Align mag y with accel y
		sensorData.mz = -1.0f * imu.mRaw[2];		//from LSM9DS1, mag z is opposite direction of accel z
		
		sensorData.mSTime = 0.0; //TODO: Get the RTC working so we can add a time stamp for more accurate data logging
		
		/* Send data over to the AHRS thread without waiting for confirmation of success */
		xQueueSendToBack(qIMU, &sensorData, 0);
		vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(updateRate_mS));
	}
}

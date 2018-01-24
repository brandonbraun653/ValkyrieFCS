#include "ahrs.hpp"


void ahrsTask(void* argument)
{
	/* Object allocation */
 	GPIOClass_sPtr lsm_ss_xg = boost::make_shared<GPIOClass>(GPIOC, PIN_4, ULTRA_SPD, NOALTERNATE);
 	GPIOClass_sPtr lsm_ss_m = boost::make_shared<GPIOClass>(GPIOC, PIN_3, ULTRA_SPD, NOALTERNATE);
	SPIClass_sPtr lsm_spi = spi2;
	
	LSM9DS1 imu(lsm_spi, lsm_ss_xg, lsm_ss_m);
	
	imu.settings.device.commInterface = IMU_MODE_SPI;
	
	uint16_t validIMU = imu.begin();
	imu.calibrate(true);
	//imu.calibrateMag(true);
	
	uint8_t accelAvailable = imu.accelAvailable();
	
	
	IMUData_t data;
	
	TickType_t lastTimeWoken = xTaskGetTickCount();
	for (;;)
	{
		if (validIMU)
		{
			
			/* MAKE NOTE IN DATA SHEET THAT AUTO ADDR INCR DOES NOT WORK */
			imu.readGyro();
			imu.readAccel();
			imu.readMag();
		
			data.ax = imu.calcAccel(imu.ax);
			data.ay = imu.calcAccel(imu.ay);
			data.az = imu.calcAccel(imu.az);
		
			data.gx = imu.calcGyro(imu.gx);
			data.gy = imu.calcGyro(imu.gy);
			data.gz = imu.calcGyro(imu.gz);
		
			data.mx = imu.calcMag(imu.mx);
			data.my = imu.calcMag(imu.my);
			data.mz = imu.calcMag(imu.mz);	
		}
		
		
		/* This should trigger a write to the SD flight logger */
		//xQueueSendToBack(qIMUFlightData, &sensorData, 0);
		
		/* Constant frequency sampling of 100 Hz */
		vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(10));
	}
}
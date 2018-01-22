#include "ahrs.hpp"


void ahrsTask(void* argument)
{
	/* Fill in the Accelerometer and Mag details */
	LSM9DS0_Settings sensor_settings;
	sensor_settings.scale.accel = A_SCALE_8G;
	sensor_settings.scale.mag = M_SCALE_2GS;
	sensor_settings.scale.gyro = G_SCALE_2000DPS;
				   
	sensor_settings.odr.accel = A_ODR_100;
	sensor_settings.odr.mag = M_ODR_100;
	sensor_settings.odr.gyro = G_ODR_190_BW_125;
	
	/* Object allocation */
 	GPIOClass_sPtr lsm_ss_xm = boost::make_shared<GPIOClass>(GPIOC, PIN_4, ULTRA_SPD, NOALTERNATE);		/* Accel/Mag Slave Select */
 	GPIOClass_sPtr lsm_ss_g = boost::make_shared<GPIOClass>(GPIOC, PIN_3, ULTRA_SPD, NOALTERNATE);		/* Gyro Slave Select */
	SPIClass_sPtr lsm_spi = spi2;
	
 	LSM9DS0 sensor(lsm_spi, lsm_ss_xm, lsm_ss_g, sensor_settings);
	
	/* Initialization: 
	 * The sensor class handles GPIO and SPI init. SPI is defaulted to blocking 
	 * mode, but this is ok due to how incredibly short it takes to read out data. ~100uS*/
 	sensor.initialize();
	
	
	IMUData_t sensorData;
	
	
	TickType_t lastTimeWoken = xTaskGetTickCount();
	for (;;)
	{
		portENTER_CRITICAL();
		sensor.readAll();
		sensor.calcAll();
		portEXIT_CRITICAL();
		
		sensorData.ax = sensor.data.accel.x;
		sensorData.ay = sensor.data.accel.y;
		sensorData.az = sensor.data.accel.z;
		
		sensorData.gx = sensor.data.gyro.x;
		sensorData.gy = sensor.data.gyro.y;
		sensorData.gz = sensor.data.gyro.z;
		
		sensorData.mx = sensor.data.mag.x;
		sensorData.my = sensor.data.mag.y;
		sensorData.mz = sensor.data.mag.z;
		
		sensorData.mSTime = 0.0;
		
		/* This should trigger a write to the SD flight logger */
		xQueueSendToBack(qIMUFlightData, &sensorData, 0);
		
		/* Constant frequency sampling of 100 Hz */
		vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(10));
	}
}
#include <../CMSIS_RTOS/cmsis_os.h>
#include "include/thor.h"
#include "include/spi.h"
#include "include/uart.h"
#include "include/gpio.h"
#include "include/print.h"
#include "libraries/SD/sd.h"
#include "stm32f7_lsm9ds0.h"

#ifdef DEBUG
#include "SysprogsProfiler.h"
#endif

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

using namespace ThorDef::GPIO;
using namespace ThorDef::SPI;

/* Quick note, don't allocate memory up here. It will be null and cause issues.
 * Only allocate inside the main func(). */

int main(void)
{
	HAL_Init();
	ThorSystemClockConfig();

	#ifdef DEBUG
	InitializeSamplingProfiler();
	InitializeInstrumentingProfiler();
	#endif 
	
	/* Fill in the Accelerometer and Mag details */
	LSM9DS0_Settings sensor_settings;
	sensor_settings.scale.accel = A_SCALE_2G;
	sensor_settings.scale.mag = M_SCALE_2GS;
	sensor_settings.scale.gyro = G_SCALE_245DPS;
				   
	sensor_settings.odr.accel = A_ODR_50;
	sensor_settings.odr.mag = M_ODR_50;
	sensor_settings.odr.gyro = G_ODR_190_BW_125;
	
	/* Object allocation */
	GPIOClass_sPtr ledPin = boost::make_shared<GPIOClass>(GPIOA, PIN_5, ULTRA_SPD, NOALTERNATE);		/* LED Status Signal */
	GPIOClass_sPtr SD_SSPin = boost::make_shared<GPIOClass>(GPIOB, PIN_12, ULTRA_SPD, NOALTERNATE);		/* SD Slave Select */
	GPIOClass_sPtr lsm_ss_xm = boost::make_shared<GPIOClass>(GPIOA, PIN_9, ULTRA_SPD, NOALTERNATE);		/* Accel/Mag Slave Select */
	GPIOClass_sPtr lsm_ss_g = boost::make_shared<GPIOClass>(GPIOA, PIN_8, ULTRA_SPD, NOALTERNATE);		/* Gyro Slave Select */
	
	SPIClass_sPtr sd_spi = spi2;
	SPIClass_sPtr lsm_spi = spi3;
	
	UARTClass_sPtr uart = uart4;
	
	SDCard_sPtr sd = boost::make_shared<SDCard>(sd_spi);
	
	LSM9DS0 sensor(lsm_spi, lsm_ss_xm, lsm_ss_g, sensor_settings);
	sensor.initialize();
	
	/* Initialize success led */
	ledPin->mode(OUTPUT_PP);
	ledPin->write(HIGH);
	
	/* Initialize SPI */
	sd_spi->attachPin(SD_SSPin);
	sd_spi->setSSMode(SS_MANUAL_CONTROL);
	sd_spi->begin(EXTERNAL_SLAVE_SELECT);
	
	/* Initialize serial */
	uart->begin(115200);
	uart->setTxModeIT();
	uart->setRxModeIT();
	
	/* Initialize the SD Card */
	sd->initialize();
	
	/* If the above doesn't work, neither will this: */
	volatile float testData[] = { 0.0, 0.0, 0.0 };
	volatile FRESULT error = FR_OK;
	
	std::string outputMessage, sAX, sAY, sAZ;
	
	sd->fopen("raw_data.dat", FA_CREATE_ALWAYS | FA_WRITE);
	
	for (;;)
	{
		/* Grab a new set of data */
		sensor.readAll();
		sensor.calcAll();
		
		testData[0] = sensor.sensorData.accel.x;
		testData[1] = sensor.sensorData.accel.y;
		testData[2] = sensor.sensorData.accel.z;
		
		/* Write that data to the serial monitor */
		sAX = float2String(testData[0]);
		sAY = float2String(testData[1]);
		sAZ = float2String(testData[2]);
		
		outputMessage = "Accel X: " + sAX + " \tAccel Y: " + "\tAccel Z: " + sAZ + "\n";
		uart->write(outputMessage);
		
		/* Write to the sd card */
		
		
		
		
		ledPin->write(HIGH);
		HAL_Delay(1);
		ledPin->write(LOW);
		HAL_Delay(1);
	}
}
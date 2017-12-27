#include <../CMSIS_RTOS/cmsis_os.h>
#include "include/thor.h"
#include "include/spi.h"
#include "include/uart.h"
#include "include/gpio.h"
#include "include/print.h"
#include "stm32f7_lsm9ds0.h"

#ifdef DEBUG
#include "SysprogsProfiler.h"
#endif

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

/* Thread Task Includes */
#include "sdcard.hpp"

using namespace ThorDef::GPIO;
using namespace ThorDef::SPI;

/* Quick note, don't allocate memory up here. It will be null and cause issues.
 * Only allocate inside the main func(). */
osThreadId sdTaskHandle;

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
	GPIOClass_sPtr lsm_ss_xm = boost::make_shared<GPIOClass>(GPIOA, PIN_9, ULTRA_SPD, NOALTERNATE);		/* Accel/Mag Slave Select */
	GPIOClass_sPtr lsm_ss_g = boost::make_shared<GPIOClass>(GPIOA, PIN_8, ULTRA_SPD, NOALTERNATE);		/* Gyro Slave Select */
	
	SPIClass_sPtr lsm_spi = spi3;
	
	UARTClass_sPtr uart = uart4;
	
	
	LSM9DS0 sensor(lsm_spi, lsm_ss_xm, lsm_ss_g, sensor_settings);
	sensor.initialize();
	
	/* Initialize serial */
	uart->begin(115200);
	uart->setTxModeIT();
	uart->setRxModeIT();
	
	/* If the above doesn't work, neither will this: */
	volatile float testData[] = { 0.0, 0.0, 0.0 };
	volatile FRESULT error = FR_OK;
	
	std::string outputMessage, sAX, sAY, sAZ;
	
	sdCardTaskInit();
	
	osThreadDef(SDCARD, sdCardTask, osPriorityNormal, 0, (uint16_t)4096);
	sdTaskHandle = osThreadCreate(osThread(SDCARD), NULL);
	
	osKernelStart();
	for (;;)
	{
	}
}
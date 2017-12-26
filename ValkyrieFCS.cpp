#include <../CMSIS_RTOS/cmsis_os.h>
#include "include/thor.h"
#include "include/spi.h"
#include "include/uart.h"
#include "include/gpio.h"
#include "libraries/SD/sd.h"

#ifdef DEBUG
#include "SysprogsProfiler.h"
#endif

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

using namespace ThorDef::GPIO;
using namespace ThorDef::SPI;

SPIClass_sPtr spi = spi2;
UARTClass_sPtr uart = uart4;
SDCard_sPtr sd = boost::make_shared<SDCard>(spi);
GPIOClass_sPtr ledPin = boost::make_shared<GPIOClass>(GPIOA, PIN_5, ULTRA_SPD, NOALTERNATE);
GPIOClass_sPtr SD_SSPin = boost::make_shared<GPIOClass>(GPIOB, PIN_12, ULTRA_SPD, NOALTERNATE);

int main(void)
{
	HAL_Init();
	ThorSystemClockConfig();

	#ifdef DEBUG
	InitializeSamplingProfiler();
	InitializeInstrumentingProfiler();
	#endif 
	
	uint8_t testTextTX[] = "\nWell I guess this worked.\n";
	size_t txSize = sizeof(testTextTX) / sizeof(testTextTX[0]);
	uint8_t testTextRX[50];
	uint32_t total = 0;
	volatile FRESULT error = FR_OK;
	memset(testTextRX, 0, 50);
	
	/* Initialize success led */
	ledPin->mode(OUTPUT_PP);
	ledPin->write(HIGH);
	
	/* Initialize SPI */
	spi->attachPin(SD_SSPin);
	spi->setSSMode(SS_MANUAL_CONTROL);
	
	spi->begin(EXTERNAL_SLAVE_SELECT);
	spi->setTxModeBlock();
	spi->setRxModeBlock();
	
	/* Initialize serial */
	uart->begin(115200);
	uart->setTxModeIT();
	uart->setRxModeIT();
	
	/* Initialize the SD Card */
	sd->initialize();

	/* Write and read!! */
	//error = sd->format(FM_FAT32);
	error = sd->fopen("newTest.txt", FA_CREATE_ALWAYS | FA_WRITE);
	error = sd->write(testTextTX, txSize, total);
	error = sd->fclose();

	error = sd->fopen("newTest.txt", FA_OPEN_EXISTING | FA_READ);
	error = sd->read(testTextRX, txSize, total);
	error = sd->fclose();
	
	const char* msg = "Received Data:\n";
	uart->write((uint8_t*)msg, sizeof(msg) / sizeof(msg[0]));
	HAL_Delay(1);
	uart->write(testTextRX, txSize);
	
	/* If the above doesn't work, neither will this: */
	volatile double testData[] = { 0.23434, -0.7373, 3.912345 };
	volatile size_t sizeTest = sizeof(testData);

	error = sd->fopen("raw_data.dat", FA_CREATE_ALWAYS | FA_WRITE);
	error = sd->write((uint8_t*)testData, sizeTest, total);
	error = sd->fclose();

	memset((void*)testData, 0, sizeTest);

	error = sd->fopen("raw_data.dat", FA_OPEN_EXISTING | FA_READ);
	error = sd->read((uint8_t*)testData, sizeTest, total);
	error = sd->fclose();
	
	if (error != FR_OK)
		while (1)
		{
			;
		}
	
	sd->deInitialize();

	for (;;)
	{
		ledPin->write(HIGH);
		HAL_Delay(250);
		ledPin->write(LOW);
		HAL_Delay(250);
	}
}
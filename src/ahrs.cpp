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
#include "libraries/DspFilters/ChebyshevI.h"

const int convergenceIterations = AHRS_UPDATE_FREQ_HZ / SENSOR_UPDATE_FREQ_HZ;
IMUData_t rawIMUData;

#define AHRS_DATA_SIZE 9
#define AHRS_SAMPLE_LENGTH 1

float* imudata[AHRS_DATA_SIZE];



void ahrsTask(void* argument)
{
	/* Initialize the data buffer for signal processing */
	for (int i = 0; i < AHRS_DATA_SIZE; i++)
		imudata[i] = new float(AHRS_SAMPLE_LENGTH);
	
//	Dsp::Filter* lpf = new Dsp::SmoothedFilterDesign
//		<Dsp::Butterworth::Design::LowPass<4>, AHRS_DATA_SIZE, Dsp::DirectFormII> (1);
	
	Dsp::SimpleFilter <Dsp::ChebyshevI::BandStop <3>, 2> f;
	f.setup(
		3,		// order
		100,	// sample rate
	    25,	// center frequency
		15,	// band width
		1);     // ripple dB
	   
	
	
	for (;;)
	{
		/* Indefinitely wait until new sensor data has become available */
		xQueueReceive(qIMURawData, &rawIMUData, portMAX_DELAY);
		
		
		/*----------------------------
		 * Filtering 
		 *---------------------------*/
		imudata[0][0] = rawIMUData.ax;
		imudata[1][0] = rawIMUData.ay;
		imudata[2][0] = rawIMUData.az;
		
		imudata[3][0] = rawIMUData.gx;
		imudata[4][0] = rawIMUData.gy;
		imudata[5][0] = rawIMUData.gz;
		
		imudata[6][0] = rawIMUData.mx;
		imudata[7][0] = rawIMUData.my;
		imudata[8][0] = rawIMUData.mz;
		
		//f.process(AHRS_SAMPLE_LENGTH, imudata);
		f.process(1, imudata);
		
		
		/*----------------------------
		 * AHRS Algorithm 
		 *---------------------------*/
		for(int i = 0 ; i < convergenceIterations ; i++)
		{
			
			//Do the madgwick filter here
		}
		
		/*----------------------------
		 * Update Queues and other post processing things (TODO: rename this crap)
		 *---------------------------*/
		//Update the SD card task with new data
		//Update console logger??
		//Update the motor control algorithm 
		
	}
}
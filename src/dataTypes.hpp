#pragma once
#ifndef DATATYPES_HPP_
#define DATATYPES_HPP_

#include <stdint.h>
#include <stdlib.h>

#include "include/thor.h"
#include "include/thor_definitions.h"

struct IMUData_t
{
	float ax, ay, az;	/* Accelerometer (X,Y,Z) */
	float gx, gy, gz;	/* Gyroscope	 (X,Y,Z) */
	float mx, my, mz;	/* Magnetometer  (X,Y,Z) */
	float mSTime;		/* Time stamp in mS */
};

struct CMDData_t
{
	uint8_t* rawPacket;
	size_t rawPacketSize;
	float rxTimeStamp;
	uint8_t priority;
};

/*------------------------------
* Radio
*-----------------------------*/
struct Radio_Command
{
	//cmd details here
};

struct Radio_Control
{
	float THROTTLE = 0.0;
	float ROLL = 0.0;
	float PITCH = 0.0;
	float YAW = 0.0;
};

struct Radio_Request
{
	//request details here
};


#endif 
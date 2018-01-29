#pragma once
#ifndef DATATYPES_HPP_
#define DATATYPES_HPP_

/* C/C++ Includes */
#include <stdint.h>
#include <stdlib.h>

/* Thor Includes */
#include "include/thor.h"
#include "include/thor_definitions.h"

/* Eigen Includes */
#include "Eigen\Eigen"


struct IMUData_t
{
	float ax, ay, az;	/* Accelerometer (X,Y,Z) */
	float gx, gy, gz;	/* Gyroscope	 (X,Y,Z) */
	float mx, my, mz;	/* Magnetometer  (X,Y,Z) */
	float mSTime;		/* Time stamp in mS */
};

struct AHRSDataDeg_t	/* Units: Degrees */
{
	void operator()(Eigen::Vector3f val)
	{
		pitch = val(0);
		roll = val(1);
		yaw = val(2);
	}

	float pitch = 0.0;
	float roll = 0.0;
	float yaw = 0.0;
};

struct PIDData_t
{
	float pitchControl = 0.0;
	float rollControl = 0.0;
	float yawControl = 0.0;
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
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

struct AHRSDataDeg_t
{
	void operator()(Eigen::Vector3f euler_deg, Eigen::Vector3f acceleration_ms2,
		Eigen::Vector3f gyroscope_dps, Eigen::Vector3f magnetometer_g)
	{
		eulerAngles = euler_deg;		
		accel = acceleration_ms2;
		gyro = gyroscope_dps;
		mag = magnetometer_g;
	}
	
	Eigen::Vector3f eulerAngles;	/* [PITCH, ROLL, YAW] (deg) */
	Eigen::Vector3f accel;			/* [X, Y, Z] (m/s^2) */
	Eigen::Vector3f gyro;			/* [X, Y, Z] (dps) */
	Eigen::Vector3f mag;			/* [X, Y, Z] (gauss) */
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
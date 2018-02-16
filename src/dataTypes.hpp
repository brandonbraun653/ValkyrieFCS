#pragma once
#ifndef DATATYPES_HPP_
#define DATATYPES_HPP_

/* C/C++ Includes */
#include <stdint.h>
#include <stdlib.h>

/* Eigen Includes */
#include "Eigen\Eigen"


struct IMUData_t
{
	float ax, ay, az;	/* Accelerometer (X,Y,Z) */
	float gx, gy, gz;	/* Gyroscope	 (X,Y,Z) */
	float mx, my, mz;	/* Magnetometer  (X,Y,Z) */
	float mSTime;		/* Time stamp in mS */
};


struct AHRSData_t
{
	AHRSData_t()
	{
		eulerAngles.setZero();
		accel.setZero();
		gyro.setZero();
		mag.setZero();
	}
	
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
	
	
	const float& pitch() { return this->eulerAngles(0); }
	const float& roll() { return this->eulerAngles(1); }
	const float& yaw() { return this->eulerAngles(2); }
	
	const float& ax() { return this->accel(0); }
	const float& ay() { return this->accel(1); }
	const float& az() { return this->accel(2); }
	
	const float& gx() { return this->gyro(0); }
	const float& gy() { return this->gyro(1); }
	const float& gz() { return this->gyro(2); }
	
	const float& mx() { return this->mag(0); }
	const float& my() { return this->mag(1); }
	const float& mz() { return this->mag(2); }
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


/*------------------------------
* SD Card 
*-----------------------------*/
#pragma pack(push,1)
struct SD_LOG_AHRS_t
{	
	/* Degrees */
	float euler_deg_pitch = 0.0;
	float euler_deg_roll = 0.0;
	float euler_deg_yaw = 0.0;
	
	/* m/s^2 */
	float accel_x = 0.0;
	float accel_y = 0.0;
	float accel_z = 0.0;
	
	/* dps */
	float gyro_x = 0.0;
	float gyro_y = 0.0;
	float gyro_z = 0.0;
	
	/* gauss */
	float mag_x = 0.0;
	float mag_y = 0.0;
	float mag_z = 0.0;
};
#pragma pack(pop)


enum SDCardInstructions
{
	SD_CARD_SHUTDOWN = 1
	//OTHERS?
};

enum LEDInstructions
{
	//Select which led to apply actions to
	LED_BLUE		= (1u << 31),
	LED_YELLOW		= (1u << 30),
	LED_RED			= (1u << 29),
	
	//Available actions for each led
	LED_STATIC_ON	= (1U << 0),
	LED_STATIC_OFF	= (1U << 1),
	LED_FLASH_SLOW	= (1U << 2),
	LED_FLASH_MED	= (1U << 3),
	LED_FLASH_FAST	= (1U << 4)
	
};


#endif 
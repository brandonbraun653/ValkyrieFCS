#pragma once
#ifndef FCSCONFIG_HPP_
#define FCSCONFIG_HPP_

/*-----------------------------
 * Task Priorities
 * 5: highest
 * 1: lowest
 *----------------------------*/
#define AHRS_UPDATE_PRIORITY		4
#define RADIO_UPDATE_PRIORITY		4
#define MOTOR_UPDATE_PRIORITY		4
#define CTRL_UPDATE_PRIORITY		4
#define STATUS_LEDS_PRIORITY		1
#define SDCARD_LOGGING_PRIORITY		3
#define CONSOLE_LOGGING_PRIORITY	1
#define BLUETOOTH_PRIORITY			2

/*-----------------------------
 * Task Update Frequencies/Constants
 *----------------------------*/
#define STATUS_LED_UPDATE_FREQ_HZ	5		
#define CONSOLE_UPDATE_FREQ_HZ		5		
#define CTRL_UPDATE_FREQ_HZ			100	
#define SENSOR_UPDATE_FREQ_HZ		100		
#define SDCARD_UPDATE_FREQ_HZ		100
#define AHRS_UPDATE_RATE_MULTIPLIER	5		/* AHRS will have an effective update at X multiple of SENSOR_UPDATE_FREQ_HZ (x5) */

/*-----------------------------
 * Memory Management 
 *----------------------------*/
#define QUEUE_MINIMUM_SIZE			5

/*-----------------------------
 * Runtime Options 
 *----------------------------*/
#define USING_PID_CONTROL 0
#define USING_LQR_CONTROL !(USING_PID_CONTROL)

#define ESC_MIN_THROTTLE 1060
#define ESC_MAX_THROTTLE 1860
#define ESC_DELTA_THROTTLE (ESC_MAX_THROTTLE - ESC_MIN_THROTTLE)

//TODO: Put these settings as something in the app
#define MOTOR_KV 2350
#define BATTERY_VOLTAGE 12.5
#define MOTOR_MAX_RPM_RAD ((MOTOR_KV*BATTERY_VOLTAGE) / 9.5492965964254f)
#define MOTOR_MIN_RPM_RAD 0

#endif /* FCSCONFIG_HPP_ */
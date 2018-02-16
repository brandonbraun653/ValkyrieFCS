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
#define PID_UPDATE_PRIORITY			4
#define STATUS_LEDS_PRIORITY		1
#define SDCARD_LOGGING_PRIORITY		3
#define CONSOLE_LOGGING_PRIORITY	1
#define BLUETOOTH_PRIORITY			1

/*-----------------------------
 * Task Update Frequencies/Constants
 *----------------------------*/
#define STATUS_LED_UPDATE_FREQ_HZ	5		
#define CONSOLE_UPDATE_FREQ_HZ		5		
#define PID_UPDATE_FREQ_HZ			450		
#define SENSOR_UPDATE_FREQ_HZ		500			
#define AHRS_UPDATE_RATE_MULTIPLIER	5		/* AHRS will have an effective update at X multiple of SENSOR_UPDATE_FREQ_HZ (x5) */

/*-----------------------------
 * Memory Management 
 *----------------------------*/
#define QUEUE_MINIMUM_SIZE			5






#endif /* FCSCONFIG_HPP_ */
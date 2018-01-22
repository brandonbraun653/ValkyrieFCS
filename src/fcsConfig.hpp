#pragma once
#ifndef FCSCONFIG_HPP_
#define FCSCONFIG_HPP_

/*-----------------------------
 * Task Priorities
 * 5: highest
 * 1: lowest
 *----------------------------*/
#define AHRS_UPDATE_PRIORITY		5
#define SENSOR_UPDATE_PRIORITY		4
#define RADIO_UPDATE_PRIORITY		3
#define MOTOR_UPDATE_PRIORITY		3
#define STATUS_LEDS_PRIORITY		1
#define SDCARD_LOGGING_PRIORITY		1
#define CONSOLE_LOGGING_PRIORITY	1
#define BLUETOOTH_PRIORITY			1

/*-----------------------------
 * Task Update Frequencies
 *----------------------------*/
#define STATUS_LED_UPDATE_FREQ_HZ	5		
#define CONSOLE_UPDATE_FREQ_HZ		5		
#define MOTOR_UPDATE_FREQ_HZ		100
#define SENSOR_UPDATE_FREQ_HZ		250		
#define AHRS_UPDATE_FREQ_HZ			750		


#endif
#pragma once
#ifndef ESC_HPP_
#define ESC_HPP_

#include <Thor/include/thor.hpp>
#include <Thor/include/gpio.hpp>
#include <Thor/include/timer.hpp>

//struct ESCQuad_Init
//{
//	GPIOClass_sPtr motor1;
//	GPIOClass_sPtr motor2; 
//	GPIOClass_sPtr motor3; 
//	GPIOClass_sPtr motor4;
//	
//	int motor1_timer_channel = -1;
//	int motor2_timer_channel = -1;
//	int motor3_timer_channel = -1;
//	int motor4_timer_channel = -1;
//	
//	float outputFrequency = 0;
//
//	uint16_t armCMD_uS = 0;
//	uint16_t minThrottleCMD_uS = 0;
//	uint16_t maxThrottleCMD_uS = 0;
//};
//
//class ESCQuad
//{
//public:
//	void initialize(const ESCQuad_Init& init);
//	void throttleCalibration();
//	int limit(int command);
//	
//	void arm();
//	void disarm();
//	
//	bool isArmed();
//
//	void updateSetPoint(int motor, uint16_t value);
//	void updateSetPoint(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);
//
//	
//	ESCQuad();
//	~ESCQuad() = default;
//
//private:
//	void initGPIO();
//	void initTIMER();
//	
//	void enableMotorsOutput();
//	void disableMotorsOutput();
//	
//	bool armed_state = false;
//
//	int timer_channel;
//	uint32_t timerSrcClkFreq;
//	
//	uint16_t motor_arm_uS = 0;
//	uint16_t motor_idle_uS = 0;
//	uint16_t motor_min_pwr_uS = 0;
//	uint16_t motor_max_pwr_uS = 0;
//	
//	float outputFreq = 55.0;				//Hz
//	const float samplingFreq = 1000000.0;	//Hz
//	
//	TIM_HandleTypeDef hTimer;
//	TIM_OC_InitTypeDef ocInit;
//	
//	GPIOClass_sPtr m1;
//	GPIOClass_sPtr m2; 
//	GPIOClass_sPtr m3; 
//	GPIOClass_sPtr m4;
//	int motor_timer_channel[5];	//Size of 5 to account for 0 index offsets 
//};

#endif /*! PWM_H_ */
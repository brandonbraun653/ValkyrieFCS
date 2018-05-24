

#include <Thor/include/config.hpp>
#include <Thor/include/definitions.hpp>

#ifdef TARGET_STM32F7
#include <stm32f7xx_hal.h>
#endif

#ifdef TARGET_STM32F4
#include <stm32f4xx_hal.h>
#include <stm32f4xx_ll_rcc.h>
#endif

#ifdef USING_FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#endif

#include <ValkyrieFCS/include/esc_quad.hpp>


//const int armDelay = 3000;				/* Must be greater than 2 seconds */
//const int calibrationDelay = 5000;
//
//ESCQuad::ESCQuad()
//{
//	timer_channel = 4;	/* For now, hard code to use timer4*/
//}
//
//void ESCQuad::initialize(const ESCQuad_Init& init)
//{	
//	m1 = init.motor1;	motor_timer_channel[1] = init.motor1_timer_channel;
//	m2 = init.motor2;	motor_timer_channel[2] = init.motor2_timer_channel;
//	m3 = init.motor3;	motor_timer_channel[3] = init.motor3_timer_channel;
//	m4 = init.motor4;	motor_timer_channel[4] = init.motor4_timer_channel;
//	
//
//	outputFreq = init.outputFrequency;
//	motor_arm_uS = init.armCMD_uS;
//	motor_idle_uS = init.armCMD_uS + 50;
//	motor_min_pwr_uS = init.minThrottleCMD_uS;
//	motor_max_pwr_uS = init.maxThrottleCMD_uS;
//	
//	initGPIO();
//	initTIMER();
//	
//	#ifdef USING_FREERTOS
//	vTaskDelay(pdMS_TO_TICKS(250));
//	#else
//	HAL_Delay(250);
//	#endif
//}
//
//void ESCQuad::throttleCalibration()
//{
//	updateSetPoint(motor_max_pwr_uS, motor_max_pwr_uS, motor_max_pwr_uS, motor_max_pwr_uS);
//	enableMotorsOutput();
//	
//	#ifdef USING_FREERTOS
//	vTaskDelay(pdMS_TO_TICKS(calibrationDelay));
//	#else
//	HAL_Delay(calibrationDelay);
//	#endif
//	
//	updateSetPoint(motor_min_pwr_uS, motor_min_pwr_uS, motor_min_pwr_uS, motor_min_pwr_uS);
//	
//	#ifdef USING_FREERTOS
//	vTaskDelay(pdMS_TO_TICKS(calibrationDelay));
//	#else
//	HAL_Delay(calibrationDelay);
//	#endif
//	
//	disableMotorsOutput();
//}
//
//int ESCQuad::limit(int command)
//{
//	if (command < motor_idle_uS)
//		command = motor_idle_uS;
//	else if (command > motor_max_pwr_uS)
//		command = motor_max_pwr_uS;
//	
//	return command;
//}
//
//void ESCQuad::arm()
//{
//	updateSetPoint(motor_arm_uS, motor_arm_uS, motor_arm_uS, motor_arm_uS);
//	enableMotorsOutput();
//	
//	#ifdef USING_FREERTOS
//	vTaskDelay(pdMS_TO_TICKS(armDelay));
//	#else
//	HAL_Delay(armDelay);
//	#endif
//	
//	//TODO: Eventually put this block as a configurable setting for the user
//
//	//Set the motors to idle then wait a sec to allow AHRS to handle the noise input
//	updateSetPoint(motor_idle_uS, motor_idle_uS, motor_idle_uS, motor_idle_uS);
//
//	#ifdef USING_FREERTOS
//	vTaskDelay(pdMS_TO_TICKS(500));
//	#else
//	HAL_Delay(2000);
//	#endif
//
//	armed_state = true;
//}
//
//void ESCQuad::disarm()
//{
//	updateSetPoint(motor_min_pwr_uS, motor_min_pwr_uS, motor_min_pwr_uS, motor_min_pwr_uS);
//	disableMotorsOutput();
//}
//
//bool ESCQuad::isArmed()
//{
//	return armed_state;
//}
//
//void ESCQuad::updateSetPoint(int motor, uint16_t value)
//{
//	/* Because the timer clock frequency is 1MHz, "value" represents exactly x-uS.
//	 * No conversions necessary */
//	switch (motor_timer_channel[motor])
//	{
//	case TIM_CHANNEL_1:
//		hTimer.Instance->CCR1 = value;
//		break;
//		
//	case TIM_CHANNEL_2:
//		hTimer.Instance->CCR2 = value;
//		break;
//		
//	case TIM_CHANNEL_3:
//		hTimer.Instance->CCR3 = value;
//		break;
//		
//	case TIM_CHANNEL_4:
//		hTimer.Instance->CCR4 = value;
//		break;
//	}
//}
//
//void ESCQuad::updateSetPoint(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4)
//{
//	updateSetPoint(1, m1);
//	updateSetPoint(2, m2);
//	updateSetPoint(3, m3);
//	updateSetPoint(4, m4);
//}
//
//
////TODO: Correct GPIO refs here
//void ESCQuad::initGPIO()
//{
////	m1->mode(ALT_PP, PULLDN);	
////	m2->mode(ALT_PP, PULLDN);
////	m3->mode(ALT_PP, PULLDN);
////	m4->mode(ALT_PP, PULLDN);
//}
//
//void ESCQuad::initTIMER()
//{
//	/*  Get timer clock source frequency */
//	#if defined(STM32F446xx) || defined(STM32F767xx)
//	uint32_t periphBaseMask = 0xFFFF0000;
//	
//	LL_RCC_ClocksTypeDef currentClockConfig;
//	LL_RCC_GetSystemClocksFreq(&currentClockConfig);
//	
//	//TODO: Re-add the timer functionality
////	if ((periphBaseMask & timerBaseAddresses[timer_channel]) == APB1PERIPH_BASE) 
////		timerSrcClkFreq = 2*currentClockConfig.PCLK1_Frequency;
////	
////	if ((periphBaseMask & timerBaseAddresses[timer_channel]) == APB2PERIPH_BASE)
////		timerSrcClkFreq = 2*currentClockConfig.PCLK2_Frequency;
////	#endif
//	
//	/* Configure basic parameters */
////	hTimer.Instance = (TIM_TypeDef*)timerBaseAddresses[timer_channel];
//	
//	hTimer.Init.Prescaler = 0xFFFFu;
//	hTimer.Init.Period = 0xFFFFu;
//	hTimer.Init.CounterMode = TIM_COUNTERMODE_UP;
//	hTimer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//	
//	if (hTimer.Instance == TIM1 || hTimer.Instance == TIM8)
//		hTimer.Init.RepetitionCounter = 0u;
//	
//	
//	ocInit.OCMode = TIM_OCMODE_PWM1;
//	ocInit.Pulse = 0xFFFFu;
//	ocInit.OCPolarity = TIM_OCPOLARITY_HIGH;
//	ocInit.OCNPolarity = TIM_OCNPOLARITY_LOW;
//	ocInit.OCFastMode = TIM_OCFAST_DISABLE;
//	ocInit.OCIdleState = TIM_OCIDLESTATE_RESET;
//	ocInit.OCNIdleState = TIM_OCNIDLESTATE_RESET;
//	ocInit.Pulse = 2000;
//	
//	hTimer.Init.Prescaler = (uint16_t)(timerSrcClkFreq / samplingFreq); /* Get correct tick */
//	hTimer.Init.Period = (uint16_t)(samplingFreq / outputFreq); /* Assign ARR Reg for correct Freq */
//	
//	/* Configure into PWM mode */
////	TIMER_EnableClock(timer_channel);
//	HAL_TIM_PWM_Init(&hTimer);
//	HAL_TIM_PWM_ConfigChannel(&hTimer, &ocInit, TIM_CHANNEL_1);
//	HAL_TIM_PWM_ConfigChannel(&hTimer, &ocInit, TIM_CHANNEL_2);
//	HAL_TIM_PWM_ConfigChannel(&hTimer, &ocInit, TIM_CHANNEL_3);
//	HAL_TIM_PWM_ConfigChannel(&hTimer, &ocInit, TIM_CHANNEL_4);	
//	
//	/* Enable the preload feature so that we don't get jittery updates. See pg. 574:
//	 * http://www.st.com/content/ccc/resource/technical/document/reference_manual/4d/ed/bc/89/b5/70/40/dc/DM00135183.pdf/files/DM00135183.pdf/jcr:content/translations/en.DM00135183.pdf 
//	 * 
//	 * TODO: Add as member function when eventually transitioned over to TIMER class */
//	hTimer.Instance->CCMR1 |= (TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE);
//	hTimer.Instance->CCMR2 |= (TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE);
//}
//
//void ESCQuad::enableMotorsOutput()
//{
//	HAL_TIM_PWM_Start(&hTimer, TIM_CHANNEL_1);
//	HAL_TIM_PWM_Start(&hTimer, TIM_CHANNEL_2);
//	HAL_TIM_PWM_Start(&hTimer, TIM_CHANNEL_3);
//	HAL_TIM_PWM_Start(&hTimer, TIM_CHANNEL_4);
//}
//
//void ESCQuad::disableMotorsOutput()
//{
//	HAL_TIM_PWM_Stop(&hTimer, TIM_CHANNEL_1);
//	HAL_TIM_PWM_Stop(&hTimer, TIM_CHANNEL_2);
//	HAL_TIM_PWM_Stop(&hTimer, TIM_CHANNEL_3);
//	HAL_TIM_PWM_Stop(&hTimer, TIM_CHANNEL_4);
//}
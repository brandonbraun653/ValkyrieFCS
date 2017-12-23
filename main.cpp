#include <stm32f7xx_hal.h>
#include <../CMSIS_RTOS/cmsis_os.h>
#include "thor.h"

osThreadId LEDThread1Handle, LEDThread2Handle;

/* Private function prototypes -----------------------------------------------*/
static void LED_Thread1(void const *argument);
static void LED_Thread2(void const *argument);

GPIOClass led1(GPIOB, PIN_7, ULTRA_SPD, NOALTERNATE);
GPIOClass led2(GPIOB, PIN_0, ULTRA_SPD, NOALTERNATE);

int main(void)
{
	HAL_Init();
	ThorSystemClockConfig();

	led1.mode(OUTPUT_PP);
	led2.mode(OUTPUT_PP);


	/* Thread 1 definition */
	osThreadDef(LED1, LED_Thread1, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);

	/*  Thread 2 definition */
	osThreadDef(LED2, LED_Thread2, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);

	/* Start thread 1 */
	LEDThread1Handle = osThreadCreate(osThread(LED1), NULL);

	/* Start thread 2 */
	LEDThread2Handle = osThreadCreate(osThread(LED2), NULL);

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	for (;;)
		;
}


static void LED_Thread1(void const *argument)
{
	(void)argument;

	for (;;)
	{
		led1.write(HIGH);
		osDelay(2000);

		led1.write(LOW);
		osThreadSuspend(LEDThread2Handle);
		osDelay(2000);

		osThreadResume(LEDThread2Handle);
	}
}

static void LED_Thread2(void const *argument)
{
	uint32_t count;
	(void)argument;

	for (;;)
	{
		led2.toggle();
		osDelay(200);
	}
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{
	while (1)
	{
	}
}
#endif

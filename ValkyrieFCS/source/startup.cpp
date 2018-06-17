#ifdef DEBUG
#include "SysprogsProfiler.h"
#endif

/* Chimera Includes */
#include <Chimera/threading.hpp>
#include <Chimera/gpio.hpp>

/* Thread Task Includes */
#include <ValkyrieFCS/include/fcsConfig.hpp>
#include <ValkyrieFCS/include/radio.hpp>
#include <ValkyrieFCS/include/sdcard.hpp>
#include <ValkyrieFCS/include/ahrs.hpp>
#include <ValkyrieFCS/include/motors.hpp>
#include <ValkyrieFCS/include/console.hpp>
#include <ValkyrieFCS/include/led.hpp>
#include <ValkyrieFCS/include/bluetooth.hpp>
#include <ValkyrieFCS/include/threads.hpp>
#include <ValkyrieFCS/include/pid.hpp>
#include <ValkyrieFCS/include/lqr.hpp>


using namespace Chimera::GPIO;
using namespace Chimera::Threading;

TaskHandle_t ledHandle;
TaskHandle_t sdCardHandle;
TaskHandle_t ctrlHandle;
TaskHandle_t motorHandle;
TaskHandle_t ahrsHandle;
TaskHandle_t radioHandle;

int main(void)
{
	ChimeraInit();
	
	#ifdef DEBUG
	InitializeSamplingProfiler();
	InitializeInstrumentingProfiler();
	#endif 

	/* Create a task and then wait until its initialization sequence has finished before continuing.
	 * The tasks MUST be created in this order:
	 * 1. LedStatus
	 * 2. SDCard
	 * 3. PID or LQR
	 * 4. Motor (This depends on [2,3] being started)
	 * 5. AHRS
	 * 
	 */
	addThread(FCS_LED::ledStatus, "ledTask", 350, NULL, STATUS_LEDS_PRIORITY, ledHandle);
	addThread(radioTask, "radio", 500, NULL, 2, radioHandle);
	//addThread(FCS_SD::sdCardTask, "sdTask", 350, NULL, SDCARD_LOGGING_PRIORITY, &sdCardHandle);
	//addThread(FCS_PID::pidTask, "pidTask", 350, NULL, CTRL_UPDATE_PRIORITY, &ctrlHandle);
	//addThread(FCS_MOTOR::motorTask, "motorTask", 350, NULL, MOTOR_UPDATE_PRIORITY, &motorHandle);
	//addThread(FCS_AHRS::ahrsTask, "ahrsTask", 8000, NULL, AHRS_UPDATE_PRIORITY, &ahrsHandle);

	startScheduler(true);
	
	/* We will never reach here as the scheduler should have taken over */
	for (;;)
	{
	}
}
	

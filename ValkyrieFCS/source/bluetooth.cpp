/* C/C++ Includes */
#include <stdint.h>
#include <stdlib.h>
#include <string>

/* Chimera Includes */
#include <Chimera/threading.hpp>
#include <Chimera/serial.hpp>
#include <Chimera/logging.hpp>

/* Project Includes */
#include <ValkyrieFCS/include/fcsConfig.hpp>
#include <ValkyrieFCS/include/bluetooth.hpp>
#include <ValkyrieFCS/include/fcMemory.hpp>
#include <ValkyrieFCS/include/appInterface.hpp>


using namespace Chimera::Serial;
using namespace Chimera::Threading;
using namespace Chimera::Logging;

namespace FCS
{
	void bluetoothTask(void* argument)
	{
		PID::PIDSettings roll, pitch, yaw;

		roll.kp = 1.0;
		roll.ki = 1.0;
		roll.kd = 1.0;

		pitch.kp = 2.0;
		pitch.ki = 2.0;
		pitch.kd = 2.0;

		yaw.kp = 3.0;
		yaw.ki = 3.0;
		yaw.kd = 3.0;

		fcSettings.initialize(Adesto::AT45DB081E, 1, 1000000);
		volatile FCSettings::Status errorCode = FCSettings::FLASH_OK;

		Console.log(Level::INFO, "BT Thread: Initialization Complete\r\n");
		signalThreadSetupComplete();
		Console.log(Level::INFO, "BT Thread: Running\r\n");

		TickType_t lastTimeWoken = xTaskGetTickCount();

		#ifdef DEBUG
		volatile UBaseType_t stackHighWaterMark_BT = uxTaskGetStackHighWaterMark(NULL);
		Console.log(Level::DBG, "BT Thread: Remaining stack size after init is %d bytes\r\n", stackHighWaterMark_BT);
		#endif
		for (;;)
		{
			errorCode = FCS::fcSettings.set(PID_TERMS_PITCH, pitch);

			Chimera::delayMilliseconds(1000);

			errorCode = FCS::fcSettings.get(PID_TERMS_PITCH, pitch);

			vTaskDelayUntil(&lastTimeWoken, pdMS_TO_TICKS(1000));
		}
	}
}

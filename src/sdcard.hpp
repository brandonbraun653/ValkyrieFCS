#pragma once
#ifndef SDCARD_HPP_
#define SDCARD_HPP_

/* C/C++ Includes */
#include <stdint.h>
#include <stdlib.h>

/* Thor Includes */
#include "include/thor.h"
#include "include/spi.h"
#include "include/gpio.h"

/* External Library Includes */
#include "sd.h"

/* Boost Includes */
#include <boost/smart_ptr.hpp>
#include <boost/make_shared.hpp>

/* FreeRTOS Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "dataTypes.hpp"
#include "messaging.hpp"


extern void sdCardTask(void* argument);


#endif /* SDCARD_HPP_ */
#pragma once
#ifndef LED_HPP_
#define LED_HPP_
/* C/C++ Includes */
#include <stdint.h>
#include <stdlib.h>

/* Thor Includes */
#include "include/thor.h"
#include "include/gpio.h"
#include "include/exti.h"
#include "include/interrupt.h"

/* Boost Includes */
#include <boost/smart_ptr.hpp>
#include <boost/make_shared.hpp>

/* FreeRTOS Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "dataTypes.hpp"
#include "threading.hpp"

extern void ledStatus(void* argument);

#endif
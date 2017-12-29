#pragma once
#ifndef CONSOLE_HPP_
#define CONSOLE_HPP_

/* C/C++ Includes */
#include <stdint.h>
#include <stdlib.h>

/* Thor Includes */
#include "include/thor.h"
#include "include/uart.h"
#include "include/exti.h"
#include "include/interrupt.h"

/* Boost Includes */
#include <boost/smart_ptr.hpp>
#include <boost/make_shared.hpp>

/* FreeRTOS Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

extern void consoleTask(void* argument);

#endif /* CONSOLE_HPP_ */
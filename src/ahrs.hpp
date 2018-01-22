#pragma once
#ifndef AHRS_HPP_
#define AHRS_HPP_

/* C/C++ Includes */
#include <stdint.h>
#include <stdlib.h>

/* Thor Includes */
#include "include/thor.h"
#include "include/spi.h"
#include "include/gpio.h"

/* Boost Includes */
#include <boost/smart_ptr.hpp>
#include <boost/make_shared.hpp>

/* FreeRTOS Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* AHRS Includes */
#include "LSM9DS1.hpp"
#include "dataTypes.hpp"
#include "messaging.hpp"


extern void ahrsTask(void* argument);

#endif /* AHRS_HPP_ */
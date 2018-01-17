#pragma once
#ifndef RADIO_HPP_
#define RADIO_HPP_

/* FreeRTOS Includes */
#include "queue.h"

/*------------------------------
* Queues for holding Radio I/O
*-----------------------------*/
extern QueueHandle_t qRadio_Command;		/* Command Inputs */
extern QueueHandle_t qRadio_Control;		/* Stick Inputs (Throttle, Roll, etc) */
extern QueueHandle_t qRadio_Request;		/* Request Inputs */

/*------------------------------
* Radio Thread
*-----------------------------*/
extern void radioTask(void* argument);


#endif /* RADIO_HPP_ */
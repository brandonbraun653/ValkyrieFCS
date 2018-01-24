#pragma once
#ifndef SDCARD_HPP_
#define SDCARD_HPP_

#include <stdint.h>


enum SDCardInstructions
{
	SD_CARD_SHUTDOWN = 1
	//OTHERS?
};

extern void sdCardTask(void* argument);


#endif /* SDCARD_HPP_ */
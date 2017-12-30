#include "messaging.hpp"

QueueHandle_t qIMUFlightData = xQueueCreate(10, sizeof(IMUData_t));

QueueHandle_t qCommandBuffer = xQueueCreate(10, sizeof(CMDData_t));

SemaphoreHandle_t sLedCmdTrigger = xSemaphoreCreateCounting(5, 0);
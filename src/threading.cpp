#include "threading.hpp"

QueueHandle_t qIMURawData = xQueueCreate(QUEUE_MINIMUM_SIZE, sizeof(IMUData_t));

QueueHandle_t qCommandBuffer = xQueueCreate(QUEUE_MINIMUM_SIZE, sizeof(CMDData_t));


boost::container::vector<TaskHandle_t> TaskHandle(TOTAL_TASK_SIZE);



BaseType_t xSendTaskMessage(TaskIndex idx, uint32_t msg)
{
	return xTaskNotify(TaskHandle[idx], msg, eSetValueWithOverwrite);
}

BaseType_t xSendTaskMessageFromISR(TaskIndex idx, uint32_t msg)
{
	return xTaskNotifyFromISR(TaskHandle[idx], msg, eSetValueWithOverwrite, NULL);
}
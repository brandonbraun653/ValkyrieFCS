#include "threading.hpp"

/*----------------------------------
* Queues
*----------------------------------*/
QueueHandle_t qIMU = xQueueCreate(QUEUE_MINIMUM_SIZE, sizeof(IMUData_t));

QueueHandle_t qAHRS = xQueueCreate(1, sizeof(AHRSData_t));

QueueHandle_t qPID = xQueueCreate(1, sizeof(PIDData_t));


QueueHandle_t qSD_AHRSData = xQueueCreate(QUEUE_MINIMUM_SIZE, sizeof(AHRSData_t));


QueueHandle_t qCommandBuffer = xQueueCreate(QUEUE_MINIMUM_SIZE, sizeof(CMDData_t));


/*----------------------------------
* Semaphores and Mutexes
*----------------------------------*/
SemaphoreHandle_t ahrsBufferMutex = xSemaphoreCreateMutex();
SemaphoreHandle_t pidBufferMutex = xSemaphoreCreateMutex();

SemaphoreHandle_t sdBufferMutex = xSemaphoreCreateMutex();

boost::container::vector<TaskHandle_t> TaskHandle(TOTAL_TASK_SIZE);
TaskIndex activeTask;


BaseType_t xTaskSendMessage(TaskIndex idx, uint32_t msg)
{
	return xTaskNotify(TaskHandle[idx], msg, eSetValueWithOverwrite);
}

BaseType_t xTaskSendMessageFromISR(TaskIndex idx, uint32_t msg)
{
	return xTaskNotifyFromISR(TaskHandle[idx], msg, eSetValueWithOverwrite, NULL);
}
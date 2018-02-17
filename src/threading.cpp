#include "threading.hpp"

/*----------------------------------
* Queues
*----------------------------------*/
/* INTERTASK*/
QueueHandle_t qIMU = xQueueCreate(QUEUE_MINIMUM_SIZE, sizeof(IMUData_t));
QueueHandle_t qAHRS = xQueueCreate(1, sizeof(AHRSData_t));
QueueHandle_t qPID = xQueueCreate(1, sizeof(PIDData_t));


/* SD CARD */
QueueHandle_t qSD_AHRSFull = xQueueCreate(QUEUE_MINIMUM_SIZE, sizeof(SDLOG_AHRS_Full_t));
QueueHandle_t qSD_AHRSMinimal = xQueueCreate(QUEUE_MINIMUM_SIZE, sizeof(SDLOG_AHRS_Minimal_t));
QueueHandle_t qSD_Motor = xQueueCreate(QUEUE_MINIMUM_SIZE, sizeof(SDLOG_Motors_t));



/*----------------------------------
* Semaphores and Mutexes
*----------------------------------*/
SemaphoreHandle_t ahrsBufferMutex = xSemaphoreCreateMutex();
SemaphoreHandle_t pidBufferMutex = xSemaphoreCreateMutex();



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
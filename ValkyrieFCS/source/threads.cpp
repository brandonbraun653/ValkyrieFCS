#include <ValkyrieFCS/include/threads.hpp>

/*----------------------------------
* Queues
*----------------------------------*/
/* INTERTASK*/
QueueHandle_t qIMU	= xQueueCreate(QUEUE_MINIMUM_SIZE, sizeof(IMUData_t));
QueueHandle_t qAHRS = xQueueCreate(1, sizeof(AHRSData_t));

#if USING_PID_CONTROL
QueueHandle_t qPID	= xQueueCreate(1, sizeof(PIDData_t));
#endif

#if USING_LQR_CONTROL
QueueHandle_t qLQR = xQueueCreate(1, sizeof(LQRData_t));
#endif


/* SD CARD */
QueueHandle_t qSD_AHRSFull			= xQueueCreate(QUEUE_MINIMUM_SIZE, sizeof(SDLOG_AHRS_Full_t));
QueueHandle_t qSD_AHRSMinimal		= xQueueCreate(QUEUE_MINIMUM_SIZE, sizeof(SDLOG_AHRS_Minimal_t));
QueueHandle_t qSD_Motor				= xQueueCreate(QUEUE_MINIMUM_SIZE, sizeof(SDLOG_Motors_t));
QueueHandle_t qSD_PIDAngleSetpoint	= xQueueCreate(QUEUE_MINIMUM_SIZE, sizeof(SDLOG_PIDAngleInput_t));
QueueHandle_t qSD_PIDRateSetpoint	= xQueueCreate(QUEUE_MINIMUM_SIZE, sizeof(SDLOG_PIDRateInput_t));

/*----------------------------------
* Semaphores and Mutexes
*----------------------------------*/
SemaphoreHandle_t ahrsBufferMutex = xSemaphoreCreateMutex();
SemaphoreHandle_t pidBufferMutex = xSemaphoreCreateMutex();
SemaphoreHandle_t lqrBufferMutex = xSemaphoreCreateMutex();

TaskIndex activeTask;

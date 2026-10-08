#ifndef APP_RTOS_H
#define APP_RTOS_H

#include "cmsis_os2.h"

extern osEventFlagsId_t SystemEventsHandle;

extern osMessageQueueId_t DriveCmdQueueHandle;
extern osMessageQueueId_t UsbRxQueueHandle;
extern osMessageQueueId_t TelemetryQueueHandle;

#endif

#include "debug_task.h"
#include "topics.h"

osThreadId_t Debug_TaskHandle;


void Debug_Task(void *argument) {
  portTickType currentTime;
  currentTime = xTaskGetTickCount();
  for (;;) {

    vTaskDelayUntil(&currentTime, 5);
    // vTaskDelay(5);
  }
}
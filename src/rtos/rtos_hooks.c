#include "FreeRTOS.h"
#include "task.h"

void vApplicationTickHook() {}
void vApplicationMallocFailedHook() {
    while (1)
    ;
}

void vApplicationIdleHook() {}
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
  while (1)
    ;
}

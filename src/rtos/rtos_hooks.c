#include "FreeRTOS.h"
#include "task.h"

// Idle task
static StaticTask_t g_idleTaskTCB;
static StackType_t g_idleTaskStack[96];

// Timer task
static StaticTask_t g_timerTaskTCB;
static StackType_t g_timerTaskStack[256];

void vApplicationMallocFailedHook() {
	while (1);
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
	while (1);
}

#if (configSUPPORT_STATIC_ALLOCATION == 1)

    void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize) {
        *ppxIdleTaskTCBBuffer = &g_idleTaskTCB;
        *ppxIdleTaskStackBuffer = g_idleTaskStack;
        *pulIdleTaskStackSize = 96;
    }

    void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize) {
        *ppxTimerTaskTCBBuffer = &g_timerTaskTCB;
        *ppxTimerTaskStackBuffer = g_timerTaskStack;
        *pulTimerTaskStackSize = 256;
    }

#endif

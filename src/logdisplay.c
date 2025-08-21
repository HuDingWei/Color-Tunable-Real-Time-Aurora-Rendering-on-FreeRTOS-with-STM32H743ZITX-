/*
 * logdisplay.c
 *
 *  Created on: Apr 16, 2025
 *      Author: Hu Ding-Wei
 */


#include "logdisplay.h"

#include <leguan.h>

#include <FreeRTOS.h>                   /* Main FreeRTOS header               */
#include <task.h>						/* FreeRTOS Tasks					  */
#include <queue.h>						/* FreeRTOS Message-Queues			  */
#include <semphr.h>						/* FreeRTOS Semaphores				  */
#include <timers.h>						/* FreeRTOS Timers					  */

void taskDisplay(void *pvParameters) {
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000); // 每 1000ms 執行一次
    uint32_t buttonEvent;

	for (;;) {
        // 使用信號量保護 systemState 和 auroraParams
		if (systemState.log){
//        if (xSemaphoreTake(paramsMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
			taskENTER_CRITICAL();
			const char* modeStr;
			switch (systemState.rgbMode) {
				case 0: modeStr = "None"; break;
				case 1: modeStr = "Red"; break;
				case 2: modeStr = "Green"; break;
				case 3: modeStr = "Blue"; break;
				default: modeStr = "Unknown"; break;
			}

			LOG_Info("Mode: %s | Auto: %s | Counter: %u | RGB: (%d, %d, %d) | Range: (%d, %d, %d)",
					 modeStr, systemState.autoMode ? "On" : "Off", systemState.clickCounter,
					 auroraParams.baseR, auroraParams.baseG, auroraParams.baseB,
					 auroraParams.rangeR, auroraParams.rangeG, auroraParams.rangeB);
			LOG_Info("Temp: %.1fC | Hum: %.1f%% | Lux: %.1f",
					 systemState.temperature, systemState.humidity, systemState.brightness);
			taskEXIT_CRITICAL();
//            xSemaphoreGive(paramsMutex);

//        }
//        else {
//            LOG_Warn("DisplayTask: Failed to take paramsMutex");
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
//        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}



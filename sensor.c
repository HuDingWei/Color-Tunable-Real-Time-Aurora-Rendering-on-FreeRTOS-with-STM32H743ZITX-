/*
 * sensor.c
 *
 *  Created on: Apr 15, 2025
 *      Author: Hu Ding-Wei
 */


#include "sensor.h"

#include <leguan.h>

#include <FreeRTOS.h>                   /* Main FreeRTOS header               */
#include <task.h>						/* FreeRTOS Tasks					  */
#include <queue.h>						/* FreeRTOS Message-Queues			  */
#include <semphr.h>						/* FreeRTOS Semaphores				  */
#include <timers.h>						/* FreeRTOS Timers					  */

void taskSensors(void *pvParameters) {
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(500); // 每 500ms 執行一次

	for (;;) {
        // 假設感測器問題已解決，使用默認值
        float humidity = 50.0;
        float temperature = 25.0;
        float brightness = 300.0;

//        // 使用信號量保護 systemState
//        if (xSemaphoreTake(paramsMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
//        taskENTER_CRITICAL();
        if (R_SUCCESS(SENSOR_GetTemperature(&temperature))) {
        	systemState.humidity = humidity;
        } else {
            LOG_Info("SENSOR_GetHumidity Failed");
        }

        if (R_SUCCESS(SENSOR_GetTemperature(&temperature))) {
        	systemState.temperature = temperature;
        } else {
            LOG_Info("SENSOR_GetTemperature Failed");
        }
//		systemState.brightness = brightness;
//		xSemaphoreGive(paramsMutex);
//        } else {
//            LOG_Warn("SensorsTask: Failed to take paramsMutex");
//        }
//		taskEXIT_CRITICAL();
		vTaskDelay(50 / portTICK_PERIOD_MS);
//        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

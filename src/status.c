/*
 * status.c
 *
 *  Created on: Apr 16, 2025
 *      Author: Hu Ding-Wei
 */


#include "status.h"

#include <leguan.h>

#include <FreeRTOS.h>                   /* Main FreeRTOS header               */
#include <task.h>						/* FreeRTOS Tasks					  */
#include <queue.h>						/* FreeRTOS Message-Queues			  */
#include <semphr.h>						/* FreeRTOS Semaphores				  */
#include <timers.h>						/* FreeRTOS Timers					  */

// Define global variables
//extern SystemState systemState;
//extern AuroraParams auroraParams;
//extern TaskHandle_t statusTaskHandle;
//extern SemaphoreHandle_t paramsMutex;

void taskStatus(void *pvParameters) {
    (void)pvParameters;
    uint32_t buttonEvent;

    while (1) {
        // Wait for button event notification
        if (xTaskNotifyWait(0, 0xFFFFFFFF, &buttonEvent, portMAX_DELAY) == pdTRUE) {
            // Update systemState using critical section for simple fields
            taskENTER_CRITICAL();
            if (buttonEvent & (1 << Button0)) {
            	systemState.rgbMode = (systemState.rgbMode + 1) % 4;
                systemState.autoMode = false;
                systemState.clickCounter++;
            }
            if (buttonEvent & (1 << Button2)) {
                systemState.autoMode = !systemState.autoMode;
                systemState.clickCounter++;
            }
            if (buttonEvent & (1 << Button3)) {
                systemState.log = !systemState.log;
                systemState.clickCounter++;
            }
            taskEXIT_CRITICAL();

            // Update auroraParams using mutex
            if (systemState.autoMode == false) {
//            if (xSemaphoreTake(paramsMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
//            if (xSemaphoreTake(paramsMutex, pdMS_TO_TICKS(50)) != NULL) {
                int step = 5;
                if (buttonEvent & (1 << JoystickNorth)) {
                    switch (systemState.rgbMode) {
                        case 1:
                            auroraParams.baseR = (auroraParams.baseR + step) > 255 ? 255 : (auroraParams.baseR + step);
                            auroraParams.rangeR = (auroraParams.rangeR + step) > 255 ? 255 : (auroraParams.rangeR + step);
                            break;
                        case 2:
                            auroraParams.baseG = (auroraParams.baseG + step) > 255 ? 255 : (auroraParams.baseG + step);
                            auroraParams.rangeG = (auroraParams.rangeG + step) > 255 ? 255 : (auroraParams.rangeG + step);
                            break;
                        case 3:
                            auroraParams.baseB = (auroraParams.baseB + step) > 255 ? 255 : (auroraParams.baseB + step);
                            auroraParams.rangeB = (auroraParams.rangeB + step) > 255 ? 255 : (auroraParams.rangeB + step);
                            break;
                    }
                }
                if (buttonEvent & (1 << JoystickSouth)) {
                    switch (systemState.rgbMode) {
                        case 1:
                            auroraParams.baseR = (auroraParams.baseR - step) < 0 ? 0 : (auroraParams.baseR - step);
                            auroraParams.rangeR = (auroraParams.rangeR - step) < -255 ? -255 : (auroraParams.rangeR - step);
                            break;
                        case 2:
                            auroraParams.baseG = (auroraParams.baseG - step) < 0 ? 0 : (auroraParams.baseG - step);
                            auroraParams.rangeG = (auroraParams.rangeG - step) < -255 ? -255 : (auroraParams.rangeG - step);
                            break;
                        case 3:
                            auroraParams.baseB = (auroraParams.baseB - step) < 0 ? 0 : (auroraParams.baseB - step);
                            auroraParams.rangeB = (auroraParams.rangeB - step) < -255 ? -255 : (auroraParams.rangeB - step);
                            break;
                    }
                }
            }
			// Auto mode adjustments
            else if (systemState.autoMode == true) {
				if (systemState.temperature > 25) {
					auroraParams.baseR = 20;
					auroraParams.rangeR = -10;
					auroraParams.baseG = 60;
					auroraParams.rangeG = 20;
					auroraParams.baseB = 120;
					auroraParams.rangeB = 40;
				} else if (systemState.temperature < 10) {
					auroraParams.baseR = 120;
					auroraParams.rangeR = 40;
					auroraParams.baseG = 60;
					auroraParams.rangeG = 20;
					auroraParams.baseB = 20;
					auroraParams.rangeB = -10;
				} else {
					auroraParams.baseR = 60;
					auroraParams.rangeR = -30;
					auroraParams.baseG = 20;
					auroraParams.rangeG = 10;
					auroraParams.baseB = 90;
					auroraParams.rangeB = 110;
				}

				if (systemState.humidity > 70) {
					auroraParams.baseB += 20;
					auroraParams.rangeB += 10;
					auroraParams.baseR -= 20;
					auroraParams.rangeR -= 10;
				} else if (systemState.humidity < 40) {
					auroraParams.baseR += 20;
					auroraParams.rangeR = 10;
					auroraParams.baseB -= 20;
					auroraParams.rangeB -= 10;
				}

				if (systemState.brightness > 500) {
					auroraParams.baseR = (auroraParams.baseR * 0.8 > 255) ? 255 : (auroraParams.baseR * 0.8);
					auroraParams.baseG = (auroraParams.baseG * 0.8 > 255) ? 255 : (auroraParams.baseG * 0.8);
					auroraParams.baseB = (auroraParams.baseB * 0.8 > 255) ? 255 : (auroraParams.baseB * 0.8);
				} else if (systemState.brightness < 200) {
					auroraParams.baseR = (auroraParams.baseR * 1.2 > 255) ? 255 : (auroraParams.baseR * 1.2);
					auroraParams.baseG = (auroraParams.baseG * 1.2 > 255) ? 255 : (auroraParams.baseG * 1.2);
					auroraParams.baseB = (auroraParams.baseB * 0.9 > 255) ? 255 : (auroraParams.baseB * 0.9);
				}

				auroraParams.baseR = auroraParams.baseR > 255 ? 255 : (auroraParams.baseR < 0 ? 0 : auroraParams.baseR);
				auroraParams.rangeR = auroraParams.rangeR > 255 ? 255 : (auroraParams.rangeR < -255 ? -255 : auroraParams.rangeR);
				auroraParams.baseG = auroraParams.baseG > 255 ? 255 : (auroraParams.baseG < 0 ? 0 : auroraParams.baseG);
				auroraParams.rangeG = auroraParams.rangeG > 255 ? 255 : (auroraParams.rangeG < -255 ? -255 : auroraParams.rangeG);
				auroraParams.baseB = auroraParams.baseB > 255 ? 255 : (auroraParams.baseB < 0 ? 0 : auroraParams.baseB);
				auroraParams.rangeB = auroraParams.rangeB > 255 ? 255 : (auroraParams.rangeB < -255 ? -255 : auroraParams.rangeB);
			}

//                xSemaphoreGive(paramsMutex);
            } else {
                LOG_Warn("Failed to take auroraMutex");
            }
//        }
    }
}



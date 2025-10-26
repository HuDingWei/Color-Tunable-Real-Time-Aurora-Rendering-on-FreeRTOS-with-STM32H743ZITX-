/*
 * button.c
 *
 *  Created on: Apr 15, 2025
 *      Author: Hu Ding-Wei
 */


#include "button.h"

#include <leguan.h>

#include <FreeRTOS.h>                   /* Main FreeRTOS header               */
#include <task.h>						/* FreeRTOS Tasks					  */
#include <queue.h>						/* FreeRTOS Message-Queues			  */
#include <semphr.h>						/* FreeRTOS Semaphores				  */
#include <timers.h>						/* FreeRTOS Timers					  */

// Define global variables
extern SemaphoreHandle_t paramsMutex;
extern TaskHandle_t statusTaskHandle;
extern TaskHandle_t displayTaskHandle;
extern SystemState systemState;
extern AuroraParams auroraParams;


void taskButtons(void *pvParameters) {
    (void)pvParameters;
    uint8_t lastButtonState = 0;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t debounceDelay = pdMS_TO_TICKS(250); // 250ms debounce
    const TickType_t longPressInterval = pdMS_TO_TICKS(300); // 300ms for long press repeat
    TickType_t lastEventTime = 0;
    TickType_t lastLongPressTime = 0;
    bool joystickHeld = false;

    while (1) {
        uint8_t buttons = 0;
        for (size_t i = 0; i < 7; i++) {
            buttons |= FPGA_GetButton(i) ? (1 << i) : 0;
        }

        // Check if joystick is held (North or South)
        joystickHeld = (buttons & ((1 << JoystickNorth) | (1 << JoystickSouth))) != 0;

        // Detect button changes (debounced)
        if (buttons != lastButtonState) {
            TickType_t currentTime = xTaskGetTickCount();
            if (currentTime - lastEventTime >= debounceDelay) {
                // Notify status task for button state change
                xTaskNotify(statusTaskHandle, buttons, eSetBits);
                lastEventTime = currentTime;
                lastLongPressTime = currentTime;

//                // Check if joystick is held (North or South)
//                joystickHeld = (buttons & ((1 << JoystickNorth) | (1 << JoystickSouth))) != 0;
            }
        }
        // Handle long press for joystick
        else if (joystickHeld) {
            TickType_t currentTime = xTaskGetTickCount();
            if (currentTime - lastLongPressTime >= longPressInterval) {
                // Notify status task for repeated joystick action
                xTaskNotify(statusTaskHandle, buttons, eSetBits);
                lastLongPressTime = currentTime;
            }
        }

        lastButtonState = buttons;
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50)); // Poll every 50ms
    }
}



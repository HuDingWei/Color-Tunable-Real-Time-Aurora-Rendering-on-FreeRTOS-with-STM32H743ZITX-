/*
 * button.h
 *
 *  Created on: Apr 15, 2025
 *      Author: Hu Ding-Wei
 */

#ifndef BUTTON_H_
#define BUTTON_H_

#include <FreeRTOS.h>
#include <queue.h>
#include <semphr.h>
#include "types.h"

//extern SemaphoreHandle_t paramsMutex;
//extern TaskHandle_t statusTaskHandle;
//extern SystemState systemState;
//extern AuroraParams auroraParams;

void taskButtons(void *pvParameters);

#endif /* BUTTON_H_ */

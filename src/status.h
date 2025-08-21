/*
 * status.h
 *
 *  Created on: Apr 16, 2025
 *      Author: Hu Ding-Wei
 */

#ifndef STATUS_H_
#define STATUS_H_

#include <FreeRTOS.h>
#include <semphr.h>
#include <queue.h>
#include "types.h"

// External variables
extern SystemState systemState;
extern TaskHandle_t statusTaskHandle;
extern SemaphoreHandle_t paramsMutex;
extern AuroraParams auroraParams;

void taskStatus(void *pvParameters);

#endif /* STATUS_H_ */

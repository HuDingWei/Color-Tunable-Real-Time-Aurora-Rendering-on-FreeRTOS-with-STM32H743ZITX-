/*
 * aurora.h
 *
 *  Created on: Apr 14, 2025
 *      Author: Hu Ding-Wei
 */

#ifndef AURORA_H_
#define AURORA_H_

#include <FreeRTOS.h>
#include <semphr.h>
#include "types.h"

#define FRAME_WIDTH   800
#define FRAME_HEIGHT  480
#define RANDOM_MIN    100
#define RANDOM_MAX    999

//typedef struct AuroraParams AuroraParams;

//extern SemaphoreHandle_t paramsMutex;
//extern AuroraParams auroraParams;

void taskAurora(void *pvParameters);
void renderWaveEffect(AuroraParams *params);
float simpleNoise(float x, float time);

#endif /* AURORA_H_ */

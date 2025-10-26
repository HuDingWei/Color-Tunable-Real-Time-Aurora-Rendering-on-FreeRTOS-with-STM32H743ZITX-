/*
 * logdisplay.h
 *
 *  Created on: Apr 16, 2025
 *      Author: Hu Ding-Wei
 */

#ifndef LOGDISPLAY_H_
#define LOGDISPLAY_H_

#include <FreeRTOS.h>
#include <semphr.h>
#include "types.h"

//typedef struct {
//    float time;
//    float waveSpeed;
//    float breathSpeed;
//    int baseR;
//    int rangeR;
//    int baseG;
//    int rangeG;
//    int baseB;
//    int rangeB;
//} AuroraParams;
//
//typedef struct {
//    uint8_t buttonState;
//    uint8_t lastButtonState;
//    uint32_t lastDebounceTime;
//    uint8_t clickCounter;
//    uint8_t rgbMode;
//    uint8_t joystickUpState;
//    uint8_t joystickDownState;
//    float humidity;
//    float temperature;
//    float brightness;
//    bool autoMode;
//} SystemState;

extern SemaphoreHandle_t paramsMutex;
extern AuroraParams auroraParams;
extern SystemState systemState;

void taskDisplay(void *pvParameters);



#endif /* LOGDISPLAY_H_ */

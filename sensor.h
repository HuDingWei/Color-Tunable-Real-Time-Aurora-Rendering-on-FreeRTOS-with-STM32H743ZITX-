/*
 * sensor.h
 *
 *  Created on: Apr 15, 2025
 *      Author: Hu Ding-Wei
 */

#ifndef SENSOR_H_
#define SENSOR_H_

#include <FreeRTOS.h>
#include <semphr.h>
#include "types.h"

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
extern SystemState systemState;

void taskSensors(void *pvParameters);


#endif /* SENSOR_H_ */

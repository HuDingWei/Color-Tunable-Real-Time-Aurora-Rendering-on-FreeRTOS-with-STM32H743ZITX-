/*
 * types.h
 *
 *  Created on: Apr 16, 2025
 *      Author: Hu Ding-Wei
 */

#ifndef TYPES_H_
#define TYPES_H_

#include <cube.h>
#include <leguan.h>

#include <FreeRTOS.h>                   /* Main FreeRTOS header               */
#include <task.h>						/* FreeRTOS Tasks					  */
#include <queue.h>						/* FreeRTOS Message-Queues			  */
#include <semphr.h>						/* FreeRTOS Semaphores				  */
#include <timers.h>						/* FreeRTOS Timers					  */

// 極光渲染參數結構
typedef struct {
    float time;
    float waveSpeed;
    float breathSpeed;
    int baseR;
    int rangeR;
    int baseG;
    int rangeG;
    int baseB;
    int rangeB;
} AuroraParams;

// 系統狀態結構
typedef struct {
    uint8_t buttonState;
    uint8_t lastButtonState;
    uint32_t lastDebounceTime;
    uint8_t clickCounter;
    uint8_t rgbMode;
    uint8_t joystickUpState;
    uint8_t joystickDownState;
    float humidity;
    float temperature;
    float brightness;
    bool autoMode;
    bool log;
} SystemState;



#endif /* TYPES_H_ */

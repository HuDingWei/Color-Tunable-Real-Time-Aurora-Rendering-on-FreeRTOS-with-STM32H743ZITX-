/*
 * aurora.c
 *
 *  Created on: Apr 14, 2025
 *      Author: Hu Ding-Wei
 */


#include "aurora.h"

#include <leguan.h>

#include <FreeRTOS.h>                   /* Main FreeRTOS header               */
#include <task.h>						/* FreeRTOS Tasks					  */
#include <queue.h>						/* FreeRTOS Message-Queues			  */
#include <semphr.h>						/* FreeRTOS Semaphores				  */
#include <timers.h>						/* FreeRTOS Timers					  */

// Define global variables
extern AuroraParams auroraParams;
extern SemaphoreHandle_t paramsMutex;

// 任務：渲染極光
void taskAurora(void *pvParameters) {
	(void)pvParameters;
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xFrequency = pdMS_TO_TICKS(50); // 每 50ms 執行一次
	int i = 1;

	while (1) {
		if (i == 1){
//		if (xSemaphoreTake(paramsMutex, xFrequency) == pdTRUE){
			renderWaveEffect_s(&auroraParams);

			// 更新時間，控制動畫速度
			if (auroraParams.time > 7) {
		    	auroraParams.time = (rand() % (RANDOM_MAX + 1 - RANDOM_MIN)) * 0.01;
			} else {
		    	auroraParams.time += (rand() % (RANDOM_MAX + 1 - RANDOM_MIN)) * 0.001;
			}

//			xSemaphoreGive(paramsMutex);

		}
		else {
	    	LOG_Warn("AuroraTask: Failed to take paramsMutex");
		}

		vTaskDelayUntil(&xLastWakeTime, xFrequency);

	}
}

// 簡化的噪聲函數，用於模擬自然律動
float simpleNoise(float x, float time) {
    return 0.5 * sin(x + time) + sin(x * 0.1 + time) * sin(x * 0.05 + time * 0.75) ;
//	return 0.5 * sin(x + time) + 0.3 * sin(x * 0.0004 * 2.0 + time * 0.5) + 0.2 * sin(x * 0.00004 * 0.5 + time * 0.3);
}

void renderWaveEffect_s(AuroraParams* params) {
    // 設定繪製區域為整個螢幕
    LCD_SetDrawArea(0, 0, FRAME_WIDTH - 1, FRAME_HEIGHT - 1);
    LCD_EnableDrawMode();

    // 遍歷螢幕每個像素（x 在外層，y 在內層）
    for (int x = 0; x < FRAME_WIDTH; x+=2) {;
        for (int y = 0; y < FRAME_HEIGHT; y++) {
            // 計算波浪中心線的 y 位置（減少波動密度，模擬更平滑的極光）
            float wave1 = 40.0 * sin(x * 0.015 + params->time * params->waveSpeed);        // 第一層波浪（更平緩）
            float wave2 = 20.0 * sin(x * 0.01 + params->time * params->waveSpeed * 0.5);  // 第二層波浪（稍快）
            float wave3 = 10.0 * simpleNoise(x, params->time * params->waveSpeed);         // 噪聲層，增加自然感
            float waveCenter = FRAME_HEIGHT * 0.75 + (wave1 + wave2 + wave3); // 中心線位置（更靠下）

            // 計算當前像素與中心線的距離（用於漸層）
            float distanceToCenter = (float)y - waveCenter;
            float gradient = 1.0 - (distanceToCenter / (FRAME_HEIGHT * 0.5)); // 範圍 0~1，中心最亮
            gradient = gradient < 0.0 ? 0.0 : (gradient > 1.0 ? 1.0 : gradient); // 限制範圍
            gradient = gradient * gradient; // 使用平方，讓漸層更平滑
//            gradient = pow(gradient, 3.0); // 使用三次方，讓漸層更平滑

            // 計算波浪相位：沿著 x 軸移動，模擬極光
            float wave = (sin(x * 0.01 + params->time * params->waveSpeed) +
                         sin(x * 0.003 + params->time * params->waveSpeed * 0.3) +
                         0.3 * simpleNoise(x, params->time * params->waveSpeed)) * 0.33;

            // 呼吸效果：整體亮度隨時間緩慢變化
            float breath = (sin(params->time * params->breathSpeed) + 1.0) * 0.8; // 範圍 0~1
            breath = breath < 0.2 ? 0.2 : breath; // 設置最小值，避免全黑

            // 計算基礎亮度（範圍 0~1）
            float intensity = (wave * 0.5 + 0.5) * breath * gradient;
            intensity = intensity < 0.05 ? 0.08 : intensity; // 設置最小值，避免全黑

            // 根據 x 軸位置調整顏色，模擬極光從左到右的漸變（藍紫色）
            float colorGradient = (float)x / FRAME_WIDTH; // 範圍 0~1，左邊 0，右邊 1

            // 根據設定的顏色範圍計算 RGB
            int r = (int)(intensity * (params->baseR + params->rangeR * colorGradient));
            int g = (int)(intensity * (params->baseG + params->rangeG * colorGradient));
            int b = (int)(intensity * (params->baseB + params->rangeB * colorGradient));

            // 限制 RGB 值在 0~255 範圍內
            r = r > 255 ? 255 : (r < 0 ? 0 : r);
            g = g > 255 ? 255 : (g < 0 ? 0 : g);
            b = b > 255 ? 255 : (b < 0 ? 0 : b);

            // 設定背景顏色
            LCD_SetBackgroundColor(LCD_ColorConvert(r, g, b));

            // 在該像素位置繪製點（類似原有的 "."）
            LCD_Character(x, y, ".");
        }
    }
}

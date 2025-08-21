#include "types.h"
#include "aurora.h"
#include "sensor.h"
#include "button.h"
#include "status.h"
#include "logdisplay.h"

#include <leguan.h>
#include <cube.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

// 定義隨機數範圍
#define RANDOM_MIN    100
#define RANDOM_MAX    999

#define FRAME_WIDTH   800
#define FRAME_HEIGHT  480

//// 極光渲染參數結構
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
//// 系統狀態結構
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

// global params
AuroraParams auroraParams;
SystemState systemState;
SemaphoreHandle_t paramsMutex;  // 保護 auroraParams 和 systemState
static TaskHandle_t buttonsTaskHandle;
static TaskHandle_t sensorsTaskHandle;
TaskHandle_t statusTaskHandle;
static TaskHandle_t auroraTaskHandle;
static TaskHandle_t displayTaskHandle;

// 創建所有任務的函數
static void vCreateTasks(void);

int main(void) {
    // Initialize Leguan board
    CUBEMX_Init();
    LEGUAN_Init();

    // SysTick
    SystemClock_Config();

    // Setting LOG on LCD
    LOG_SetDestination(LCD_Stream);

    // Initialize auroraParams
    auroraParams.time = 0.0;
    auroraParams.waveSpeed = 1;
    auroraParams.breathSpeed = 0.3;
    auroraParams.baseR = 60;
    auroraParams.rangeR = -30;
    auroraParams.baseG = 20;
    auroraParams.rangeG = 10;
    auroraParams.baseB = 90;
    auroraParams.rangeB = 110;

    // Initialize systemState
    systemState.buttonState = 0;
    systemState.lastButtonState = 0;
    systemState.lastDebounceTime = 0;
    systemState.clickCounter = 0;
    systemState.rgbMode = 0;
    systemState.joystickUpState = 0;
    systemState.joystickDownState = 0;
    systemState.humidity = 0.0;
    systemState.temperature = 0.0;
    systemState.brightness = 0.0;
    systemState.autoMode = false;
    systemState.log = false;

    // Initialize Semaphore if needed
    paramsMutex = xSemaphoreCreateMutex();
    if (paramsMutex == NULL) {
        LOG_Error("Failed to create paramsMutex");
        while (1);
    }

    LOG_Info("Creating vCreateTasks...");
    // Task creating
    vCreateTasks();

    // Activate FreeRTOS Scheduler
    vTaskStartScheduler();

    // Shouldnt come to here
    LOG_Error("RTOS Scheduler Failed");
    while (1);
}


static void vCreateTasks(void) {
    // Create FreeRTOS
    if (xTaskCreate(taskButtons, "ButtonsTask", 128, NULL, 4, &buttonsTaskHandle) != pdPASS) {
        LOG_Error("Failed to create ButtonsTask");
//        while (1);
    }
    if (xTaskCreate(taskAurora, "AuroraTask", 512, NULL, 2, &auroraTaskHandle) != pdPASS) {
        LOG_Error("Failed to create AuroraTask");
//        while (1);
    }
    if (xTaskCreate(taskStatus, "StatusTask", 128, NULL, 3, &statusTaskHandle) != pdPASS) {
        LOG_Error("Failed to create StatusTask");
//        while (1);
    }
    if (xTaskCreate(taskSensors, "SensorsTask", 128, NULL, 2, &sensorsTaskHandle) != pdPASS) {
        LOG_Error("Failed to create SensorsTask");
//        while (1);
    }
//    if (xTaskCreate(taskDisplay, "DisplayTask", 128, NULL, 1, &displayTaskHandle) != pdPASS) {
//        LOG_Error("Failed to create DisplayTask");
////        while (1);
//    }
//    LOG_Info("vCreateTasks Successful");
}

//#include <stdio.h>
//#include <stdlib.h>
//#include <leguan.h>
//#include <cube.h>
//#include <lcd.h>
////#include <color.h>
//#include <usart.h>
//#include <gpio.h>
//#include <main.h>
//#include <sdram.h>
//#include <sensors.h>
//#include <stm32h7xx_hal_sdram.h>
//#include <dma2d.h>
//#include <math.h>
//
///* SDRAM */
//static FMC_SDRAM_CommandTypeDef Command;   //定义SDRAM命令结构体
//#define sdramHandle hsdram1
//#define SDRAM_TIMEOUT                    ((uint32_t)0xFFFF)  //定义超时时间
////volatile uint16_t* sdram = (volatile uint16_t*) 0x30000000ULL;  // SDRAM 基址
//volatile uint16_t *sdram_buffer = (volatile uint16_t *) 0x30000000ULL;  // SDRAM 基址
//
///* LCD */
//#define FRAME_WIDTH  800
//#define FRAME_HEIGHT 480
//#define CAM_FRAME_WIDTH  450
//#define CAM_FRAME_HEIGHT 260
//float time = 0;
//
/////* UART 缓存区 */
//#define UART_BUFFER_SIZE (CAM_FRAME_WIDTH * CAM_FRAME_HEIGHT * 2)  // 每個像素 2 bytes (RGB565)
//volatile uint8_t *uart_buffer = (volatile uint8_t *) 0x30000000;  // 將 SDRAM 當作緩衝區
////uint8_t uart_buffer[UART_BUFFER_SIZE];
//uint8_t rgb_buffer[CAM_FRAME_WIDTH * CAM_FRAME_HEIGHT * 3]; // 存 RGB888 圖像數據
//
///* 使用 SDRAM 存储 UART 数据 */
////__attribute__((section(".sdram"))) uint8_t uart_buffer[UART_BUFFER_SIZE];
//__attribute__((section(".sdram"))) LCD_Color_t buffer[FRAME_WIDTH * FRAME_HEIGHT];  // 放入 SDRAM
//
///* 设置随机数的取值区间范围 */
//#define RANDOM_MIN    100    // 随机数最小值
//#define RANDOM_MAX    999    // 随机数最大值
//
//#define NUMBER_OR_LEDS     (8)
//#define LAST_ENTRY    	   (0)
//#define NUMBER_OF_CYCLES   (6)
//
///**
// * @brief 結構體：封裝極光渲染參數
// */
//typedef struct {
//    float time;         // 時間參數
//    float waveSpeed;    // 波浪移動速度
//    float breathSpeed;  // 呼吸速度
//    int baseR;          // 紅色基礎值
//    int rangeR;         // 紅色範圍
//    int baseG;          // 綠色基礎值
//    int rangeG;         // 綠色範圍
//    int baseB;          // 藍色基礎值
//    int rangeB;         // 藍色範圍
//} AuroraParams;
//
///**
// * @brief 結構體：封裝系統狀態
// */
//typedef struct {
//    uint8_t buttonState;      // 當前按鈕狀態
//    uint8_t lastButtonState;  // 上一次按鈕狀態（用於去抖動）
//    uint32_t lastDebounceTime;// 上一次去抖動時間
//    uint8_t clickCounter;     // 按鈕計數器
//    uint8_t rgbMode;          // RGB 調控模式（0: 不調控, 1: 紅色, 2: 綠色, 3: 藍色）
//    uint8_t joystickUpState;  // 搖桿向上狀態
//    uint8_t joystickDownState;// 搖桿向下狀態
//    float32_t humidity;
//    float32_t temperature;
//    float32_t brightness;
//    bool autoMode;
//} SystemState;
//
////static uint8_t buttonState = 0;
//
///**
// * Structure describing a table entry for cyclical OS
// */
//typedef struct   {
//    uint32_t taskPeriod;     //!< The function will be called every `taskPeriod` cycle
//    void (*function)(void);  //!< Function pointer to action
//} TaskTablEntry;
//
//static void taskLed1(void);
//static void taskLed2(void);
//static void taskButtons(void);
//static void taskDisplay(void);
//void SystemClock_Config(void);
//void LCD_DisplayImage(void);
//void UART_ReceiveImage(void);
//void test_lcd_bitmap();
//static float simpleNoise(float x, float time);
//static void renderWaveEffect_s(AuroraParams* params);
//static void taskAurora(void);
//static void taskStatus(void);
//static void taskSensors(void);
//static void taskButtons(void);
//static void taskDisplay(void);
//
////static uint8_t ledPosition = 0;
////static uint8_t buttonState = 0;
////static uint8_t clickCounter = 0;
//
///**
// * Table (array) defining cyclical OS. Each entry defines an action which is
// * executed cyclically.
// */
//static AuroraParams auroraParams;  // 極光渲染參數
//static SystemState systemState;    // 系統狀態
//static TaskTablEntry taskTable[] = {
//    {2, taskButtons},
//	{2, taskSensors},
//	{2, taskStatus},
//    {1, taskDisplay},
//	{1, taskAurora},    // 極光渲染任務
//    {0, LAST_ENTRY}
//};
//
//int main(void)
//{
//	// Initialize Hardware
//	CUBEMX_Init();
//	// Initialize Leguan board
//	LEGUAN_Init();
//
//	// 設定日誌輸出到 LCD
//	LOG_SetDestination(LCD_Stream);
//
//	//	// 可調整的參數
//	//	float waveSpeed = 1;   // 波浪移動速度（越大越快）
//	//	float breathSpeed = 0.3; // 呼吸速度（越大越快）
//	//
//	//	// 顏色範圍設定（可自由調整）
//	//	int baseR = 120;  // 紅色基礎值（底部顏色）
//	//	int rangeR = 40; // 紅色範圍（從底部到頂部的變化量）
//	//	int baseG = 30;  // 綠色基礎值
//	//	int rangeG = 60; // 綠色範圍
//	//	int baseB = 180; // 藍色基礎值
//	//	int rangeB = 20;// 藍色範圍
//
//	// 初始化極光渲染參數
//	auroraParams.time = 0.0;
//	auroraParams.waveSpeed = 1;
//	auroraParams.breathSpeed = 0.3;
//	auroraParams.baseR = 60;
//	auroraParams.rangeR = -30;
//	auroraParams.baseG = 20;
//	auroraParams.rangeG = 10;
//	auroraParams.baseB = 90;
//	auroraParams.rangeB = 110;
//
//	// 初始化系統狀態
//	systemState.buttonState = 0;
//	systemState.lastButtonState = 0;
//	systemState.lastDebounceTime = 0;
//	systemState.clickCounter = 0;
//	systemState.rgbMode = 0;
//	systemState.joystickUpState = 0;  // 假設初始為未按下（高電平）
//	systemState.joystickDownState = 0; // 假設初始為未按下（高電平）
//	systemState.humidity = 0.0;
//	systemState.temperature = 0.0;
//	systemState.brightness = 0.0;
//	systemState.autoMode = false;
//
//	// 設置任務表中的函數指針
//	taskTable[0].function = taskButtons;
//	taskTable[1].function = taskSensors;
//	taskTable[2].function = taskStatus;
//	taskTable[3].function = taskDisplay;
//	taskTable[4].function = taskAurora;
//
//	// -----------------------------------
//	// Cycle
//	// -----------------------------------
//	size_t cycleCounter = 0;
//	while (1) {
//	    // 遍歷任務表，執行所有準備好的任務
//	    for (size_t taskIndex = 0; taskTable[taskIndex].function != LAST_ENTRY; taskIndex++) {
//	        // 跳過週期為 0 的任務（避免除以 0）
//	        if (taskTable[taskIndex].taskPeriod == 0) {
//	            continue;
//	        }
//
//	        if ((cycleCounter % taskTable[taskIndex].taskPeriod) == 0) {
//	            (*taskTable[taskIndex].function)();
//	        }
//	    }
//
//	    cycleCounter++;
//	    if (cycleCounter >= NUMBER_OF_CYCLES) {
//	        cycleCounter = 0;
//	    }
//
//	    CORE_Delay(100);
//	}
//}
//
//// 簡化的噪聲函數，用於模擬自然律動
//static float simpleNoise(float x, float time) {
//    return 0.5 * sin(x + time) + sin(x * 0.1 + time) * sin(x * 0.05 + time * 0.75) ;
////	return 0.5 * sin(x + time) + 0.3 * sin(x * 0.0004 * 2.0 + time * 0.5) + 0.2 * sin(x * 0.00004 * 0.5 + time * 0.3);
//}
//
//static void renderWaveEffect_s(AuroraParams* params) {
//    // 設定繪製區域為整個螢幕
//    LCD_SetDrawArea(0, 0, FRAME_WIDTH - 1, FRAME_HEIGHT - 1);
//    LCD_EnableDrawMode();
//
//    // 遍歷螢幕每個像素（x 在外層，y 在內層）
//    for (int x = 0; x < FRAME_WIDTH; x+=2) {;
//        for (int y = 0; y < FRAME_HEIGHT; y++) {
//            // 計算波浪中心線的 y 位置（減少波動密度，模擬更平滑的極光）
//            float wave1 = 40.0 * sin(x * 0.015 + params->time * params->waveSpeed);        // 第一層波浪（更平緩）
//            float wave2 = 20.0 * sin(x * 0.01 + params->time * params->waveSpeed * 0.5);  // 第二層波浪（稍快）
//            float wave3 = 10.0 * simpleNoise(x, params->time * params->waveSpeed);         // 噪聲層，增加自然感
//            float waveCenter = FRAME_HEIGHT * 0.75 + (wave1 + wave2 + wave3); // 中心線位置（更靠下）
////            float position = 0.65;
////            if(x % 2 == ){
////            	float position = 0.65;
////            }
////            else{
////            	float position = 0.55;
////            };
////            float waveCenter = FRAME_HEIGHT * position + (wave1 + wave2 + wave3); // 中心線位置（更靠下）
//
//            // 計算當前像素與中心線的距離（用於漸層）
//            float distanceToCenter = (float)y - waveCenter;
//            float gradient = 1.0 - (distanceToCenter / (FRAME_HEIGHT * 0.5)); // 範圍 0~1，中心最亮
//            gradient = gradient < 0.0 ? 0.0 : (gradient > 1.0 ? 1.0 : gradient); // 限制範圍
//            gradient = gradient * gradient; // 使用平方，讓漸層更平滑
////            gradient = pow(gradient, 3.0); // 使用三次方，讓漸層更平滑
//
//            // 計算波浪相位：沿著 x 軸移動，模擬極光
//            float wave = (sin(x * 0.01 + params->time * params->waveSpeed) +
//                         sin(x * 0.003 + params->time * params->waveSpeed * 0.3) +
//                         0.3 * simpleNoise(x, params->time * params->waveSpeed)) * 0.33;
//
//            // 呼吸效果：整體亮度隨時間緩慢變化
//            float breath = (sin(params->time * params->breathSpeed) + 1.0) * 0.8; // 範圍 0~1
//            breath = breath < 0.2 ? 0.2 : breath; // 設置最小值，避免全黑
//
//            // 計算基礎亮度（範圍 0~1）
//            float intensity = (wave * 0.5 + 0.5) * breath * gradient;
//            intensity = intensity < 0.05 ? 0.08 : intensity; // 設置最小值，避免全黑
//
//            // 根據 x 軸位置調整顏色，模擬極光從左到右的漸變（藍紫色）
//            float colorGradient = (float)x / FRAME_WIDTH; // 範圍 0~1，左邊 0，右邊 1
//
//            // 根據設定的顏色範圍計算 RGB
//            int r = (int)(intensity * (params->baseR + params->rangeR * colorGradient));
//            int g = (int)(intensity * (params->baseG + params->rangeG * colorGradient));
//            int b = (int)(intensity * (params->baseB + params->rangeB * colorGradient));
//
//            // 限制 RGB 值在 0~255 範圍內
//            r = r > 255 ? 255 : (r < 0 ? 0 : r);
//            g = g > 255 ? 255 : (g < 0 ? 0 : g);
//            b = b > 255 ? 255 : (b < 0 ? 0 : b);
//
//            // 設定背景顏色
//            LCD_SetBackgroundColor(LCD_ColorConvert(r, g, b));
//
//            // 在該像素位置繪製點（類似原有的 "."）
//            LCD_Character(x, y, ".");
//        }
//    }
//}
//
//// 任務：渲染極光
//static void taskAurora(void) {
//    renderWaveEffect_s(&auroraParams);
//
//    // 更新時間，控制動畫速度
//    if (auroraParams.time > 7) {
//        auroraParams.time = (rand() % (RANDOM_MAX + 1 - RANDOM_MIN)) * 0.01;
//    } else {
//        auroraParams.time += (rand() % (RANDOM_MAX + 1 - RANDOM_MIN)) * 0.001;
//    }
//}
//
//static void taskButtons (void)
//{
//	uint8_t buttons = 0;
//	for (size_t i = 0; i < 7; i++) {
//		buttons |= FPGA_GetButton(i) ? (1 << i) : 0;
//	}
//
//	systemState.buttonState = buttons;
//
//	// 檢測第三個按鈕（Button 2）是否按下（下降沿）
//	if (systemState.buttonState & (1 << Button2)) {
//		if(systemState.autoMode == false){
//			systemState.autoMode = true; // 切換到自動模式
//			systemState.clickCounter++;  // 計數器加 1
//		}
//		else {
//			systemState.autoMode = false; // 切換到自動模式
//			systemState.clickCounter++;  // 計數器加 1
//		}
//	}
//}
//
//// 任務：讀取感測器數據
//static void taskSensors(void) {
//		// Temp CHECK
//		float32_t temperature;
//		if (R_SUCCESS(SENSOR_GetTemperature(&temperature))) {
//			systemState.temperature = temperature;
//	//		LOG_Info("SENSOR_GetTemperature Initialized Successfully");
//		}
//		else {
//		    LOG_Info("SENSOR_GetTemperature Failed");
//		    CORE_Delay(10);
//		}
//
//		// Humid CHECK
//		float32_t humidity;
//		if (R_SUCCESS(SENSOR_GetHumidity(&humidity))) {
//			systemState.humidity = humidity;
////			LOG_Info("SENSOR_GetHumidity Initialized Successfully");
//		}
//		else {
//			LOG_Info("SENSOR_GetTemperature Failed");
//		    CORE_Delay(10);
//		}
//
////		//Color CHECK
////		color_t color;
////		if (R_SUCCESS(SENSOR_GetColor(&color))) {
////		    uint16_t r = color.r;
////		    uint16_t g = color.g;
////		    uint16_t b = color.b;
////
////		    uint16_t brightness = color.a;
////		    systemState.brightness = brightness;
////		    LOG_Info("SENSOR_GetColor Initialized Successfully");
////		}
////		else {
////			LOG_Info("SENSOR_GetColor Failed");
////		    CORE_Delay(10);
////		}
//
//	    LOG_Info("Temp: %.1fC | Hum: %.1f%% | Lux: %.1f",
//	                 systemState.temperature, systemState.humidity, systemState.brightness);
//	//
//	//    CORE_Delay(10);
//
//}
//
//// 任務：讀取按鈕和搖桿狀態，調整 RGB 模式和數值
//void taskStatus(void) {
//    // 讀取按鈕狀態
////    uint8_t button0State = FPGA_GetButton(0); // Button0 狀態
////    uint8_t joystickUpState = FPGA_GetButton(4);   // 假設搖桿向上對應引腳（需根據實際引腳映射）
////    uint8_t joystickDownState = FPGA_GetButton(6); // 假設搖桿向下對應引腳（需根據實際引腳映射）
//
//    // 按鈕去抖動：檢測 Button0 的下降沿（按下）button0State == 1 && systemState.lastButtonState == 0
//    if (systemState.buttonState & (1 << Button0) ) {
//        systemState.rgbMode = (systemState.rgbMode + 1) % 4; // Mode：0 -> 1(R) -> 2(G) -> 3(B) -> 0
//        systemState.clickCounter++;
//    }
////    systemState.lastButtonState = systemState.buttonState;
//
//    // 搖桿控制：根據當前模式調整 RGB 值
//    int step = 5; // 每次調整的步長
//    if (systemState.buttonState & (1 << JoystickNorth)) { // 搖桿向上（下降沿）
//        switch (systemState.rgbMode) {
//            case 1: // 調控紅色
//                auroraParams.baseR = (auroraParams.baseR + step) > 255 ? 255 : (auroraParams.baseR + step);
//                auroraParams.rangeR = (auroraParams.rangeR + step) > 255 ? 255 : (auroraParams.rangeR + step);
//                break;
//            case 2: // 調控綠色
//                auroraParams.baseG = (auroraParams.baseG + step) > 255 ? 255 : (auroraParams.baseG + step);
//                auroraParams.rangeG = (auroraParams.rangeG + step) > 255 ? 255 : (auroraParams.rangeG + step);
//                break;
//            case 3: // 調控藍色
//                auroraParams.baseB = (auroraParams.baseB + step) > 255 ? 255 : (auroraParams.baseB + step);
//                auroraParams.rangeB = (auroraParams.rangeB + step) > 255 ? 255 : (auroraParams.rangeB + step);
//                break;
//            default: // 不調控
//                break;
//        }
//    }
//    if (systemState.buttonState & (1 << JoystickSouth)) { // 搖桿向下（下降沿）
//        switch (systemState.rgbMode) {
//            case 1: // 調控紅色
//                auroraParams.baseR = (auroraParams.baseR - step) < 0 ? 0 : (auroraParams.baseR - step);
//                auroraParams.rangeR = (auroraParams.rangeR - step) < -255 ? -255 : (auroraParams.rangeR - step);
//                break;
//            case 2: // 調控綠色
//                auroraParams.baseG = (auroraParams.baseG - step) < 0 ? 0 : (auroraParams.baseG - step);
//                auroraParams.rangeG = (auroraParams.rangeG - step) < -255 ? -255 : (auroraParams.rangeG - step);
//                break;
//            case 3: // 調控藍色
//                auroraParams.baseB = (auroraParams.baseB - step) < 0 ? 0 : (auroraParams.baseB - step);
//                auroraParams.rangeB = (auroraParams.rangeB - step) < -255 ? -255 : (auroraParams.rangeB - step);
//                break;
//            default: // 不調控
//                break;
//        }
//    }
//
////    // 更新搖桿狀態（用於去抖動）
////    systemState.joystickUpState = systemState.buttonState & (1 << JoystickNorth);
////    systemState.joystickDownState = systemState.buttonState & (1 << JoystickSouth);
//
//    // 根據感測器數據自動調整顏色（如果 autoMode 為 true）
//    if (systemState.autoMode) {
//        // 根據溫度調整顏色（高溫偏暖色，低溫偏冷色）
//        if (systemState.temperature > 25) {
//            // 高溫：偏藍色、綠色（冷色調）
//            auroraParams.baseR = 20;
//            auroraParams.rangeR = -10;
//            auroraParams.baseG = 60;
//            auroraParams.rangeG = 20;
//            auroraParams.baseB = 120;
//            auroraParams.rangeB = 40;;
//        } else if (systemState.temperature < 10) {
//            // 低溫：偏紅色、橙色（暖色調）
//            auroraParams.baseR = 120;
//            auroraParams.rangeR = 40;
//            auroraParams.baseG = 60;
//            auroraParams.rangeG = 20;
//            auroraParams.baseB = 20;
//            auroraParams.rangeB = -10;
//        } else {
//            // 適中溫度：中性色調（淡紫色）
//            auroraParams.baseR = 60;
//            auroraParams.rangeR = -30;
//            auroraParams.baseG = 20;
//            auroraParams.rangeG = 10;
//            auroraParams.baseB = 90;
//            auroraParams.rangeB = 110;
//        }
//
//        // 根據濕度微調（高濕度偏冷色，低濕度偏暖色）
//        if (systemState.humidity > 70) {
//            // 高濕度：增加藍色分量，減少紅色
//            auroraParams.baseB += 20;
//            auroraParams.rangeB += 10;
//            auroraParams.baseR -= 20;
//            auroraParams.rangeR -= 10;
//        } else if (systemState.humidity < 40) {
//            // 低濕度：增加紅色分量，減少藍色
//            auroraParams.baseR += 20;
//            auroraParams.rangeR += 10;
//            auroraParams.baseB -= 20;
//            auroraParams.rangeB -= 10;
//        }
//
//        // 根據亮度微調（高亮度偏柔和色，低亮度偏明亮色）
//        if (systemState.brightness > 500) {
//           // 高亮度：降低整體亮度，偏柔和色（淡色）
//           auroraParams.baseR = (auroraParams.baseR * 0.8 > 255) ? 255 : (auroraParams.baseR * 0.8);
//           auroraParams.baseG = (auroraParams.baseG * 0.8 > 255) ? 255 : (auroraParams.baseG * 0.8);
//           auroraParams.baseB = (auroraParams.baseB * 0.8 > 255) ? 255 : (auroraParams.baseB * 0.8);
//        } else if (systemState.brightness < 200) {
//           // 低亮度：增加整體亮度，偏明亮色（暖色）
//           auroraParams.baseR = (auroraParams.baseR * 1.2 > 255) ? 255 : (auroraParams.baseR * 1.2);
//           auroraParams.baseG = (auroraParams.baseG * 1.2 > 255) ? 255 : (auroraParams.baseG * 1.2);
//           auroraParams.baseB = (auroraParams.baseB * 0.9 > 255) ? 255 : (auroraParams.baseB * 0.9);
//        }
//
//        // 確保 RGB 值在有效範圍內
//        auroraParams.baseR = auroraParams.baseR > 255 ? 255 : (auroraParams.baseR < 0 ? 0 : auroraParams.baseR);
//        auroraParams.rangeR = auroraParams.rangeR > 255 ? 255 : (auroraParams.rangeR < -255 ? -255 : auroraParams.rangeR);
//        auroraParams.baseG = auroraParams.baseG > 255 ? 255 : (auroraParams.baseG < 0 ? 0 : auroraParams.baseG);
//        auroraParams.rangeG = auroraParams.rangeG > 255 ? 255 : (auroraParams.rangeG < -255 ? -255 : auroraParams.rangeG);
//        auroraParams.baseB = auroraParams.baseB > 255 ? 255 : (auroraParams.baseB < 0 ? 0 : auroraParams.baseB);
//        auroraParams.rangeB = auroraParams.rangeB > 255 ? 255 : (auroraParams.rangeB < -255 ? -255 : auroraParams.rangeB);
//    }
//}
//
//// 任務：顯示按鈕計數器、RGB 模式和數值
//static void taskDisplay(void) {
//    const char* modeStr;
//    switch (systemState.rgbMode) {
//        case 0: modeStr = "None"; break;
//        case 1: modeStr = "Red"; break;
//        case 2: modeStr = "Green"; break;
//        case 3: modeStr = "Blue"; break;
//        default: modeStr = "Unknown"; break;
//    }
//
//    LOG_Info("Mode: %s | Auto: %s | Counter: %u | RGB: (%d, %d, %d) | Range: (%d, %d, %d)",
//                 modeStr, systemState.autoMode ? "On" : "Off", systemState.clickCounter,
//                 auroraParams.baseR, auroraParams.baseG, auroraParams.baseB,
//                 auroraParams.rangeR, auroraParams.rangeG, auroraParams.rangeB);
//    LOG_Info("            Temp: %.1fC | Hum: %.1f%% | Lux: %.1f",
//                 systemState.temperature, systemState.humidity, systemState.brightness);
//}

//void test_lcd_bitmap() {
//	LCD_SetDrawArea(0, 0, FRAME_WIDTH - 1, FRAME_HEIGHT - 1);  // 手動設定顯示區域
//	LCD_EnableDrawMode();
//    // 設定 SDRAM 基址
//    volatile uint16_t *buffer = (volatile uint16_t *) 0x30000000ULL;  // SDRAM 基址
//
//    for (int i = 0; i < FRAME_WIDTH; i++){
//    	for (int j = 0; j < FRAME_HEIGHT; j++){
//    		LCD_SetBackgroundColor(LCD_ColorConvert(j/4, i/4, (j+i)/5));
////    		LCD_ColorConvert(j/4, i/4, (j+i)/5);
//    		LCD_Character(i, j, ".");
////    		LCD_String(i, j, ".");
//    	}
//    }
//
////    // 設定紅色 (RGB565 格式)
////    uint16_t red = 0xF800;  // RGB565 格式紅色
////
////    // 填充整個 buffer
////    for (int i = 0; i < FRAME_WIDTH * FRAME_HEIGHT; i++) {
////        buffer[i] = red;
////    }
////
////    // 使用 LCD_DrawBitmap() 顯示圖片
////    LCD_DrawBitmap(buffer, 0, 0, FRAME_WIDTH, FRAME_HEIGHT, North);
////
////    CORE_Delay(100);  // 讓 LCD 有時間更新
//}
//
////void renderWaveEffect(float time) {
////    // 設定繪製區域為整個螢幕
////    LCD_SetDrawArea(0, 0, FRAME_WIDTH - 1, FRAME_HEIGHT - 1);
////    LCD_EnableDrawMode();
////
////    // 遍歷螢幕每個像素
////    for (int x = 0; x < FRAME_WIDTH; x++) {
////        for (int y = 0; y < FRAME_HEIGHT; y++) {
////            // 計算波浪相位：結合 x、y 和時間
////            float waveR = sin(x * 0.02 + time);         // 紅色波浪
////            float waveG = sin(x * 0.03 + time + 1.0);   // 綠色波浪（相位偏移）
////            float waveB = sin(y * 0.04 + time + 2.0);   // 藍色波浪（相位偏移）
////
////            // 呼吸效果：整體亮度隨時間緩慢變化
////            float breath = (sin(time * 0.1) + 1.0) * 0.5; // 範圍 0~1
////
////            // 計算 RGB 亮度（範圍 0~255）
////            int r = (int)((waveR * 0.5 + 0.5) * breath * 255);
////            int g = (int)((waveG * 0.5 + 0.5) * breath * 255);
////            int b = (int)((waveB * 0.5 + 0.5) * breath * 255);
////
////            // 限制 RGB 值在 0~255 範圍內
////            r = r > 255 ? 255 : (r < 0 ? 0 : r);
////            g = g > 255 ? 255 : (g < 0 ? 0 : g);
////            b = b > 255 ? 255 : (b < 0 ? 0 : b);
////
////            // 使用 LCD_ColorConvert 轉換 RGB 值
//////            int color = LCD_ColorConvert(r, g, b);
////
////            // 設定背景顏色
////            LCD_SetBackgroundColor(LCD_ColorConvert(r, g, b));
////
////            // 在該像素位置繪製點（類似原有的 "."）
////            LCD_Character(x, y, ".");
////        }
////    }
////}
//
////void renderWaveEffect(float time,
////                      float waveSpeed, float breathSpeed,
////                      int baseR, int rangeR,
////                      int baseG, int rangeG,
////                      int baseB, int rangeB) {
////    // 設定繪製區域為整個螢幕
////    LCD_SetDrawArea(0, 0, FRAME_WIDTH - 1, FRAME_HEIGHT - 1);
////    LCD_EnableDrawMode();
////
////    // 遍歷螢幕每個像素
////    for (int y = 0; y < FRAME_HEIGHT; y++) {
////        for (int x = 0; x < FRAME_WIDTH; x++) {
////            // 計算波浪相位：沿著 x 軸移動，並在 y 軸上稍微變化
////            float wave = sin(x * 0.02 + time * waveSpeed) + 0.3 * sin(y * 0.05 + time * waveSpeed * 0.5);
////
////            // 呼吸效果：整體亮度隨時間緩慢變化
////            float breath = (sin(time * breathSpeed) + 1.0) * 0.5; // 範圍 0~1
////
////            // 計算基礎亮度（範圍 0~1）
////            float intensity = (wave * 0.5 + 0.5) * breath;
////
////            // 根據 y 軸位置調整顏色，模擬極光從底部到頂部的漸變
////            float colorGradient = (float)y / FRAME_HEIGHT; // 範圍 0~1，底部 0，頂部 1
////
////            // 根據設定的顏色範圍計算 RGB
////            int r = (int)(intensity * (baseR + rangeR * colorGradient));
////            int g = (int)(intensity * (baseG + rangeG * colorGradient));
////            int b = (int)(intensity * (baseB + rangeB * colorGradient));
////
////            // 限制 RGB 值在 0~255 範圍內
////            r = r > 255 ? 255 : (r < 0 ? 0 : r);
////            g = g > 255 ? 255 : (g < 0 ? 0 : g);
////            b = b > 255 ? 255 : (b < 0 ? 0 : b);
////
////            // 設定背景顏色
////            LCD_SetBackgroundColor(LCD_ColorConvert(r, g, b));
////
////            // 在該像素位置繪製點（類似原有的 "."）
////            LCD_Character(x, y, ".");
////        }
////    }
////}
//
////// 簡化的噪聲函數，用於模擬自然律動
////float simpleNoise(float x, float time) {
////    return 0.5 * sin(x + time) + sin(x * 0.1 + time) * sin(x * 0.05 + time * 0.75) ;
//////	return 0.5 * sin(x + time) + 0.3 * sin(x * 0.0004 * 2.0 + time * 0.5) + 0.2 * sin(x * 0.00004 * 0.5 + time * 0.3);
////}
//
//// 渲染波浪效果的函數
//void renderWaveEffect(float time,
//                      float waveSpeed, float breathSpeed,
//                      int baseR, int rangeR,
//                      int baseG, int rangeG,
//                      int baseB, int rangeB) {
//    // 設定繪製區域為整個螢幕
//    LCD_SetDrawArea(0, 0, FRAME_WIDTH - 1, FRAME_HEIGHT - 1);
//    LCD_EnableDrawMode();
//
//    // 遍歷螢幕每個像素
//    for (int x = 0; x < FRAME_WIDTH; x++) {
//        for (int y = 0; y < FRAME_HEIGHT; y++) {
//        	// 計算波浪中心線的 y 位置（疊加多層正弦波和噪聲，模擬自然律動）
//        	float wave1 = 50.0 * sin(x * 0.015 + time * waveSpeed);        // 第一層波浪（更平緩）
//        	float wave2 = 30.0 * sin(x * 0.025 + time * waveSpeed * 0.7);  // 第二層波浪（稍快）
//        	float wave3 = 20.0 * simpleNoise(x, time * waveSpeed);         // 噪聲層，增加自然感
//        	float waveCenter = FRAME_HEIGHT * 0.65 + (wave1 + wave2 + wave3); // 中心線位置（更靠下）
//
//        	// 計算當前像素與中心線的距離（用於漸層）
//        	float distanceToCenter = (float)y - waveCenter;
//        	float gradient = 1.0 - (distanceToCenter / (FRAME_HEIGHT * 0.2)); // 範圍 0~1，中心最亮
//        	gradient = gradient < 0.0 ? 0.0 : (gradient > 1.0 ? 1.0 : gradient); // 限制範圍
//
//        	// 計算波浪相位：沿著 x 軸移動，模擬極光
//        	float wave = (sin(x * 0.015 + time * waveSpeed) +
//        	              sin(x * 0.025 + time * waveSpeed * 0.7) +
//        	              0.5 * simpleNoise(x, time * waveSpeed)) * 0.33;
//
//        	// 呼吸效果：整體亮度隨時間緩慢變化
//        	float breath = (sin(time * breathSpeed) + 1.0) * 0.5; // 範圍 0~1
//
//        	// 計算基礎亮度（範圍 0~1）
//        	float intensity = (wave * 0.5 + 0.5) * breath * gradient;
//
//        	// 根據設定的顏色範圍計算 RGB
//        	int r = (int)(intensity * (baseR + rangeR * gradient));
//        	int g = (int)(intensity * (baseG + rangeG * gradient));
//        	int b = (int)(intensity * (baseB + rangeB * gradient));
//
//        	// 限制 RGB 值在 0~255 範圍內
//        	r = r > 255 ? 255 : (r < 0 ? 0 : r);
//        	g = g > 255 ? 255 : (g < 0 ? 0 : g);
//        	b = b > 255 ? 255 : (b < 0 ? 0 : b);
//
//            // 設定背景顏色
//            LCD_SetBackgroundColor(LCD_ColorConvert(r, g, b));
//
//            // 在該像素位置繪製點（類似原有的 "."）
//            LCD_Character(x, y, ".");
//        }
//    }
//}
//
////// 渲染波浪效果的函數
////void renderWaveEffect_s(float time,
////                      float waveSpeed, float breathSpeed,
////                      int baseR, int rangeR,
////                      int baseG, int rangeG,
////                      int baseB, int rangeB) {
////    // 設定繪製區域為整個螢幕
////    LCD_SetDrawArea(0, 0, FRAME_WIDTH - 1, FRAME_HEIGHT - 1);
////    LCD_EnableDrawMode();
////
////    // 遍歷螢幕每個像素（x 在外層，y 在內層）
////    for (int x = 0; x < FRAME_WIDTH; x+=3) {;
////        for (int y = 0; y < FRAME_HEIGHT; y++) {
////            // 計算波浪中心線的 y 位置（減少波動密度，模擬更平滑的極光）
////            float wave1 = 40.0 * sin(x * 0.015 + time * waveSpeed);        // 第一層波浪（更平緩）
////            float wave2 = 20.0 * sin(x * 0.01 + time * waveSpeed * 0.5);  // 第二層波浪（稍快）
////            float wave3 = 10.0 * simpleNoise(x, time * waveSpeed);         // 噪聲層，增加自然感
////            float waveCenter = FRAME_HEIGHT * 0.75 + (wave1 + wave2 + wave3); // 中心線位置（更靠下）
//////            float position = 0.65;
//////            if(x % 2 == ){
//////            	float position = 0.65;
//////            }
//////            else{
//////            	float position = 0.55;
//////            };
//////            float waveCenter = FRAME_HEIGHT * position + (wave1 + wave2 + wave3); // 中心線位置（更靠下）
////
////            // 計算當前像素與中心線的距離（用於漸層）
////            float distanceToCenter = (float)y - waveCenter;
////            float gradient = 1.0 - (distanceToCenter / (FRAME_HEIGHT * 0.5)); // 範圍 0~1，中心最亮
////            gradient = gradient < 0.0 ? 0.0 : (gradient > 1.0 ? 1.0 : gradient); // 限制範圍
////            gradient = gradient * gradient; // 使用平方，讓漸層更平滑
//////            gradient = pow(gradient, 3.0); // 使用三次方，讓漸層更平滑
////
////            // 計算波浪相位：沿著 x 軸移動，模擬極光
////            float wave = (sin(x * 0.01 + time * waveSpeed) +
////                         sin(x * 0.003 + time * waveSpeed * 0.3) +
////                         0.3 * simpleNoise(x, time * waveSpeed)) * 0.33;
////
////            // 呼吸效果：整體亮度隨時間緩慢變化
////            float breath = (sin(time * breathSpeed) + 1.0) * 0.8; // 範圍 0~1
////            breath = breath < 0.2 ? 0.2 : breath; // 設置最小值，避免全黑
////
////            // 計算基礎亮度（範圍 0~1）
////            float intensity = (wave * 0.5 + 0.5) * breath * gradient;
////            intensity = intensity < 0.05 ? 0.08 : intensity; // 設置最小值，避免全黑
////
////            // 根據 x 軸位置調整顏色，模擬極光從左到右的漸變（藍紫色）
////            float colorGradient = (float)x / FRAME_WIDTH; // 範圍 0~1，左邊 0，右邊 1
////
////            // 根據設定的顏色範圍計算 RGB
////            int r = (int)(intensity * (baseR + rangeR * colorGradient));
////            int g = (int)(intensity * (baseG + rangeG * colorGradient));
////            int b = (int)(intensity * (baseB + rangeB * colorGradient));
////
////            // 限制 RGB 值在 0~255 範圍內
////            r = r > 255 ? 255 : (r < 0 ? 0 : r);
////            g = g > 255 ? 255 : (g < 0 ? 0 : g);
////            b = b > 255 ? 255 : (b < 0 ? 0 : b);
////
////            // 設定背景顏色
////            LCD_SetBackgroundColor(LCD_ColorConvert(r, g, b));
////
////            // 在該像素位置繪製點（類似原有的 "."）
////            LCD_Character(x, y, ".");
////        }
////    }
////}
//
//// 簡單的 1D 噪聲函數（模擬 Perlin 噪聲）
//float simpleMultiNoise(float x, float time) {
//    return 0.5 * sin(x * 0.01 + time) + 0.3 * sin(x * 0.02 + time * 0.5) + 0.2 * sin(x * 0.005 + time * 0.3);
//}
//
//// 渲染極光效果的函數
//void renderWaveEffect_multi(float time,
//                        float waveSpeed, float breathSpeed,
//                        int baseR, int rangeR,
//                        int baseG, int rangeG,
//                        int baseB, int rangeB) {
////    // 每次刷新前清空螢幕（避免殘留數據）
////    LCD_ClearScreen();
//
//    // 設定繪製區域為整個螢幕
//    LCD_SetDrawArea(0, 0, FRAME_WIDTH - 1, FRAME_HEIGHT - 1);
//    LCD_EnableDrawMode();
//
//    // 遍歷螢幕每個像素（x 在外層，y 在內層）
//    for (int x = 0; x < FRAME_WIDTH; x += 3) {
//        for (int y = 0; y < FRAME_HEIGHT; y++) {
//            // 計算波浪中心線的 y 位置（模擬 2~3 個有大有小的波）
//            float wave1 = 60.0 * sin(x * 0.0005 + time * waveSpeed);       // 第一層波浪（大波，極低頻）
//            float wave2 = 40.0 * sin(x * 0.0008 + time * waveSpeed * 0.7); // 第二層波浪（中波，稍快）
//            float wave3 = 20.0 * sin(x * 0.0012 + time * waveSpeed * 0.4); // 第三層波浪（小波，更快）
//            float noise = 10.0 * simpleMultiNoise(x, time * waveSpeed);         // 噪聲層，增加自然感
//            float waveCenter = FRAME_HEIGHT * 0.65 + (wave1 + wave2 + wave3 + noise); // 中心線位置
//
//            // 計算當前像素與中心線的距離（用於上下漸層）
//            float distanceToCenter = (float)y - waveCenter;
//            float gradientWave = 0.2 * sin(x * 0.0003 + time * waveSpeed * 0.5); // 上下漸層的波動
//            float gradient = 1.0 - (distanceToCenter / (FRAME_HEIGHT * (0.8 + gradientWave))); // 範圍 0~1，中心最亮
//            gradient = gradient < 0.0 ? 0.0 : (gradient > 1.0 ? 1.0 : gradient); // 限制範圍
//            gradient = pow(gradient, 3.0); // 使用三次方，讓漸層更平滑且有長短變化
//
//            // 計算波浪相位：沿著 x 軸移動，模擬極光
//            float wave = (sin(x * 0.0005 + time * waveSpeed) +
//                          sin(x * 0.0008 + time * waveSpeed * 0.7) +
//                          0.3 * simpleMultiNoise(x, time * waveSpeed)) * 0.33;
//
//            // 呼吸效果：整體亮度隨時間緩慢變化
//            float breath = (sin(time * breathSpeed) + 1.0) * 0.5; // 範圍 0~1
//            breath = breath < 0.3 ? 0.3 : breath; // 設置最小值，避免全黑
//
//            // 計算基礎亮度（範圍 0~1）
//            float intensity = (wave * 0.5 + 0.5) * breath * gradient;
//            intensity = intensity < 0.05 ? 0.05 : intensity; // 設置最小值，避免全黑
//
//            // 根據 x 軸位置調整顏色，模擬極光從左到右的漸變（藍紫色）
//            float colorGradient = (float)x / FRAME_WIDTH; // 範圍 0~1，左邊 0，右邊 1
//            colorGradient = pow(colorGradient, 2.0); // 使用更高指數，讓顏色過渡更柔和
//
//            // 根據設定的顏色範圍計算 RGB（去除綠色，專注藍紫色）
//            int r = (int)(intensity * (baseR + rangeR * colorGradient));
//            int g = (int)(intensity * (baseG + rangeG * colorGradient));
//            int b = (int)(intensity * (baseB + rangeB * colorGradient));
//
//            // 限制 RGB 值在 0~255 範圍內
//            r = r > 255 ? 255 : (r < 0 ? 0 : r);
//            g = g > 255 ? 255 : (g < 0 ? 0 : g);
//            b = b > 255 ? 255 : (b < 0 ? 0 : b);
//
//            // 設定背景顏色
//            LCD_SetBackgroundColor(LCD_ColorConvert(r, g, b));
//
//            // 在該像素位置繪製點（類似原有的 "."）
//            LCD_Character(x, y, ".");
//        }
//    }
//}
//
//// 渲染波浪效果的函數
//void renderWaveEffect_mix(float time,
//                      float waveSpeed, float breathSpeed,
//                      int baseR, int rangeR,
//                      int baseG, int rangeG,
//                      int baseB, int rangeB) {
//    // 設定繪製區域為整個螢幕
//    LCD_SetDrawArea(0, 0, FRAME_WIDTH - 1, FRAME_HEIGHT - 1);
//    LCD_EnableDrawMode();
//
//    // 遍歷螢幕每個像素（x 在外層，y 在內層）
//    for (int x = 0; x < FRAME_WIDTH; x++) {
//        for (int y = 0; y < FRAME_HEIGHT; y++) {
//        	// 計算 2~3 層波動，模擬層次感（進一步降低頻率，模擬窗簾起伏）
//        	float wave1 = 30.0 * sin(x * 0.0001 + time * waveSpeed * 0.5); // 第一層波浪（最慢）
//        	float wave2 = 20.0 * sin(x * 0.00015 + time * waveSpeed * 0.3); // 第二層波浪（稍快）
//        	float wave3 = 10.0 * sin(x * 0.0002 + time * waveSpeed * 0.2); // 第三層波浪（更快）
//        	float waveCenter = FRAME_HEIGHT * 0.65 + (wave1 + wave2 + wave3); // 中心線位置
//
//        	// 計算當前像素與中心線的距離（用於漸層）
//        	float distanceToCenter = (float)y - waveCenter;
//
//        	// 引入波動影響上下亮度分佈，讓漸層有長短變化
//        	float gradientWave = 0.2 * sin(x * 0.0001 + time * waveSpeed * 0.4); // 上下漸層的波動
//        	float gradient = 1.0 - (distanceToCenter / (FRAME_HEIGHT * (1.0 + gradientWave))); // 範圍 0~1，中心最亮
//        	gradient = gradient < 0.0 ? 0.0 : (gradient > 1.0 ? 1.0 : gradient); // 限制範圍
//        	gradient = pow(gradient, 4.0); // 使用四次方，讓漸層更平滑
//
//        	// 呼吸效果：整體亮度隨時間緩慢變化
//        	float breath = (sin(time * breathSpeed) + 1.0) * 0.5; // 範圍 0~1
//        	breath = breath < 0.3 ? 0.3 : breath; // 設置最小值，避免全黑
//
//        	// 計算基礎亮度（範圍 0~1）
//        	float intensity = breath * gradient; // 亮度由 gradient 控制，波動影響中心線和漸層
//        	intensity = intensity < 0.1 ? 0.1 : intensity; // 設置最小值，避免全黑
//
//        	// 根據 x 軸位置調整顏色，模擬從左到右的平滑漸變（紫色到藍色）
//        	float colorGradient = (float)x / FRAME_WIDTH; // 範圍 0~1，左邊 0，右邊 1
//        	colorGradient = pow(colorGradient, 1.5); // 使用指數，讓顏色過渡更平滑
//
//            // 根據設定的顏色範圍計算 RGB（紫色到藍色）
//            int r = (int)(intensity * (baseR + rangeR * colorGradient));
//            int g = (int)(intensity * (baseG + rangeG * colorGradient));
//            int b = (int)(intensity * (baseB + rangeB * colorGradient));
//
//            // 限制 RGB 值在 0~255 範圍內
//            r = r > 255 ? 255 : (r < 0 ? 0 : r);
//            g = g > 255 ? 255 : (g < 0 ? 0 : g);
//            b = b > 255 ? 255 : (b < 0 ? 0 : b);
//
//            // 設定背景顏色
//            LCD_SetBackgroundColor(LCD_ColorConvert(r, g, b));
//
//            // 在該像素位置繪製點（類似原有的 "."）
//            LCD_Character(x, y, ".");
//
//
//        }
//    }
//}
//
//// 假設的延遲函數（單位：毫秒）
//extern void delay(int ms);
//
//// 簡單的 1D 噪聲函數（模擬 Perlin 噪聲）
//float simpleAuroraNoise(float x, float freq, float time) {
//	return 0.5 * sin(x * freq + time) + 0.3 * sin(x * freq * 2.0 + time * 0.5) + 0.2 * sin(x * freq * 0.5 + time * 0.3);
//}
//
//// 渲染波浪效果的函數
//void renderAuroraEffect(float time,
//                      float waveSpeed, float breathSpeed,
//                      int baseR, int rangeR,
//                      int baseG, int rangeG,
//                      int baseB, int rangeB) {
//    // 設定繪製區域為整個螢幕
//    LCD_SetDrawArea(0, 0, FRAME_WIDTH - 1, FRAME_HEIGHT - 1);
//    LCD_EnableDrawMode();
//
//    // 遍歷螢幕每個像素（x 在外層，y 在內層）
//    for (int x = 0; x < FRAME_WIDTH; x++) {
//        for (int y = 0; y < FRAME_HEIGHT; y++) {
//        	// 模擬極光幕的平行結構（文章提到的 Auroral Curtains）
//        	// 使用多層低頻正弦波生成 2D 光柵（footprint）
//        	float wave1 = 40.0 * sin(x * 0.00007 + time * waveSpeed * 0.5); // 第一層波浪（極低頻）
//        	float wave2 = 30.0 * sin(x * 0.0001 + time * waveSpeed * 0.3);  // 第二層波浪（低頻）
//        	float wave3 = 20.0 * sin(x * 0.00013 + time * waveSpeed * 0.2); // 第三層波浪（稍高頻）
//
//        	// 添加噪聲，增加隨機性
//        	float noiseHighFreq = 5.0 * simpleAuroraNoise(x, 0.01, time * 0.5);  // 高頻小幅度噪聲
//        	float noiseLowFreq = 20.0 * simpleAuroraNoise(x, 0.0004, time * 0.2); // 低頻大幅度噪聲
//        	float noiseMidFreq = 0.3 * simpleAuroraNoise(x, 0.004, time * 0.3);  // 中頻中幅度噪聲
//
//        	// 中心線位置（模擬極光幕）
//        	float waveCenter = FRAME_HEIGHT * 0.65 + (wave1 + wave2 + wave3) + noiseHighFreq + noiseLowFreq;
//
//        	// 計算當前像素與中心線的距離（用於上下漸層）
//        	float distanceToCenter = (float)y - waveCenter;
//
//        	// 引入波動影響上下亮度分佈，讓漸層有長短變化
//        	float gradientWave = 0.25 * sin(x * 0.0001 + time * waveSpeed * 0.4); // 上下漸層的波動
//        	float gradient = 1.0 - (distanceToCenter / (FRAME_HEIGHT * (1.0 + gradientWave))); // 範圍 0~1，中心最亮
//        	gradient = gradient < 0.0 ? 0.0 : (gradient > 1.0 ? 1.0 : gradient); // 限制範圍
//        	gradient = pow(gradient, 3.5); // 使用 3.5 次方，讓漸層更柔和
//
//        	// 呼吸效果：模擬環境光照的變化
//        	float breath = (sin(time * breathSpeed) + 1.0) * 0.5; // 範圍 0~1
//        	breath = breath < 0.3 ? 0.3 : breath; // 設置最小值，避免全黑
//
//        	// 計算基礎亮度（範圍 0~1）
//        	float density = breath * gradient * (1.0 + noiseMidFreq); // 加入中頻噪聲影響密度
//        	density = density < 0.1 ? 0.1 : density; // 設置最小值，避免全黑
//
//        	// 根據 x 軸位置調整顏色，模擬從左到右的平滑漸變（紫色到藍色）
//        	float colorGradient = (float)x / FRAME_WIDTH; // 範圍 0~1，左邊 0，右邊 1
//        	colorGradient = pow(colorGradient, 2.0); // 使用更高指數，讓顏色過渡更柔和
//
//        	// 根據設定的顏色範圍計算 RGB（紫色到藍色，論文 2.2 節的自定義 colormap）
//        	int r = (int)(density * (baseR + rangeR * colorGradient));
//        	int g = (int)(density * (baseG + rangeG * colorGradient));
//        	int b = (int)(density * (baseB + rangeB * colorGradient));
//
//            // 限制 RGB 值在 0~255 範圍內
//            r = r > 255 ? 255 : (r < 0 ? 0 : r);
//            g = g > 255 ? 255 : (g < 0 ? 0 : g);
//            b = b > 255 ? 255 : (b < 0 ? 0 : b);
//
//            // 設定背景顏色
//            LCD_SetBackgroundColor(LCD_ColorConvert(r, g, b));
//
//            // 在該像素位置繪製點（類似原有的 "."）
//            LCD_Character(x, y, ".");
//
//
//        }
//    }
//}
//
//void getAuroraRGB(int x, int y, int* r, int* g, int* b) {
//    double r_total = 0.0;
//    double g_total = 0.0;
//    double b_total = 0.0;
//
//    // 藍色帶
//    double base_x1 = FRAME_WIDTH / 4.0;
//    double amplitude1 = FRAME_WIDTH / 20.0;
//    double frequency1 = 2.0 * M_PI * 2.0 / FRAME_HEIGHT;
//    double phase1 = 0.0;
//    double sigma1 = FRAME_WIDTH / 30.0;
//    double x_center1 = base_x1 + amplitude1 * sin(frequency1 * y + phase1);
//    double distance1 = fabs(x - x_center1);
//    double intensity1 = exp(- (distance1 * distance1) / (2.0 * sigma1 * sigma1));
//    r_total += intensity1 * 0;
//    g_total += intensity1 * 0;
//    b_total += intensity1 * 255;
//
//    // 綠色帶（中心）
//    double base_x2 = FRAME_WIDTH / 2.0;
//    double amplitude2 = FRAME_WIDTH / 10.0;
//    double frequency2 = 2.0 * M_PI * 1.5 / FRAME_HEIGHT;
//    double phase2 = M_PI / 3.0;
//    double sigma2 = FRAME_WIDTH / 15.0;
//    double x_center2 = base_x2 + amplitude2 * sin(frequency2 * y + phase2);
//    double distance2 = fabs(x - x_center2);
//    double intensity2 = exp(- (distance2 * distance2) / (2.0 * sigma2 * sigma2));
//    r_total += intensity2 * 0;
//    g_total += intensity2 * 255;
//    b_total += intensity2 * 0;
//
//    // 紫色帶
//    double base_x3 = 3.0 * FRAME_WIDTH / 4.0;
//    double amplitude3 = FRAME_WIDTH / 15.0;
//    double frequency3 = 2.0 * M_PI * 2.5 / FRAME_HEIGHT;
//    double phase3 = M_PI / 2.0;
//    double sigma3 = FRAME_WIDTH / 30.0;
//    double x_center3 = base_x3 + amplitude3 * sin(frequency3 * y + phase3);
//    double distance3 = fabs(x - x_center3);
//    double intensity3 = exp(- (distance3 * distance3) / (2.0 * sigma3 * sigma3));
//    r_total += intensity3 * 128;
//    g_total += intensity3 * 0;
//    b_total += intensity3 * 128;
//
//    // 應用垂直漸變：底部較暗
//    double gradient = 1.0 - (double)y / FRAME_HEIGHT;
//    r_total *= gradient;
//    g_total *= gradient;
//    b_total *= gradient;
//
//    // 限制RGB值在0-255範圍內
//    *r = (int)fmin(255.0, fmax(0.0, r_total));
//    *g = (int)fmin(255.0, fmax(0.0, g_total));
//    *b = (int)fmin(255.0, fmax(0.0, b_total));
//}
//
//void genAuroraRGB(){
//	for (int x = 0; x < FRAME_WIDTH; x++) {
//	    for (int y = 0; y < FRAME_HEIGHT; y++) {
//	        int r, g, b;
//	        getAuroraRGB(x, y, &r, &g, &b);
//	        LCD_SetBackgroundColor(LCD_ColorConvert(r, g, b));
//	        LCD_Character(x, y, ".");
//	    }
//	}
//}
//
////int main(void)
////{
////	// Initialize Hardware
////	CUBEMX_Init();
////	// Initialize Leguan board
////	LEGUAN_Init();
//////	test_lcd_bitmap();
//////	volatile uint16_t *sdram = (volatile uint16_t *) 0xC0000000ULL;  // SDRAM 基址
////
//////    HAL_Init();
//////    SystemClock_Config();
////
////	// Set logging output destination to be the LCD
////	LOG_SetDestination(LCD_Stream);
//////	LCD_EnableDrawMode();
//////	MX_GPIO_Init();
//////	MX_FMC_Init();
//////    MX_DMA2D_Init();
//////	test_lcd_bitmap();
////
//////	MX_USART1_UART_Init();
//////	HAL_UART_Init(&huart1);
////
//////	test_lcd_buffer();
//////	test_sdram();
//////	test_lcd_rect();
//////	test_lcd();
//////	test_lcd_bitmap();
//////	USART_ReceiveImage();
//////	test_lcd_bitmap_cam();
////
//////	volatile uint16_t *test_sdram = (volatile uint16_t *)0xC0000000;
//////	*test_sdram = 0x1234;
////
//////	LCD_SetForegroundColor(ColorGreen);
//////	LCD_SetBackgroundColor(ColorBlue);
//////	if (*test_sdram == 0x1234) {
//////		LCD_String(100, 300, "SDRAM JA");
//////	} else {
//////		LCD_String(100, 300, "SDRAM Nei");
//////	}
//////	test_lcd_bitmap();
//////	size_t cycleCounter = 0;
////	float time = 0.0;
////
////	// 可調整的參數
////	float waveSpeed = 1;   // 波浪移動速度（越大越快）
////	float breathSpeed = 0.3; // 呼吸速度（越大越快）
////
////	// 顏色範圍設定（可自由調整）
////	int baseR = 120;  // 紅色基礎值（底部顏色）
////	int rangeR = 40; // 紅色範圍（從底部到頂部的變化量）
////	int baseG = 30;  // 綠色基礎值
////	int rangeG = 60; // 綠色範圍
////	int baseB = 180; // 藍色基礎值
////	int rangeB = 20;// 藍色範圍
////
////    while (1) {
////
//////    	test_lcd_bitmap();
////
////    	// 渲染波浪效果
////    	renderWaveEffect_s(time, waveSpeed, breathSpeed,
////    	                 baseR, rangeR, baseG, rangeG, baseB, rangeB);
////
//////    	renderAuroraEffect(time, waveSpeed, breathSpeed,
//////                baseR, rangeR, baseG, rangeG, baseB, rangeB);
////
//////    	renderWaveEffect(time);
//////    // 更新時間，控制動畫速度
////    	if(time > 7 ){
//////    		time = ((rand() % (RANDOM_MAX + 1 - RANDOM_MIN)) + RANDOM_MIN) * 0.001;      //产生一个30到50的随机数
////    		time = rand() % (RANDOM_MAX + 1 - RANDOM_MIN) * 0.01;
////    	}
////    	else{
////    		time += rand() % (RANDOM_MAX + 1 - RANDOM_MIN) * 0.001;
////    	}
//////    	time = (random(1000) % (RANDOM_MAX + 1 - RANDOM_MIN) + RANDOM_MIN) * 0.0001;
////
//////    	genAuroraRGB();
////    }
////
////}
//
///**
// * Doing something with LED's L(0,0) to L(0,7)
// */
////void taskLed1(void)
////{
////    for (size_t i = 0; i < NUMBER_OR_LEDS; i++) {
////    	FPGA_MatrixSetPixel(i, 0, i == ledPosition ? ColorRed : ColorBlack);
////    }
////
////    ledPosition++;
////    if (ledPosition >= NUMBER_OR_LEDS) {
////    	ledPosition = 0;
////    }
////}
//
//
///**
// * Doing something with LED's L(1,0) to L(1,7)
// */
////void  taskLed2 (void)
////{
////	for (size_t i = 0; i < NUMBER_OR_LEDS; i++) {
////		FPGA_MatrixSetPixel(i, 1, i == ledPosition ? ColorRed : ColorBlack);
////	}
////}
////
/////**
//// * Get the Buttons value
//// */
////void  taskButtons (void)
////{
////	uint8_t buttons = 0;
////	for (size_t i = 0; i < 4; i++) {
////		buttons |= FPGA_GetButton(i) ? (1 << i) : 0;
////	}
////
////	buttonState = buttons;
////}
////
/////**
//// * Display the button counter
//// */
////void  taskDisplay (void)
////{
////    if (buttonState & (1 << Button0)) {
////    	clickCounter++;
////    }
////
////    if (buttonState & (1 << Button1)) {
////    	clickCounter--;
////    }
////
////    LOG_Info("Button counter = %u", clickCounter);
////}
//
///* CSDN https://blog.csdn.net/as480133937/article/details/123791568 */
////uint16_t testsram[250000] __attribute__((at(0XC0000000)));//测试用数组
////int main(void)
////{
////  uint32_t ts=0;
////  for(ts=0;ts<250000;ts++)
////  {
////	testsram[ts]=ts;//预存测试数据
////  }
////	HAL_Delay(2000);
////	fsmc_sdram_test();
////	HAL_Delay(2000);
////
////	for(ts=0;ts<250000;ts++)
////	{
////	  printf("testsram[%d]:%d\r\n",ts,testsram[ts]);  //打印SDRAM数据
////	}
////
////
////	while(1)
////	{
////
////	}
////}
//
//////SDRAM内存测试
////void fsmc_sdram_test()
////{
////	__IO uint32_t i=0;
////	__IO uint32_t temp=0;
////	__IO uint32_t sval=0;	//在地址0读到的数据
////
////	//每隔16K字节,写入一个数据,总共写入2048个数据,刚好是32M字节
////	for(i=0;i<32*1024*1024;i+=16*1024)
////	{
////		*(__IO uint32_t*)(SDRAM_BANK_ADDR+i)=temp;
////		temp++;
////	}
////	//依次读出之前写入的数据,进行校验
//// 	for(i=0;i<32*1024*1024;i+=16*1024)
////	{
////  		temp=*(__IO uint32_t*)(SDRAM_BANK_ADDR+i);
////		if(i==0)sval=temp;
//// 		else if(temp<=sval)break;//后面读出的数据一定要比第一次读到的数据大.
////		printf("SDRAM Capacity:%dKB\r\n",(uint16_t)(temp-sval+1)*16);//打印SDRAM容量
//// 	}
////}
///* CSDN https://blog.csdn.net/as480133937/article/details/123791568 */
//
////void test_lcd() {
////    LCD_Color_t test_image[FRAME_WIDTH * FRAME_HEIGHT];
////    for (int i = 0; i < FRAME_WIDTH * FRAME_HEIGHT; i++) {
////        test_image[i] = LCD_COLOR(5, 0, 0);  // 全部填充红色
////    }
////    LCD_DrawBuffer(test_image, FRAME_WIDTH * FRAME_HEIGHT);
////}
//
////void test_lcd() {
////    uint16_t test_image[FRAME_WIDTH * FRAME_HEIGHT];  // 直接使用 uint16_t 数组
////    uint16_t red = ((5 & 0x1F) << 11);  // 5-bit 红色，转换为 RGB565
////
////    for (int i = 0; i < FRAME_WIDTH * FRAME_HEIGHT; i++) {
////        test_image[i] = red;  // 直接填充红色
////    }
////
////    LCD_DrawBuffer((LCD_Color_t *)test_image, FRAME_WIDTH * FRAME_HEIGHT);
////}
//
////void test_lcd() {
////    LCD_Color_t test_image[FRAME_WIDTH * FRAME_HEIGHT];
////    LCD_Color_t image = { image.r = 255, image.g = 5, image.b = 0 };  // 结构体赋值
////
////    for (int i = 0; i < FRAME_WIDTH * FRAME_HEIGHT; i++) {
////        test_image[i] = image;
////    }
////
////    LCD_SetDrawArea(0, 0, FRAME_WIDTH - 1, FRAME_HEIGHT - 1);
////    LCD_EnableDrawMode();
////    LCD_DrawBuffer((LCD_Color_t *)test_image, FRAME_WIDTH * FRAME_HEIGHT);
////
////    CORE_Delay(50);  // 确保数据完成绘制
////    LCD_Clear();     // 再次清屏，避免覆盖
////}
//
////void test_lcd() {
////    volatile uint16_t *framebuffer = (volatile uint16_t *)LCD_BASE_ADDRESS;
////    uint16_t red = (31 << 11);  // 純紅色 RGB565 (5-bit R, 6-bit G, 5-bit B)
////
////    for (int i = 0; i < FRAME_WIDTH * FRAME_HEIGHT; i++) {
////        framebuffer[i] = red;  // 直接寫入 LCD 顯存
////    }
////
////    CORE_Delay(100);  // 等待畫面更新
////}
//
////void test_lcd() {
////    LCD_Color_t test_image[FRAME_WIDTH * FRAME_HEIGHT];
////    LCD_Color_t image = { .r = 31, .g = 5, .b = 0 };
////
////    for (int i = 0; i < FRAME_WIDTH * FRAME_HEIGHT; i++) {
////        test_image[i] = image;  // 紅色 (RGB565)
////    }
////
////    LCD_DrawBuffer(test_image, FRAME_WIDTH * FRAME_HEIGHT);
////}
//
////void test_lcd() {
////    LCD_Color_t test_image[FRAME_WIDTH * FRAME_HEIGHT];
////
////    for (int i = 0; i < FRAME_WIDTH * FRAME_HEIGHT; i++) {
////        test_image[i] = LCD_COLOR(31, 0, 0);  // 紅色 (RGB565)
////    }
////
////    LCD_DrawBuffer(test_image, FRAME_WIDTH * FRAME_HEIGHT);
////}
//
//void test_lcd() {
////    uint16_t test_image[FRAME_WIDTH * FRAME_HEIGHT];  // 直接使用 uint16_t 数组
////    uint16_t red = ((5 & 0x1F) << 11);  // 5-bit 红色，转换为 RGB565
////
////    for (int i = 0; i < FRAME_WIDTH * FRAME_HEIGHT; i++) {
////        test_image[i] = red;  // 直接填充红色
////    }
//
////    LCD_DrawBuffer((LCD_Color_t *)test_image, FRAME_WIDTH * FRAME_HEIGHT);
//
////    static LCD_Color_t buffer[FRAME_WIDTH * FRAME_HEIGHT];
////    volatile LCD_Color_t *buffer = (volatile LCD_Color_t *) 0x30000000;  // SDRAM 基址
////    volatile uint16_t *buffer = (volatile uint16_t *) 0x30000000;  // SDRAM 基址
//    uint8_t r = 100;
//    uint8_t g = 100;
//    uint8_t b = 0;
//    for (int i = 0; i < FRAME_WIDTH * FRAME_HEIGHT; i++) {
//    	buffer[i] = LCD_ColorConvert565(r, g, b);  // 紅色
////        buffer[i] = LCD_ColorConvert565_uint16(r, g, b);  // 紅色
////        buffer[i] = RGB565_unit16(255,0,0);
//
//    }
//    LCD_SetBackgroundColor(ColorPurple);
//    LCD_DrawBuffer((LCD_Color_t *)buffer, FRAME_WIDTH * FRAME_HEIGHT);
////    LCD_DrawBuffer_uint16((uint16_t *)buffer, FRAME_WIDTH * FRAME_HEIGHT);
////    LCD_DrawBuffer((LCD_Color_t *)SDRAM_BASE_ADDRESS, FRAME_WIDTH * FRAME_HEIGHT);
//}
//
////void test_lcd_bitmap(void) {
////	LCD_SetDrawArea(0, 0, FRAME_WIDTH - 1, FRAME_HEIGHT - 1);  // 手動設定顯示區域
////	LCD_EnableDrawMode();
////    // 設定 SDRAM 基址
////    volatile uint16_t *buffer = (volatile uint16_t *) 0x30000000ULL;  // SDRAM 基址
////
////    for (int i = 0; i < FRAME_WIDTH; i++){
////    	for (int j = 0; j < FRAME_HEIGHT; j++){
////    		LCD_SetBackgroundColor(LCD_ColorConvert(j/4, i/4, (j+i)/5));
//////    		LCD_ColorConvert(j/4, i/4, (j+i)/5);
////    		LCD_Character(i, j, ".");
//////    		LCD_String(i, j, ".");
////    	}
////    }
////
//////    // 設定紅色 (RGB565 格式)
//////    uint16_t red = 0xF800;  // RGB565 格式紅色
//////
//////    // 填充整個 buffer
//////    for (int i = 0; i < FRAME_WIDTH * FRAME_HEIGHT; i++) {
//////        buffer[i] = red;
//////    }
//////
//////    // 使用 LCD_DrawBitmap() 顯示圖片
//////    LCD_DrawBitmap(buffer, 0, 0, FRAME_WIDTH, FRAME_HEIGHT, North);
//////
//////    CORE_Delay(100);  // 讓 LCD 有時間更新
////}
//
////void test_lcd() {
////    volatile uint16_t *lcd_mem = (volatile uint16_t *)0xD0000000;
////    uint16_t red = (31 << 11);  // RGB565 紅色
////
////    printf("TESTING LCD RED...\n");
////
////    for (int i = 0; i < FRAME_WIDTH * FRAME_HEIGHT; i++) {
////        lcd_mem[i] = red;
////    }
////
////    printf("LCD RED！\n");
////}
////#define RGB565_COLOR(r, g, b) 		{ ((uint8_t)(r)) >> 3, 	    ((uint8_t)(g)) >> 2,    ((uint8_t)(b)) >> 3 }
////typedef struct {
////	uint16_t r : 5;
////	uint16_t g : 0;
////	uint16_t b : 0;
////} RGB565_t;
//typedef uint16_t LCD_Color_main;  // 這樣 LCD_Color_t 直接對應 uint16_t
//
//void test_lcd_buffer() {
//    static LCD_Color_t buffer[FRAME_WIDTH * FRAME_HEIGHT];
//
//    for (int i = 0; i < FRAME_WIDTH * FRAME_HEIGHT; i++) {
//        buffer[i] = LCD_ColorConvert565(255, 0, 0);  // 紅色
////        buffer[i] = RGB565_unit16(255,0,0);
//
//    }
//
//    printf("SDRAM FOR LCD_DrawBuffer...\n");
//    memcpy((void *)SDRAM_BASE_ADDRESS, buffer, FRAME_WIDTH * FRAME_HEIGHT * sizeof(LCD_Color_t));  // 將 buffer 的內容複製到 SDRAM
//    printf("LCD_Buffer TO SDRAM FINISH！\n");
//
//    printf("TEST LCD_DrawBuffer...\n");
//    LCD_DrawBuffer((LCD_Color_t *)SDRAM_BASE_ADDRESS, FRAME_WIDTH * FRAME_HEIGHT);
////    LCD_DrawBuffer(buffer, FRAME_WIDTH * FRAME_HEIGHT);
//    printf("LCD_Buffer TESTING FINISH！\n");
//
////		volatile uint16_t *sdram = (volatile uint16_t *) 0x30000000ULL;  // SDRAM 基址
////		for (int i = 0; i < FRAME_WIDTH * FRAME_HEIGHT; i++) {
////			sdram[i] = LCD_ColorConvert565(255, 0, 0);  // 紅色
////	//        buffer[i] = RGB565_unit16(255,0,0);
////
////		}
////	    printf("TEST LCD_DrawBuffer...\n");
////	    LCD_DrawBuffer(sdram, FRAME_WIDTH * FRAME_HEIGHT);
////	//    LCD_DrawBuffer(buffer, FRAME_WIDTH * FRAME_HEIGHT);
////	    printf("LCD_Buffer TESTING FINISH！\n");
//
//}
//
//void test_sdram() {
//    volatile uint16_t *sdram = (volatile uint16_t *) 0x30000000;  // SDRAM 基址
////    sdram[0] = 0xC1111111;  // 寫入測試數據
////    sdram[0] = 0x30000000ULL;  // 寫入測試數據
//    sdram[0] = 0x1234;  // 寫入測試數據
//    LCD_SetForegroundColor(ColorGreen);
//    LCD_SetBackgroundColor(ColorBlue);
//    while(1){
//    	if (sdram[0] == 0x1234) {
////    	    LCD_SetForegroundColor(ColorGreen);
////    	    LCD_SetBackgroundColor(ColorBlue);
//    		LCD_String(100, 300, "SDRAM JA");
//    	}
////    	else if(sdram[0] == 0xC1111112){
////    		LCD_String(100, 300, "SDRAM JA-1");
////    	}
//    	else {
////    	    LCD_SetForegroundColor(ColorGreen);
////    	    LCD_SetBackgroundColor(ColorBlue);
//    		LCD_String(100, 300, "SDRAM Neii");
//    	}
//    }
//}
//
//void test_lcd_rect() {
//    LCD_SetForegroundColor(ColorRed);
//    LCD_FilledRect(100, 100, 200, 200, 0);
//}
//
////void USART_ReceiveImage() {
////    HAL_UART_Receive(&huart1, (uint8_t *)uart_buffer, UART_BUFFER_SIZE, 10);
////}
////
////void test_lcd_bitmap_cam() {
////	LCD_SetDrawArea(0, 0, FRAME_WIDTH - 1, FRAME_HEIGHT - 1);  // 手動設定顯示區域
////	LCD_EnableDrawMode();
////    // 設定 SDRAM 基址
////    volatile uint16_t *buffer = (volatile uint16_t *) 0x30000000ULL;  // SDRAM 基址
////
////    for (int i = 0; i < FRAME_WIDTH; i++){
////    	for (int j = 0; j < FRAME_HEIGHT; j++){
////            uint16_t color = buffer[j * FRAME_WIDTH + i];  // 讀取 RGB565
////            uint8_t r = (color >> 11) & 0x1F;  // 提取 5-bit R
////            uint8_t g = (color >> 5) & 0x3F;   // 提取 6-bit G
////            uint8_t b = color & 0x1F;         // 提取 5-bit B
////    		LCD_SetBackgroundColor(LCD_ColorConvert(r, g, b));
////    		LCD_Character(i, j, ".");
//////    		LCD_String(i, j, ".");
////    	}
////    }
////}
//
//void USART_ReceiveImage() {
////	uint32_t image_size = CAM_FRAME_WIDTH * CAM_FRAME_HEIGHT * 3 + 1;
//	uint32_t image_size = 0;
////	SERIAL_Read(uart_buffer, sizeof(image_size), 100);
//
//	// ** 第一步：先接收 4 Bytes，取得影像大小**
//	HAL_UART_Receive(&huart1, (uint8_t *)&image_size, sizeof(image_size), HAL_MAX_DELAY);
//
//	//  確保 `image_size` 不超過 `uart_buffer` 的大小
////	if (image_size > UART_BUFFER_SIZE) {
//////		printf("[ERROR] Received image size too large: %lu\n", image_size);
////        LCD_SetBackgroundColor(ColorRed);
////        LCD_String(100, 100, "[ERROR] Received image size too large");  // 直接發送 RGB565 資料給 LCD
////	    return;
////	}
//
//	// ** 第二步：接收影像數據**
////	HAL_UART_Receive_IT(&huart1, uart_buffer, UART_BUFFER_SIZE);
//	HAL_UART_Receive_DMA(&huart1, uart_buffer, UART_BUFFER_SIZE);
//
////	for(int i = 0; i < image_size; i++){
////		HAL_UART_Receive(&huart1, uart_buffer[i], image_size, 0xFFF);
////	}
////	HAL_UART_Receive_DMA(&huart1, uart_buffer, image_size);
//
////	for (int j = 100; j < 200; j++) {
////	    for (int i = 100; i < 200; i++) {
////	//            uint16_t color = sdram_buffer[j * FRAME_WIDTH + i];  // 讀取 RGB565
////	        int index = (j * 10 + i) * 3;
////	        uint8_t b = uart_buffer[index + 0];  // B
////	        uint8_t g = uart_buffer[index + 1];  // G
////	        uint8_t r = uart_buffer[index + 2];  // R
////	        LCD_SetBackgroundColor(LCD_ColorConvert(r, g, b));
////	        LCD_Character(i, j, ".");  // 直接發送 RGB565 資料給 LCD
////	    }
////	}
//
////    uint32_t bytes_received = 0;
////    uint32_t total_bytes = CAM_FRAME_WIDTH * CAM_FRAME_HEIGHT * 3;  // BGR888，每像素 3 Bytes
////    uint8_t bgr_pixel[3];
////    uint16_t *lcd_mem = (uint16_t *)LCD_BASE_ADDRESS ;  // LCD 記憶體
////
////    for (int i = 0; i < CAM_FRAME_WIDTH * CAM_FRAME_HEIGHT; i++) {
////        // 接收 3 Bytes（BGR888）
////        HAL_UART_Receive(&huart1, bgr_pixel, 3, HAL_MAX_DELAY);
////
////        uint8_t r = bgr_pixel[0];  // B
////        uint8_t g = bgr_pixel[1];  // G
////        uint8_t b = bgr_pixel[2];  // R
////
////        // 轉換成 RGB565
////        uint16_t rgb565 = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
////
////        //寫入 LCD 記憶體
////        lcd_mem[i] = rgb565;
//////        lcd_mem[i] = (r << 16) | (g << 8) | b;
////    }
//
////    uint32_t bytes_received = 0;
////    uint32_t total_bytes = UART_BUFFER_SIZE;
////
////    while (bytes_received < total_bytes) {
////        bytes_received += HAL_UART_Receive(&huart1, uart_buffer + bytes_received, total_bytes - bytes_received, 100);
////    }
//
////    uint8_t uart_buffer[FRAME_WIDTH * FRAME_HEIGHT * 3]; // 每個像素 3 bytes
////    HAL_UART_Receive(&huart1, uart_buffer, sizeof(uart_buffer), HAL_MAX_DELAY);
//
////    volatile uint16_t *sdram_buffer = (volatile uint16_t *) 0x30000000ULL;  // SDRAM 基址
////
////    for (int i = 0; i < FRAME_WIDTH * FRAME_HEIGHT; i++) {
////        uint8_t b = uart_buffer[i * 3 + 0];  // B
////        uint8_t g = uart_buffer[i * 3 + 1];  // G
////        uint8_t r = uart_buffer[i * 3 + 2];  // R
////
////        // 轉換為 RGB565
////        LCD_ColorConvert(r, g, b);
//////        sdram_buffer[i] = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
////    }
//}
//volatile uint8_t image_ready = 0;
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//
//        // 解碼 RGB565 -> RGB888
//    for (uint32_t i = 0; i < CAM_FRAME_WIDTH * CAM_FRAME_HEIGHT; i++) {
//        uint16_t pixel = (uart_buffer[i * 2] | (uart_buffer[i * 2 + 1] << 8));
//
//        uint8_t r = (pixel & 0xF800) >> 8;
//        uint8_t g = (pixel & 0x07E0) >> 3;
//        uint8_t b = (pixel & 0x001F) << 3;
//
//        rgb_buffer[i * 3] = r;
//        rgb_buffer[i * 3 + 1] = g;
//        rgb_buffer[i * 3 + 2] = b;
//    }
//    image_ready = 1;  // 設定標誌位
//
//        // 如果你要繼續接收下一張影像，必須再次啟動 UART
//        //HAL_UART_Receive_IT(&huart1, uart_buffer, IMAGE_SIZE);
//
//}
//
//void test_lcd_bitmap_cam() {
//
////	LCD_SetBackgroundColor(ColorRed);
//
//	uint16_t X0 = (FRAME_WIDTH-CAM_FRAME_WIDTH)/2;
//	uint16_t Y0 = (FRAME_HEIGHT-CAM_FRAME_HEIGHT)/2;
//	uint16_t X1 = (FRAME_WIDTH-CAM_FRAME_WIDTH)/2 + CAM_FRAME_WIDTH -1;
//	uint16_t Y1 = (FRAME_HEIGHT-CAM_FRAME_HEIGHT)/2 + CAM_FRAME_HEIGHT -1;
//    LCD_SetDrawArea(X0, Y0, X1, Y1);
//    LCD_EnableDrawMode();
//    uint8_t *image = (uint8_t *)rgb_buffer;
////    volatile uint16_t *test_image = (volatile uint16_t *)LCD_BASE_ADDRESS;
////    for (int i = 0; i < FRAME_WIDTH * FRAME_HEIGHT; i++) {
////        test_image[i] = 3250;  // 紅色
////    }
////    LCD_DrawBitmap(test_image, X0, Y0, CAM_FRAME_WIDTH, CAM_FRAME_HEIGHT, North);
////    LCD_DrawBitmap(image, X0, Y0, CAM_FRAME_WIDTH, CAM_FRAME_HEIGHT, North);
////    LCD_String(X0, Y0, image);
//
////    for (int i = 0; i < FRAME_WIDTH * FRAME_HEIGHT; i++) {
////        LCD_Data16(uart_buffer[i]); // 直接寫入 LCD
////    }
//    for (int j = 0; j < CAM_FRAME_HEIGHT; j++) {
//        for (int i = 0; i < CAM_FRAME_WIDTH; i++) {
////            uint16_t color = sdram_buffer[j * FRAME_WIDTH + i];  // 讀取 RGB565
//            int index = (j * CAM_FRAME_WIDTH + i) * 3;
//            uint8_t b = rgb_buffer[index + 0];  // B
//            uint8_t g = rgb_buffer[index + 1];  // G
//            uint8_t r = rgb_buffer[index + 2];  // R
//            LCD_SetBackgroundColor(LCD_ColorConvert(r, g, b));
//            LCD_Character(i+X0, j+Y0, ".");  // 直接發送 RGB565 資料給 LCD
//        }
//    }
////    for (int i = 0; i < CAM_FRAME_WIDTH; i++) {
////        for (int j = 0; j < CAM_FRAME_HEIGHT; j++) {
////    //            uint16_t color = sdram_buffer[j * FRAME_WIDTH + i];  // 讀取 RGB565
////            int index = (i * CAM_FRAME_WIDTH + j) * 3;
////            uint8_t r = uart_buffer[index + 0];  // B
////            uint8_t g = uart_buffer[index + 1];  // G
////            uint8_t b = uart_buffer[index + 2];  // R
////            LCD_SetBackgroundColor(LCD_ColorConvert(r, g, b));
////            LCD_Character(i + X0, j + Y0, ".");  // 直接發送 RGB565 資料給 LCD
////        }
////    }
//}
////
//extern DMA2D_HandleTypeDef hdma2d;  // 引入 DMA2D 句柄
//
//void USART_ReceiveImage_DMA() {
//    HAL_UART_Receive(&huart1, uart_buffer, UART_BUFFER_SIZE, HAL_MAX_DELAY);
//}
//
//void Convert_BGR888_to_RGB565_DMA2D(uint8_t *src, uint16_t *dst, uint32_t width, uint32_t height) {
//    hdma2d.Init.Mode         = DMA2D_M2M_PFC;  // 設定為記憶體到記憶體，並做顏色轉換
//    hdma2d.Init.ColorMode    = DMA2D_OUTPUT_RGB565;
//    hdma2d.Init.OutputOffset = 0;
//
//    hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_RGB888;
//    hdma2d.LayerCfg[1].InputOffset = 0;
//    hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
//
//    if (HAL_DMA2D_Init(&hdma2d) != HAL_OK) {
//        Error_Handler();
//    }
//
//    if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK) {
//        Error_Handler();
//    }
//
//    // 使用 DMA2D 轉換
//    HAL_DMA2D_Start(&hdma2d, (uint32_t)src, (uint32_t)dst, width, height);
//    HAL_DMA2D_PollForTransfer(&hdma2d, HAL_MAX_DELAY);
//}
//
//void Display_Image() {
//    LCD_SetDrawArea(0, 0, FRAME_WIDTH - 1, FRAME_HEIGHT - 1);  // 設定繪製範圍
//    LCD_EnableDrawMode();
//
//    for (uint32_t i = 0; i < FRAME_WIDTH * FRAME_HEIGHT; i++) {
//        LCD_Data16(buffer[i]);  // 寫入 LCD
//    }
//}
////
//void USART_ReceiveImage_DMA2D() {
//    uint32_t image_size = 0;
//
//    //  先接收 4 Bytes 的影像大小
//    HAL_UART_Receive(&huart1, (uint8_t *)&image_size, sizeof(image_size), HAL_MAX_DELAY);
//
//    // 確保影像大小合理
//    if (image_size > UART_BUFFER_SIZE) {
//        return;  // 錯誤處理：影像過大
//    }
//
//    //  接收影像數據
//    HAL_UART_Receive(&huart1, uart_buffer, image_size, HAL_MAX_DELAY);
//}
//void test_lcd_DMA2D_cam() {
//    // 設定 LCD 繪圖區域
//    LCD_SetDrawArea(0, 0, FRAME_WIDTH - 1, FRAME_HEIGHT - 1);
//    LCD_EnableDrawMode();
//
//    // SDRAM 記憶體基址
//    uint32_t *dst = (uint32_t *)0xC0000000;
//
//    // 確保 UART 先接收到影像
//    USART_ReceiveImage();
//
//    // 使用 DMA2D 轉換影像格式 (BGR888 → RGB565)
//    DMA2D_Transfer((uint32_t *)uart_buffer, dst, FRAME_WIDTH, FRAME_HEIGHT);
//}
////void test_lcd_dma2d() {
////    uint32_t *src = (uint32_t *)uart_buffer;   // 影像資料 (RGB888)
////    uint32_t *dst = (uint32_t *)0xC0000000;    // LCD SDRAM (RGB565)
////
////    LCD_SetDrawArea(0, 0, FRAME_WIDTH - 1, FRAME_HEIGHT - 1);
////    LCD_EnableDrawMode();
////
////    // 使用 DMA2D 搬移影像
////    DMA2D_Transfer(src, dst, FRAME_WIDTH, FRAME_HEIGHT);
////}

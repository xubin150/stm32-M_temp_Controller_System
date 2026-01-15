/*
 * bsp_io.h
 *
 *  Created on: Dec 4, 2025
 *      Author: Administrator
 */

#ifndef SRC_BSP_IO_H_
#define SRC_BSP_IO_H_
#include "main.h"
#include "cmsis_os.h" // 包含 FreeRTOS/CMSIS RTOS 定义

// ====================================================
// 1. 常量定义
// ====================================================

// --- 按键定义 (位掩码) ---
#define KEY_MENU    (1 << 0)
#define KEY_UP      (1 << 1)
#define KEY_DOWN    (1 << 2)
#define KEY_OK      (1 << 3)
#define ALL_KEYS    (KEY_MENU | KEY_UP | KEY_DOWN | KEY_OK)

// --- 按键定时常量 ---
#define KEY_MENU_LONG_PRESS_MS 5000 // Menu 长按时间 5秒
#define KEY_SHORT_PRESS_MS     10   // 最小短按时间 (消抖后)

// --- PID/SV 限制常量 ---
#define MAX_SETPOINT 200.0f
#define MIN_SETPOINT 20.0f

// ====================================================
// 2. 函数声明
// ====================================================

/**
 * @brief 读取按键状态
 * @return 激活按键的位掩码 (KEY_MENU | KEY_UP | ...)
 */
uint32_t KEY_Read_State(void);

/**
 * @brief 获取短按事件 (一次性触发)
 * @param key_code: 目标按键的位掩码 (e.g., KEY_MENU)
 * @return 1: 短按已发生且未被读取; 0: 未发生
 */
uint32_t Key_Get_ShortPress(uint32_t key_code);

/**
 * @brief 检查长按事件是否达到指定时间
 * @param key_code: 目标按键的位掩码
 * @param ms: 指定的长按持续时间 (毫秒)
 * @return 1: 持续时间达到; 0: 未达到
 */
uint32_t KEY_Get_LongPress(uint32_t key_code, uint32_t ms);

/**
 * @brief 重置指定按键的长按状态
 * @param key_code: 目标按键的位掩码
 */
void KEY_Reset_LongPress(uint32_t key_code);

// --- LED/蜂鸣器控制函数 ---

/**
 * @brief 设置红色 LED 状态
 * @param on: 1 亮, 0 灭 (低电平有效)
 */
void LED_Red_Set(uint8_t on);

/**
 * @brief 设置绿色 LED 状态
 * @param on: 1 亮, 0 灭 (低电平有效)
 */
void LED_Green_Set(uint8_t on);

void LED_Green_blink(void);
/**
 * @brief 启动蜂鸣器
 * @note (发送信号给 Beep_Task)。
 */
void Beep_Start(void);

/**
 * @brief 停止蜂鸣器 (直接操作GPIO关闭)
 */
void Beep_Stop(void);

#endif /* SRC_BSP_IO_H_ */

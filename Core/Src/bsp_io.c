/*
 * bsp_io.c
 *
 *  Created on: Dec 4, 2025
 *      Author: Administrator
 */
#include "bsp_io.h"


// 引用在 app_freertos.c 中定义的信号量句柄
extern osSemaphoreId_t beep_signal_sem;

// ====================================================
// 1. 按键状态机内部定义
// ====================================================

// 按键管脚配置 (假设 CubeMX 已生成，这里仅作逻辑映射)
// 实际工程中应使用宏定义或 PinName
#define MENU_PORT       GPIOD
#define MENU_PIN        GPIO_PIN_0
#define UP_PORT         GPIOD
#define UP_PIN          GPIO_PIN_1
#define DOWN_PORT       GPIOD
#define DOWN_PIN        GPIO_PIN_2
#define OK_PORT         GPIOD
#define OK_PIN          GPIO_PIN_3



#define RED_LED_PORT    GPIOB
#define RED_LED_PIN     GPIO_PIN_8
#define GREEN_LED_PORT  GPIOB
#define GREEN_LED_PIN   GPIO_PIN_9
#define DEBOUNCE_MS 1   // 20ms足够

// 内部状态跟踪结构体，每个按键一个实例
typedef struct {
    uint32_t press_start_tick; // 按下时的 FreeRTOS Tick
    uint8_t  is_pressed_raw;   // 原始按下状态 (用于消抖)
    uint8_t  is_long_press_read; // 长按是否已被读取 (防止重复触发)
    uint8_t  is_short_press_pending; // 短按是否挂起 (一次性触发)
    uint8_t stable_state;         // 稳定按键状态（0=松开，1=按下）
    uint32_t debounce_tick;       // 用于消抖计时
} KeyState_t;

// 跟踪所有按键的状态
static KeyState_t key_states[4] = {0}; // 0:MENU, 1:UP, 2:DOWN, 3:OK

// 将 KEY_CODE 位掩码映射到 key_states 数组的索引
static int getKeyIndex(uint32_t key_code) {
    if (key_code & KEY_MENU) return 0;
    if (key_code & KEY_UP)   return 1;
    if (key_code & KEY_DOWN) return 2;
    if (key_code & KEY_OK)   return 3;
    return -1;
}

// ====================================================
// 2. 按键驱动函数实现
// ====================================================

/**
 * @brief 原始按键读取 (含消抖处理)
 * @note 必须在周期性任务中调用 (e.g., Key_Task, 10-100ms周期)
 * @return 激活按键的位掩码
 */
uint32_t KEY_Read_State(void)
{
    uint32_t raw_state = 0;
    uint32_t current_state = 0;
    uint32_t current_tick = osKernelGetTickCount();

    // 由于按键是上拉，低电平有效，所以读取后需要取反
    // raw_state 位0:MENU, 位1:UP, 位2:DOWN, 位3:OK
    if (HAL_GPIO_ReadPin(MENU_PORT, MENU_PIN) == GPIO_PIN_RESET) raw_state |= KEY_MENU;
    if (HAL_GPIO_ReadPin(UP_PORT, UP_PIN)     == GPIO_PIN_RESET) raw_state |= KEY_UP;
    if (HAL_GPIO_ReadPin(DOWN_PORT, DOWN_PIN) == GPIO_PIN_RESET) raw_state |= KEY_DOWN;
    if (HAL_GPIO_ReadPin(OK_PORT, OK_PIN)     == GPIO_PIN_RESET) raw_state |= KEY_OK;

    for (int i = 0; i < 4; i++) {
        uint32_t key_code = (1 << i);
        KeyState_t *state = &key_states[i];
        uint8_t raw = (raw_state & key_code) ? 1 : 0;   // 当前硬件读数
        // 消抖机制
        if (raw != state->stable_state) {
            // 状态变化 → 开始计时
            if (state->debounce_tick == 0) {
                state->debounce_tick = current_tick;
            }
            // 达到消抖时间 → 更新稳定状态
            else if ((current_tick - state->debounce_tick) >= DEBOUNCE_MS) {
                state->stable_state = raw;
                state->debounce_tick = 0;
            }
        } else {
            // 状态一致，清除计时
            state->debounce_tick = 0;
        }
        // 下面开始用 **stable_state** 代替未消抖的 raw 输入
		uint8_t stable = state->stable_state;
        // 1. 判断当前按键是否按下
        if (stable) {
            // 按下
            if (state->is_pressed_raw == 0) {
                // 首次按下或刚消抖完成
                state->press_start_tick = current_tick;
                state->is_pressed_raw = 1;
                state->is_long_press_read = 0; // 只要按下，长按标记就重置
            }
            // 只要是按下状态，就返回当前状态
            current_state |= key_code;

        } else {
            // 释放
            if (state->is_pressed_raw == 1) {
                // 刚释放
                uint32_t duration = current_tick - state->press_start_tick;

                // 检查是否为有效的短按 (忽略极短的抖动和被长按占用的情况)
                if (duration >= KEY_SHORT_PRESS_MS) {
                    // 如果不是长按（未触发或未被读取），则触发短按
                    if (state->is_long_press_read == 0) {
                        state->is_short_press_pending = 1; // 挂起短按事件
                    }
                }

                // 重置按下状态
                state->is_pressed_raw = 0;
            }
        }
    }

    return current_state;
}


/**
 * @brief 检查长按事件是否达到指定时间
 * @param key_code: 目标按键的位掩码
 * @param ms: 指定的长按持续时间 (毫秒)
 * @return 1: 持续时间达到; 0: 未达到
 */
uint32_t KEY_Get_LongPress(uint32_t key_code, uint32_t ms)
{
    int index = getKeyIndex(key_code);
    if (index == -1) return 0;

    KeyState_t *state = &key_states[index];
    uint32_t current_tick = osKernelGetTickCount();

    // 1. 必须处于按下状态
    // 2. 长按事件未被读取
    // 3. 持续时间达到要求
    if (state->is_pressed_raw &&
        !state->is_long_press_read &&
        (current_tick - state->press_start_tick) >= pdMS_TO_TICKS(ms))
    {
        state->is_long_press_read = 1; // 标记已触发
        return 1;
    }
    return 0;
}


/**
 * @brief 重置指定按键的长按状态
 * @param key_code: 目标按键的位掩码
 */
void KEY_Reset_LongPress(uint32_t key_code)
{
    int index = getKeyIndex(key_code);
    if (index != -1) {
        key_states[index].is_long_press_read = 0;
    }
}


/**
 * @brief 获取短按事件 (一次性触发)
 * @return 1: 短按已发生且未被读取; 0: 未发生
 */
uint32_t Key_Get_ShortPress(uint32_t key_code)
{
    int index = getKeyIndex(key_code);
    if (index == -1) return 0;

    // 只有当挂起标志为1时才返回1
    if (key_states[index].is_short_press_pending) {
        key_states[index].is_short_press_pending = 0; // 读取后立即清零，实现一次性触发
        return 1;
    }
    return 0;
}

// ====================================================
// 3. LED/蜂鸣器驱动函数实现
// ====================================================

/**
 * @brief 设置红色 LED 状态
 * @param on: 1 亮, 0 灭 (低电平有效)
 */
void LED_Red_Set(uint8_t on) {
    // LED 是低电平亮，所以 on=1 对应 GPIO_PIN_RESET
    HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/**
 * @brief 设置绿色 LED 状态
 * @param on: 1 亮, 0 灭 (低电平有效)
 */
void LED_Green_Set(uint8_t on) {
    // LED 是低电平亮，所以 on=1 对应 GPIO_PIN_RESET
    HAL_GPIO_WritePin(GREEN_LED_PORT, GREEN_LED_PIN, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void LED_Green_blink(void)
{
	 HAL_GPIO_TogglePin(GREEN_LED_PORT, GREEN_LED_PIN);
}
/**
 * @brief 启动蜂鸣器
 */
void Beep_Start(void)

{
    if (beep_signal_sem != NULL) {
        // 使用 osSemaphoreRelease 发送信号，将 Beep_Task 从阻塞中唤醒
        osSemaphoreRelease(beep_signal_sem);
    }
    //HAL_GPIO_WritePin(BEEP_PORT, BEEP_PIN, GPIO_PIN_SET);
    // 在实际应用中，Beep_Task 会在短时间后调用 Beep_Stop 来关闭它，以实现短暂的提示音。
}

/**
 * @brief 停止蜂鸣器
 */
void Beep_Stop(void) {
    // 蜂鸣器是高电平叫
    HAL_GPIO_WritePin(BEEP_PORT, BEEP_PIN, GPIO_PIN_RESET);
}

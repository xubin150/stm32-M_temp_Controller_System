/*
 * bsp_autotune.h
 *
 *  Created on: Dec 2, 2025
 *      Author: Administrator
 */

#ifndef SRC_BSP_AUTOTUNE_H_
#define SRC_BSP_AUTOTUNE_H_
#include "main.h"
#include <stdint.h>

typedef enum {
    AT_MODE_STANDARD = 0,   // 标准 Ziegler-Nichols (0.6 * Ku)
    AT_MODE_MODERATE,       // 适度超调 (0.33 * Ku)
    AT_MODE_NO_OVERSHOOT    // 无超调 (0.2 * Ku)
} AT_CalcMode_t;

typedef enum {
    AUTOTUNE_IDLE = 0,
    AUTOTUNE_RUNNING,
    AUTOTUNE_COMPLETE,
    AUTOTUNE_FAILED
} AT_Status;

typedef struct {
    // 配置参数
    float Target;           // 目标温度
    float OutputStep;       // 继电器跳变步长 (如 PWM 为 0-1000, 该值可设为 300)
    float BaseOutput;       // 继电器基础输出 (如 200)
    float Hysteresis;       // 滞后带 (防止噪声导致频繁切换, 建议 0.5-1.0)

    AT_CalcMode_t CalcMode; // 当前选择的整定计算模式
    // 运行状态
    AT_Status Status;
    int8_t  Direction;      // 当前加热方向: 1-加热, -1-停止
    uint32_t PeakCount;     // 已检测到的波峰数量
    uint32_t LastPeakTime;  // 上一次波峰的时间 (ms)

    // 测量值
    float MaxTemp;          // 当前周期内的最高温
    float MinTemp;          // 当前周期内的最低温
    float Amplitude;        // 平均振荡幅值
    float Ku;               // 临界增益
    float Pu;               // 临界周期 (秒)

    int8_t LastDirection; //加热/停止加热方向

    // 结果 (供外部读取)
    float SuggestedKp;
    float SuggestedKi;
    float SuggestedKd;
} PID_AutotuneTypeDef;

// 初始化自整定结构体
void PID_Autotune_Init(PID_AutotuneTypeDef *at, float target, float base_out, float step);

// 自整定核心循环 (建议每 100ms-500ms 调用一次)
float PID_Autotune_Process(PID_AutotuneTypeDef *at, float current_temp, uint32_t tick_ms);
#endif /* SRC_BSP_AUTOTUNE_H_ */

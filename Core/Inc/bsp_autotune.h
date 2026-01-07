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
typedef struct {
    float Kp;
    float Ki;
    float Kd;
} PID_Params;

/**
 * 离线自整定（FOPDT 识别 + 保守 IMC 风格参数计算）
 *
 * read_temp(): 返回当前温度 (°C)
 * set_heater(power): 设置加热功率，范围 0.0 .. 1.0
 *
 * step_power: 施加的阶跃功率变化量 (例如 0.1 表示 +10% 功率)
 * base_power: 施加阶跃前的基准功率 (0..1)
 * duration_ms: 总采样时间（建议至少 30s ~ 120s，取决于热系统）
 *
 * 返回：0 表示成功并写入 params；非 0 表示失败（例如无响应或超时）
 */
int PID_AutoTune(
    float (*read_temp)(void),
    void  (*set_heater)(float),
    PID_Params *params,
    float step_power,
    float base_power,
    uint32_t duration_ms,
    float power_max,         // 最大允许功率 (0..1)
    float max_safe_temp);    // 超温保护阈值 (°C)

int PID_AutoTune_SSR(PID_Params *out_params, float step_power, float base_power,
                     uint32_t duration_ms, float power_max, float max_safe_temp);
#endif /* SRC_BSP_AUTOTUNE_H_ */

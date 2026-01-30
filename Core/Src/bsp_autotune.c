/*
 * bsp_autotune.c
 *
 *  Created on: Dec 2, 2025
 *      Author: Administrator
 */

#include "bsp_autotune.h"
#include <string.h>
#include <math.h>
#include "cmsis_os2.h" // osDelay, osKernelGetTickCount 等
#include <stdio.h>


// 你的开关量加热函数
extern void set_heater_impl(uint8_t on);

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// 建议增加一个启动时间记录，用于超时检测
static uint32_t g_at_start_tick = 0;

PID_AutotuneTypeDef myAT;
uint8_t is_autotuning = 0; // 标志位

void PID_Autotune_Init(PID_AutotuneTypeDef *at, float target, float base_out, float step) {
    at->Target = target;
    at->BaseOutput = base_out;
    at->OutputStep = step;

    at->Hysteresis = 0.5f;
    at->Status = AUTOTUNE_RUNNING;
    at->Direction = 0;
    at->PeakCount = 0;
    at->LastDirection = 0;
    at->MaxTemp = -999.0f;
    at->MinTemp = 999.0f;
    at->LastPeakTime = 0;
    at->Pu = 0;
    at->Amplitude = 0;

    g_at_start_tick = osKernelGetTickCount();
}

/**
 * @brief  继电器反馈法核心逻辑
 * @return 返回当前应该输出的 PWM 值
 */
float PID_Autotune_Process(PID_AutotuneTypeDef *at, float current_temp, uint32_t tick_ms) {
    if (at->Status != AUTOTUNE_RUNNING) {
        return at->BaseOutput;
    }

    // 0. 安全超时检查 (例如 20分钟强制退出)
/*    if (tick_ms - g_at_start_tick > 1200000) {
        at->Status = AUTOTUNE_FAILED;
        return at->BaseOutput;
    }*/

    // 1. 继电器切换逻辑 (带 Hysteresis 抗噪)
    if (at->Direction <= 0 && current_temp < (at->Target - at->Hysteresis)) {
        at->Direction = 1; // 开始加热
    } else if (at->Direction >= 0 && current_temp > (at->Target + at->Hysteresis)) {
        at->Direction = -1; // 停止加热
    }

    // 2. 更新当前半波内的极值
    if (current_temp > at->MaxTemp) at->MaxTemp = current_temp;
    if (current_temp < at->MinTemp) at->MinTemp = current_temp;

    if (at->LastDirection != at->Direction && at->Direction != 0) {
        at->PeakCount++;

        /* * 逻辑调整：
         * PeakCount == 1: 第一次越过设定点（从冷态爬升），舍弃。
         * PeakCount == 2: 第一次反向跳变，LastPeakTime 获得有效的第一个参考点。
         * PeakCount >= 3: 开始计算半周期。
         */
        if (at->PeakCount >= 3) {
            if (at->LastPeakTime > 0) {
                // 计算当前半周期的持续时间 (秒)
                float half_period = (float)(tick_ms - at->LastPeakTime) / 1000.0f;
                float current_Pu = half_period * 2.0f; // 转换为完整周期

                // 计算当前半周期的振幅 A
                float current_amp = (at->MaxTemp - at->MinTemp) / 2.0f;

                // 只有当振荡趋于稳定时（第5次跳变后），才开始滤波计入结果
                if (at->PeakCount > 5) {
                    if (at->Pu == 0) at->Pu = current_Pu;
                    else at->Pu = at->Pu * 0.4f + current_Pu * 0.6f;

                    if (at->Amplitude == 0) at->Amplitude = current_amp;
                    else at->Amplitude = at->Amplitude * 0.4f + current_amp * 0.6f;
                }
            }

        }
        // 重要：每次方向切换时，都要更新参考时间并重置极值记录
        at->LastPeakTime = tick_ms;
        at->MaxTemp = current_temp; // 重置为当前温度，开始下一半波的寻优
        at->MinTemp = current_temp;

       // last_dir = at->Direction;
        at->LastDirection = at->Direction;
    }

    // 3. 停止条件: 记录足够多的稳定周期
    // 增加到 10次以确保经过了至少 3个稳定的滤波周期
    if (at->PeakCount >= 10 && at->Amplitude > 0.1f) {
        // 计算 Ku = 4d / (pi * A)
        at->Ku = (4.0f * at->OutputStep) / (M_PI * at->Amplitude);
        at->Ku =  at->Ku * 2 ;// 系数修正
        // 定义不同模式下的系数 (Coefficients)
		float kp_coeff = 0.6f;
		float ti_coeff = 0.5f;
		float td_coeff = 0.125f;
		// 引入多模式系数选择
	switch (at->CalcMode) {
		case AT_MODE_MODERATE: // 适度超调 (Pessen Integral Rule 变体)
			kp_coeff = 0.33f; // 约为标准值的 1/2~1/3

			break;

		case AT_MODE_NO_OVERSHOOT: // 无超调 (Tyreus-Luyben 变体)
			//kp_coeff = 0.20f; // 约为标准值的 1/3
			kp_coeff = 0.33f; //使用适度超调的参数  ->对于大迟滞温控系统很难不超调
			break;

		case AT_MODE_STANDARD:
		default: // 标准 Ziegler-Nichols
			kp_coeff = 0.6f;
			break;
	}
        // Ziegler-Nichols 公式计算
        at->SuggestedKp = kp_coeff * at->Ku;

        // 保护：防止 Pu 为 0 导致除以 0 崩溃
        if (at->Pu > 0) {
            float Ti = ti_coeff * at->Pu;
            float Td = td_coeff * at->Pu;
            at->Pu = at->Pu*0.2;

            at->SuggestedKi = at->SuggestedKp / Ti;
            at->SuggestedKd = at->SuggestedKp * Td;
            at->Status = AUTOTUNE_COMPLETE;
        } else {
            at->Status = AUTOTUNE_FAILED;
        }
    }

    // 返回继电器输出：基础值 + 步长偏移
    return (at->Direction > 0) ? (at->BaseOutput + at->OutputStep) : (at->BaseOutput - at->OutputStep);
}




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

#define MAX_SAMPLES 300

// 你的开关量加热函数
extern void set_heater_impl(uint8_t on);
extern float read_temp_impl(void); // 读取当前温度

static uint32_t g_tpc_on_time_ms = 0;
static uint32_t g_tpc_period_ms = TPC_PERIOD_MS;
static uint32_t g_tpc_start_tick = 0;
static void tpc_set_heater_top(float unused)
{
    if (g_tpc_start_tick == 0) g_tpc_start_tick = osKernelGetTickCount();
    uint32_t t = osKernelGetTickCount() - g_tpc_start_tick;
    uint32_t phase = t % g_tpc_period_ms;
    if (phase < g_tpc_on_time_ms) {
        set_heater_impl(1);
    } else {
        set_heater_impl(0);
    }
}

// 小工具：指数滤波，alpha in (0,1)
static float exp_filter(float prev, float in, float alpha) {
    return prev + alpha * (in - prev);
}

int PID_AutoTune(
    float (*read_temp)(void),
	void  (*set_heater)(float), // 更改参数类型以适配 TPC 模拟
    PID_Params *params,
    float step_power,
    float base_power,
    uint32_t duration_ms,
    float power_max,
    float max_safe_temp)
{
    if (!read_temp || !set_heater || !params) return -1;
    if (step_power <= 0.0f) return -2;
    if (base_power < 0.0f || base_power > 1.0f) return -3;
    if (power_max <= 0.0f || power_max > 1.0f) return -4;

    // 限制 step_power 不要太大（避免瞬时过冲）
    if (step_power > 0.3f) step_power = 0.3f;

    // 保证施加功率不超过 power_max
    float applied = base_power + step_power;
    if (applied > power_max) applied = power_max;

    uint32_t sample_time_ms = TEMP_PERIOD_MS;
    uint32_t sample_count = duration_ms / sample_time_ms;
    if (sample_count < 10) return -5;
    if (sample_count > MAX_SAMPLES) sample_count = MAX_SAMPLES;

    float temps[MAX_SAMPLES];
    float times[MAX_SAMPLES];

    // 1) 记录基准温度（取平均几次）
    float base_temp = 0.0f;
    const int base_reads = 5;
    for (int i=0;i<base_reads;i++) {
        base_temp += read_temp();
        osDelay(pdMS_TO_TICKS(sample_time_ms));
    }
    base_temp /= (float)base_reads;

    // 2) 施加阶跃：设置到 applied（限幅）
   // set_heater(applied);  <-- 移除此行，首次设置交给循环内部

    // 3) 采样（并做简单指数滤波以降噪）
    float filt = base_temp;
    float alpha = 0.2f; // 指数滤波强度，0.2为较强平滑
  //  uint32_t start_tick = osKernelGetTickCount();
    for (uint32_t i=0; i<sample_count; i++) {
    	// *** 修正: 周期性调用 set_heater，允许 TPC 逻辑更新 SSR 状态 ***
		set_heater(applied);

		osDelay(pdMS_TO_TICKS(sample_time_ms));
        float now = read_temp();

        // 超温保护
        if (now >= max_safe_temp) {
            set_heater(base_power); // 立即恢复
            set_heater_impl(0);   // 确保硬件关闭
            return -10; // 超温中止
        }

        // 指数滤波
        filt = exp_filter(filt, now, alpha);
        temps[i] = filt;
        times[i] = (float)((i+1) * sample_time_ms) / 1000.0f;
    }

    // 4) 恢复基准功率（安全）
    set_heater(base_power);
    set_heater_impl(0);     // 确保 SSR 立即关闭，然后由 Heat_Task 接管

    // 5) 识别：计算 steady temperature, K, R (max slope), L
    float T_final = temps[sample_count-1];
    float K = (T_final - base_temp) / (applied - base_power + 1e-9f); // 系统静态增益 °C per unit-power

    // 5.1 计算斜率（用平滑后的点差比）
    float max_slope = 0.0f;
    uint32_t idx_max = 0;
    for (uint32_t i=1; i<sample_count; i++) {
        float dt = times[i] - times[i-1];
        if (dt <= 0.0f) continue;
        float slope = (temps[i] - temps[i-1]) / dt; // °C/s
        if (slope > max_slope) {
            max_slope = slope;
            idx_max = i;
        }
    }

    if (max_slope < 1e-4f) {
        // 系统没有明显响应
        return -6;
    }

    // 5.2 计算 L (dead time)：交点法
    float t_at_max = times[idx_max];
    float temp_at_max = temps[idx_max];
    float L = t_at_max - (temp_at_max - base_temp) / (max_slope + 1e-9f);
    if (L < 0.0f) L = 0.0f;

    // 5.3 估算时间常数 T (基于 R ≈ K/T => T ≈ K / R)
    float T = K / (max_slope + 1e-9f);
    if (T <= 0.0f) T = (times[sample_count-1] - L);

    // 6) 选择闭环时间 λ（保守策略）
    // 选取 lambda = max(3*L, 5s) 保守且足够稳定
    float lambda = 3.0f * L;
    if (lambda < 5.0f) lambda = 5.0f;

    // 7) IMC/λ-调参（比较保守，适合温控）
    // Kp = T / (K * (lambda + L))
    // Ki = Kp / T
    // Kd = Kp * 0.5 * L
    float Kp = (T) / (K * (lambda + L + 1e-9f));
    float Ki = Kp / (T + 1e-9f);
    float Kd = Kp * 0.5f * L;

    // 8) 边界保护：避免太大系数
    if (Kp < 0.0f) Kp = 0.0f;
    if (Ki < 0.0f) Ki = 0.0f;
    if (Kd < 0.0f) Kd = 0.0f;

    // 设置最大上限（工程经验值，可调整）
    if (Kp > 1000.0f) Kp = 1000.0f;
    if (Ki > 100.0f) Ki = 100.0f;
    if (Kd > 100.0f) Kd = 100.0f;

    params->Kp = Kp;
    params->Ki = Ki;
    params->Kd = Kd;

    return 0; // 成功
}


/**
 * @brief  适配 SSR TPC 的 PID 自整定调用
 */
int PID_AutoTune_SSR(PID_Params *out_params, float step_power, float base_power,
                     uint32_t duration_ms, float power_max, float max_safe_temp)
{
    if (!out_params) return -1;

    // 将原来的 set_heater 转换为时间比例开关量
    float target_power = base_power + step_power;
    if (target_power > power_max) target_power = power_max;
    if (target_power > 1.0f) target_power = 1.0f;
    if (target_power < 0.0f) target_power = 0.0f;

 //   g_tpc_on_time_ms = (uint32_t)(target_power * TPC_PERIOD_MS);
 //   g_tpc_start_tick = 0; // 重新启动计时基点
	// 调用原 PID_AutoTune，传入 TPC 模拟函数
	int ret = PID_AutoTune(read_temp_impl, tpc_set_heater_top, out_params,
					   step_power, base_power, duration_ms, power_max, max_safe_temp);

	// 调整结束后，确保加热器安全关闭
	set_heater_impl(0);

	return ret;
}



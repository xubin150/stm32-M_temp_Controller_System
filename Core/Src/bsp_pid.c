/*
 * bsp_pid.c
 *
 *  Created on: Dec 2, 2025
 *      Author: Administrator
 */

#include "bsp_pid.h"
#include <math.h>
#include <stdio.h>
float i_band = 10;
extern osMessageQueueId_t UartQueueHandle; // 串口打印队列
void PID_Init(PID_HandleTypeDef *pid, float Kp, float Ki, float Kd, float out_min, float out_max)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->prev_error = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->d_filtered = 0.0f;
    pid->integral = 0.0f;
    pid->last_out = 0.0f;

    pid->out_min = out_min;
    pid->out_max = out_max;
}

float PID_Compute(PID_HandleTypeDef *pid, float setpoint, float measurement, float dt)
{
    if (dt <= 0.0f)
	{
    	pid->prev_measurement = measurement;
    	return pid->last_out;
	}
    /**/
    float error = setpoint - measurement;

    /* ---------- P ---------- */
	float P = pid->Kp * error;

/* ---------- I (with enable & anti-windup) ---------- */
	uint8_t integral_enable = 1;

	// 在 PID_Compute 内部根据目标温度调整 i_band
	float dynamic_i_band = i_band;
	if (setpoint > 150.0f) {
	    dynamic_i_band = 10.0f; // 高温提前介入积分
	} else {
	    dynamic_i_band = 5.0f;  // 低温推迟介入防止过冲
	}

	// 误差窗口限制 还在升温阶段不积分
		if (error > dynamic_i_band) {  // 从fabsf(error) >i_band 改为 if (error > i_band)
		integral_enable = 0;
	}

	// ② 输出饱和：继续积分只会 windup
	if ((pid->last_out >= pid->out_max && error > 0) ||
	        (pid->last_out <= pid->out_min && error < 0)) {
	        integral_enable = 0;
	    }

    // Integral with anti-windup (clamp integration)
	if (integral_enable){
		pid->integral += error * dt;
	}

	if (pid->Ki > 1e-9f) {
	        float max_int = pid->out_max / pid->Ki;
	        if (pid->integral > max_int) pid->integral = max_int;
	        if (pid->integral < -max_int) pid->integral = -max_int;
	    }

		float Ki_eff = pid->Ki;
		float abs_err = fabsf(error);
		if (abs_err < 3.0f) {
		        Ki_eff = pid->Ki * (1.0f + (3.0f - abs_err) / 3.0f * 0.3f);
		    }
		if (!integral_enable) {
		    Ki_eff = pid->Ki;
		}
	    float I = Ki_eff* pid->integral;

	    /* ---------- I 微泄放（改进 2️） ---------- */
	    if (abs_err < 0.5f && error > 0) {
	        pid->integral *= 0.999f;   // 极慢泄放，防慢振荡
	    }

	    /* ---------- D (on measurement, 推荐用于温控) ---------- */
	    float raw_d = -(measurement - pid->prev_measurement) / dt;
	    pid->prev_measurement = measurement;
	    /* 一阶低通滤波 */
	    float alpha = 0.2f;   // 0.1~0.3 之间都可以
	    pid->d_filtered += alpha * (raw_d - pid->d_filtered);

	    /* ---------- D 衰减（改进 1️） ---------- */
	    float D_gain = pid->Kd;
	    if (abs_err < 1.0f) {
	        D_gain *= abs_err;   // 越接近目标，D 越小
	    }

	    float D =D_gain * pid->d_filtered;
	    /* ---------- Output ---------- */
	    float out = P + I + D;

	    // 2.6 打印调试信息
	/*	char msg[128];
		snprintf(msg, sizeof(msg), "P: %.6f, I: %.6f, D: %.6f\r\n",
				 P,I,D);
		osMessageQueuePut(UartQueueHandle, &msg, 0, pdMS_TO_TICKS(10));*/

	    if (out > pid->out_max) out = pid->out_max;
	    if (out < pid->out_min) out = pid->out_min;

	    pid->last_out = out;
	    pid->prev_error = error;



    return out;
}

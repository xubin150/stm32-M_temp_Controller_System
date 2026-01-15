/*
 * bsp_pid.h
 *
 *  Created on: Dec 2, 2025
 *      Author: Administrator
 *      PID  加热控温驱动程序
 */

#ifndef SRC_BSP_PID_H_
#define SRC_BSP_PID_H_
#include "main.h"
#include <stdint.h>

typedef struct {
    float Kp;           // 比例增益 (Proportional Gain)
    float Ki;           // 积分增益 (Integral Gain)
    float Kd;           // 微分增益 (Derivative Gain)


    float prev_error;   // 上一次的误差值 (e[k-1])，用于计算微分项
    float prev_measurement;
    float integral;     // 积分累加值，用于计算积分项
    float last_out;
    float out_min;      // 输出限制的最小值 (例如 0.0)
    float out_max;      // 输出限制的最大值 (例如 1.0 或 PWM 最大值)
    float d_filtered; //低通滤波状态
} PID_HandleTypeDef;

/**
 * @brief  PID 控制器初始化函数。
 * @param  pid:       指向 PID_HandleTypeDef 结构体的指针。
 * @param  Kp:        比例增益 Kp。
 * @param  Ki:        积分增益 Ki。
 * @param  Kd:        微分增益 Kd。
 * @param  out_min:   输出信号的最小值限制 (防止执行器反向或超出物理限制)。
 * @param  out_max:   输出信号的最大值限制 (防止输出功率过高)。
 * @retval None
 */
void PID_Init(PID_HandleTypeDef *pid, float Kp, float Ki, float Kd, float out_min, float out_max);

/**
 * @brief  PID 控制核心计算函数。
 * 根据设定点和当前测量值计算新的控制输出。
 * @param  pid:        指向 PID_HandleTypeDef 结构体的指针。
 * @param  setpoint:   系统期望达到的目标值/设定点 (SP)。
 * @param  measurement: 当前时刻系统的实际测量值/被控变量 (PV)。
 * @param  dt:         两次调用此函数之间的时间间隔 (秒，例如 0.01s)。
 * 用于计算积分项 (Ki * error * dt) 和微分项 (Kd * dError / dt)。
 * @retval float:      计算出的控制输出值，位于 [out_min, out_max] 之间。
 */
float PID_Compute(PID_HandleTypeDef *pid, float setpoint, float measurement, float dt);

#endif /* SRC_BSP_PID_H_ */

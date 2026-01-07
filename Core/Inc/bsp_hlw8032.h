/*
 * bsp_hlw8032.h
 *
 *  Created on: Dec 3, 2025
 *      Author: Administrator
 */

#ifndef SRC_BSP_HLW8032_H_
#define SRC_BSP_HLW8032_H_
#include "main.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
// --- 宏定义 ---
#define HLW8032_FRAME_SIZE      24 // 帧长度 (Reg 1 到 Reg 11 总计 24 字节)
#define HLW8032_REG_BYTES       3  // 3 字节寄存器长度

// 关键数据寄存器在 24 字节帧中的起始字节偏移
// Reg 3 (U_Parameter REG, 3B): 偏移 2
#define HLW_REG_RMS_U_PARAM_OFFSET    2
// Reg 4 (Voltage REG, 3B): 偏移 5
#define HLW_REG_RMS_U_OFFSET          5
// Reg 5 (I_Parameter REG, 3B): 偏移 8
#define HLW_REG_RMS_I_PARAM_OFFSET    8
// Reg 6 (Current REG, 3B): 偏移 11
#define HLW_REG_RMS_I_OFFSET          11
// Reg 7 (P_Parameter REG, 3B): 偏移 14
#define HLW_REG_ACTIVE_P_PARAM_OFFSET 14
// Reg 8 (Power REG, 3B): 偏移 17
#define HLW_REG_ACTIVE_P_OFFSET       17
// Reg 11 (CheckSum REG, 1B): 偏移 23
#define HLW_CHECKSUM_OFFSET           23


// --- 数据结构 ---

// HLW8032 测量数据结构 (已校准值)
typedef struct {
    float Voltage; // V (电压)
    float Current; // A (电流)
    float Power;   // W (有功功率)
    uint32_t RawVoltage; // 原始电压计数 (Reg 4)
    uint32_t RawCurrent; // 原始电流计数 (Reg 6)
    uint32_t RawPower;   // 原始功率计数 (Reg 8)
    uint32_t RawU_Param; // 电压参数寄存器 (Reg 3)
    uint32_t RawI_Param; // 电流参数寄存器 (Reg 5)
    uint32_t RawP_Param; // 功率参数寄存器 (Reg 7)
} HLW8032_Value_t;

// HLW8032 驱动句柄
typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t RxBuffer[HLW8032_FRAME_SIZE]; // DMA 接收缓冲区
    HLW8032_Value_t Data;        // 存储最新的校准数据（由任务更新）
} HLW8032_Handle_t;


// --- 外部 FreeRTOS 对象 (用于任务同步) ---

// 用于 DMA 接收完成的信号量 (由 ISR 释放，由 Power_Task 获取)
extern osSemaphoreId_t hlw8032_data_ready_sem;


// --- 函数声明 ---

/**
 * @brief 初始化 HLW8032 驱动，配置 UART 和 FreeRTOS 信号量
 * @param huart: UART 句柄 (应为 huart3)
 */
void HLW8032_Init(UART_HandleTypeDef *huart);

/**
 * @brief 启动 UART 接收 (必须在每次接收完成后重新调用)
 */
void HLW8032_StartReceive(void);

/**
 * @brief 在任务上下文中处理接收到的数据 (校验、解析、计算浮点值)
 * @note 必须在 Power_Task 中，在接收到信号量后调用此函数。
 */
void HLW8032_ProcessAndStore(void);

/**
 * @brief 获取最新的 HLW8032 测量数据
 * @retval HLW8032_Value_t 结构体 (注意：返回共享数据，读取时应考虑互斥)
 */
HLW8032_Value_t HLW8032_GetData(void);

/**
 * @brief UART 接收完成后的内部处理函数 (在 ISR 上下文中运行)
 */
void HLW8032_RxCplt(UART_HandleTypeDef *huart);


#endif /* SRC_BSP_HLW8032_H_ */

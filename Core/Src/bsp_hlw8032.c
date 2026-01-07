#include "bsp_hlw8032.h"
#include <string.h>
#include <math.h>
#include <stdio.h>


// --- FreeRTOS 对象定义 ---
// 信号量句柄定义 (在 hlw8032.h 中已声明为 extern)
osSemaphoreId_t hlw8032_data_ready_sem = NULL;

// --- 外部变量引用 ---
extern UART_HandleTypeDef huart3;

// --- 静态变量 ---
static HLW8032_Handle_t hlw8032_h;
// 用于 ISR 向任务传递数据的临时缓冲区 (确保 RxFrameCopy 永远不会被 ISR 和任务同时写入)
 uint8_t RxFrameCopy[HLW8032_FRAME_SIZE];

// --- 系统校准系数 (用于动态计算，请替换为您的实际值) ---
// ** 警告：以下系数是示例占位符，必须根据您的硬件和芯片手册进行校准！**
#define SYSTEM_U_COEF      2// 示例: 假设 220V 对应 8192 计数
#define SYSTEM_I_COEF     1  // 示例: 假设 1A 对应 8192 计数


/**
 * @brief 从 3 字节数组中提取 24 位数据 (高位在前)
 */
static uint32_t Extract_24bit(const uint8_t *pBuf)
{
    uint32_t val = 0;
    val |= (uint32_t)pBuf[0] << 16;
    val |= (uint32_t)pBuf[1] << 8;
    val |= (uint32_t)pBuf[2] << 0;
    return val;
}

/**
 * @brief 校验 24 字节数据帧
 * @note 此函数在任务上下文或解析函数中调用
 */
static int HLW8032_CheckSum(const uint8_t *pFrame)
{
    uint8_t sum = 0;

    // 校验和是 Reg 3 (帧索引 2) 到 Reg 10 (帧索引 22) 的所有字节相加，取低 8 位。
    for (int i = 2; i <= HLW_CHECKSUM_OFFSET - 1; i++) {
        sum += pFrame[i];
    }

    if (sum == pFrame[HLW_CHECKSUM_OFFSET]) {
        return 0; // 校验通过
    }

    return -1; // 校验失败
}


/**
 * @brief 将原始寄存器计数转换为物理量 (V, A, W)
 * @note 转换公式基于用户提供：(参数寄存器 / 值寄存器) * 系数
 * @note 此函数在任务上下文中运行，允许耗时的浮点运算
 */
static void HLW8032_Calculate(HLW8032_Value_t *pData)
{
    float U_Param = (float)pData->RawU_Param;
    float I_Param = (float)pData->RawI_Param;
    float P_Param = (float)pData->RawP_Param;

    float U_Reg = (float)pData->RawVoltage;
    float I_Reg = (float)pData->RawCurrent;
    float P_Reg = (float)pData->RawPower;

    // 添加防止除以零的保护
    if (U_Reg < 1.0f) U_Reg = 1.0f;
    if (I_Reg < 1.0f) I_Reg = 1.0f;
    if (P_Reg < 1.0f) P_Reg = 1.0f;

    // 1. 电压有效值计算： (电压参数寄存器/电压寄存器 )*电压系数
    pData->Voltage = (U_Param / U_Reg) * SYSTEM_U_COEF;

    // 2. 电流效值计算： (电流参数寄存器/电流寄存器 )*电流系数
    pData->Current = (I_Param / I_Reg) * SYSTEM_I_COEF;

    // 3. 有功功率的计算：（功率参数寄存器/功率寄存器）*电压系数*电流系数
    pData->Power = (P_Param / P_Reg) * SYSTEM_U_COEF * SYSTEM_I_COEF;

    // RMS 值不应为负
    if (pData->Voltage < 0.0f) pData->Voltage = 0.0f;
    if (pData->Current < 0.0f) pData->Current = 0.0f;
}


/**
 * @brief 提取和解析数据帧（在任务上下文中运行）
 */
static int HLW8032_ExtractAndParse(const uint8_t *pFrame, HLW8032_Value_t *pData)
{
    // 1. 校验和检查
    if (HLW8032_CheckSum(pFrame) != 0) {
        return -1; // 校验失败
    }

    // 2. 提取原始寄存器值和参数寄存器值

    // 电压参数 (Reg 3) 和 电压原始值 (Reg 4)
    pData->RawU_Param = Extract_24bit(&pFrame[HLW_REG_RMS_U_PARAM_OFFSET]);
    pData->RawVoltage = Extract_24bit(&pFrame[HLW_REG_RMS_U_OFFSET]);

    // 电流参数 (Reg 5) 和 电流原始值 (Reg 6)
    pData->RawI_Param = Extract_24bit(&pFrame[HLW_REG_RMS_I_PARAM_OFFSET]);
    pData->RawCurrent = Extract_24bit(&pFrame[HLW_REG_RMS_I_OFFSET]);

    // 功率参数 (Reg 7) 和 功率原始值 (Reg 8)
    pData->RawP_Param = Extract_24bit(&pFrame[HLW_REG_ACTIVE_P_PARAM_OFFSET]);
    pData->RawPower = Extract_24bit(&pFrame[HLW_REG_ACTIVE_P_OFFSET]);

    // 3. 原始值转换为物理量 (浮点运算)
    HLW8032_Calculate(pData);

    return 0; // 成功
}


// --- 接口函数实现 ---

void HLW8032_Init(UART_HandleTypeDef *huart)
{
    // 1. 初始化驱动句柄
    hlw8032_h.huart = huart;
    memset(hlw8032_h.RxBuffer, 0, HLW8032_FRAME_SIZE);

    // 2. 创建 FreeRTOS 信号量 (二进制信号量)
    // 确保在任务启动前调用此函数
    if (hlw8032_data_ready_sem == NULL) {
        // 初始计数为 0
        hlw8032_data_ready_sem = osSemaphoreNew(1, 0, NULL);
    }

    // 3. 启动首次接收
 //   HLW8032_StartReceive();
}

void HLW8032_StartReceive(void)
{
    // 启动 DMA 接收一帧数据
    if (hlw8032_h.huart != NULL) {
      //   HAL_UART_Receive_DMA(hlw8032_h.huart, hlw8032_h.RxBuffer, HLW8032_FRAME_SIZE);
    	//  HAL_UART_Receive_IT(hlw8032_h.huart, hlw8032_h.RxBuffer, HLW8032_FRAME_SIZE);
    	if(HAL_UART_Receive(hlw8032_h.huart, hlw8032_h.RxBuffer, HLW8032_FRAME_SIZE,1000)==HAL_OK)
    	{
    		HLW8032_RxCplt(hlw8032_h.huart);
    	}
    }
}

/**
 * @brief 在任务上下文中处理并存储数据
 */
void HLW8032_ProcessAndStore(void)
{
    HLW8032_Value_t temp_data;

    // 1. 在任务上下文中执行耗时的解析、校验和浮点计算
    // 注意：使用从 ISR 复制过来的 RxFrameCopy 缓冲区

    if (HLW8032_ExtractAndParse(RxFrameCopy, &temp_data) == 0) {
        // 2. 如果解析成功，原子性地更新全局数据
        // 仅在任务上下文中写入 hlw8032_h.Data
        hlw8032_h.Data = temp_data;
        memset(RxFrameCopy,0,sizeof(RxFrameCopy));
    } else {
        // 校验失败，不更新数据，保持旧值
    }
}


HLW8032_Value_t HLW8032_GetData(void)
{
    // 注意：读取共享数据时，理想情况下应该使用 osMutex 保护
    // 考虑到 HLW8032_Value_t 包含多个 float/uint32_t，
    // 在 32位 MCU 上，float 和 uint32_t 的读写通常是原子性的，但结构体的复制可能不是。
    // 为了线程安全，这里应该使用互斥锁，但为了简化，我们仅在 app_freertos.c 的 UI_Task 中加锁。
    // 如果没有互斥锁，返回的 Power/Voltage/Current 可能会来自不同帧。
    return hlw8032_h.Data;
}


/**
 * @brief UART 接收完成中断回调处理 (在 ISR 上下文中运行)
 * @note 此函数现在只负责数据复制和信号量释放
 */
void HLW8032_RxCplt(UART_HandleTypeDef *huart)
{

        // 1. 将原始数据快速复制到任务可见的临时缓冲区 (临界区内操作)
        memcpy(RxFrameCopy, hlw8032_h.RxBuffer, HLW8032_FRAME_SIZE);

        // 2. 释放信号量通知 Power_Task
        if (hlw8032_data_ready_sem != NULL) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            // 释放信号量。注意：这里使用了 FreeRTOS 原生 API 的 FromISR 版本
            // 如果使用 CMSIS V2，推荐使用 osSemaphoreRelease，它会内部处理任务切换
            osSemaphoreRelease(hlw8032_data_ready_sem);
            // 实际上，在 CMSIS V2 接口中，osSemaphoreRelease 内部会调用 FromISR 版本并处理 xHigherPriorityTaskWoken
        }

        // 3. 立即重新启动 DMA 接收下一帧 (最小化 ISR 延迟)
    //    HLW8032_StartReceive();

}



// ** 提示：您需要在 stm32f4xx_it.c 中添加以下代码 (假设 UART3): **
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//     HLW8032_RxCplt(huart);
// }

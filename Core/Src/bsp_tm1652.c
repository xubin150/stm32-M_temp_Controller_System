/*
 * bsp_tm1652.c
 *
 *  Created on: Dec 3, 2025
 *      Author: Administrator
 */
#include "bsp_tm1652.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#define TM1652_BAUD_DELAY 52 // 19200bps => ~52us/bit

// ==========================================================
// 公阴极 (Common Cathode) 段码表
// 段码定义: DP G F E D C B A (最高位是DP)
// ==========================================================
const uint8_t TM1652_SegmentMap[] = {
    0x3F, // 0: 0011 1111 (A-F ON)
    0x06, // 1: 0000 0110 (B, C ON)
    0x5B, // 2: 0101 1011
    0x4F, // 3: 0100 1111
    0x66, // 4: 0110 0110
    0x6D, // 5: 0110 1101
    0x7D, // 6: 0111 1101
    0x07, // 7: 0000 0111
    0x7F, // 8: 0111 1111
    0x6F, // 9: 0110 1111
    0x40, // -: 0100 0000 (G ON)
    0x00, // Blank
};

// 定义通道结构体
typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} TM1652_Channel_t;

// 定义4个通道 (根据你的硬件连接修改)
TM1652_Channel_t tm_channels[4] = {
    {PV_SEG_GPIO_Port, PV_SEG_Pin}, // Channel 0  PV
    {SV_SEG_GPIO_Port, SV_SEG_Pin}, // Channel 1  SV
    {POW_SEG_GPIO_Port, POW_SEG_Pin}, // Channel 2  POW
    {VOL_SEG_GPIO_Port, VOL_SEG_Pin}  // Channel 3 VOL
};

// 核心函数：软件模拟 UART 发送一个字节
// 必须加 static 限制作用域，这是底层函数
static void TM1652_SendByte(TM1652_Channel_t* ch, uint8_t data) {

	uint8_t data_copy = data;
	// 1. 进入临界区，禁止任务切换和中断，保证时序严格
    taskENTER_CRITICAL();


    // Start Bit (Low)
    HAL_GPIO_WritePin(ch->port, ch->pin, GPIO_PIN_RESET);
    delay_us(TM1652_BAUD_DELAY);

    // Data Bits (LSB First)
    for (int i = 0; i < 8; i++) {
        if (data & 0x01) {
            HAL_GPIO_WritePin(ch->port, ch->pin, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(ch->port, ch->pin, GPIO_PIN_RESET);
        }
        data >>= 1;
        delay_us(TM1652_BAUD_DELAY);
    }

    // Parity bit
   uint8_t ones = __builtin_popcount(data_copy);
   uint8_t parity = (ones % 2 == 0) ? 1 : 0;  // even → 1, odd → 0

   if(parity)
	   HAL_GPIO_WritePin(ch->port, ch->pin, GPIO_PIN_SET);
   else
	   HAL_GPIO_WritePin(ch->port, ch->pin, GPIO_PIN_RESET);

   delay_us(TM1652_BAUD_DELAY);

    // Stop Bit (High)
    HAL_GPIO_WritePin(ch->port, ch->pin, GPIO_PIN_SET);
    delay_us(TM1652_BAUD_DELAY);

    // 2. 退出临界区
    taskEXIT_CRITICAL();

    // 字节间稍微停顿一下，防止太快 (可选)
    delay_us(20);
}

// 发送命令或数据接口
void TM1652_WriteCommand(uint8_t ch_index, uint8_t cmd) {
    if(ch_index >= 4) return;
    TM1652_SendByte(&tm_channels[ch_index], cmd);
}
void next_cmd(uint8_t ch_index)
{
	if(ch_index >= 4) return;
    HAL_GPIO_WritePin(tm_channels[ch_index].port, tm_channels[ch_index].pin, GPIO_PIN_SET);
    osDelay(5); // 发送下调指令至少SDA拉高后延时3ms
}


/**
 * @brief 设置 TM1652 通道的显示控制和亮度 (对应 CommandX 和 CommandY)
 * @param ch_index: TM1652通道索引 (0-3)
 * @param current_level: 亮度/电流调节命令 (如 TM1652_SET_CONTROL_MID)
 * @note 这个命令是使能显示的关键，通常在显示数据后发送。
 */
void TM1652_SetControl(uint8_t ch_index, uint8_t current_level)
{
    if (ch_index >= 4) return;

    // 1. CommandX: 发送显示控制命令 (0x18)
    TM1652_WriteCommand(ch_index, TM1652_CMD_Control);

    // 2. CommandY: 发送显示控制调节命令 (亮度/模式) - 包含显示使能位
    TM1652_WriteCommand(ch_index, current_level);

    // 3. 间隔：用于分隔控制命令和下一条命令/数据
    next_cmd(ch_index);
}


/**
 * @brief 向TM1652通道发送4个数码管的显示数据
 * @param ch_index: TM1652通道索引 (0-3)
 * @param seg_data: 包含4个字节段码数据的数组 (DDR0, DDR1, DDR2, DDR3)
 * @note 遵循 Address -> Data x 4 -> 3ms Time 的流程。
 */
void TM1652_SetSegments(uint8_t ch_index, const uint8_t seg_data[4])
{
    if (ch_index >= 4) return;

    // 1. Command1: 选择显示地址命令 (0x08 | 0x00) -> 启动地址自动递增
    TM1652_WriteCommand(ch_index, TM1652_CMD_ADDR_BASE );

    // 2. Data1~Data4: 顺序发送4个显示数据字节 (TM1652内部地址递增)
    for (int i = 0; i < 4; i++) {
        TM1652_WriteCommand(ch_index, seg_data[i]);
    }

    // 3. Time: 数据线置高时间（最小时间为3ms），用于分隔数据和控制命令
    next_cmd(ch_index);
}

/*
 * bsp_tm1652.h
 *
 *  Created on: Dec 3, 2025
 *      Author: Administrator
 */

#ifndef SRC_BSP_TM1652_H_
#define SRC_BSP_TM1652_H_
#include "main.h"
// 1. 地址写入命令基址 (Address Write Command Base)
// 0x08 是地址写入命令
#define TM1652_CMD_ADDR_BASE        0x08   //后面可跟最多6Byte（实际板子是4位数码管）
#define TM1652_CMD_Control   0x18 //显示控制命令
//显示调节控制命令
#define TM1652_set_Control_off   0x00  // 输入电流为0
//#define TM1652_set_Control_mid   0x1C //段驱动电流1/2
#define TM1652_set_Control_max   0xFE //段驱动电流1/2

void TM1652_WriteCommand(uint8_t ch_index, uint8_t cmd);
void next_cmd(uint8_t ch_index);
void TM1652_SetControl(uint8_t ch_index, uint8_t current_level);
void TM1652_SetSegments(uint8_t ch_index, const uint8_t seg_data[4]);

#endif /* SRC_BSP_TM1652_H_ */

/*
 * @Author: Tian Wei
 * @Date: 2021-11-05 20:02:11
 * @Description:
 * @Version: V1.0
 */
#ifndef __HT4315_H
#define __HT4315_H

#include "gpio.h"
#include "usart.h"
#include "UART.h"

#define RS458RE (RS485_DE_RE_GPIO_Port->BSRR = (uint32_t)RS485_DE_RE_Pin << 16U)
#define RS458DE (RS485_DE_RE_GPIO_Port->BSRR = RS485_DE_RE_Pin)

#define MasterHeader 0x3e //主机发送协议头  see 协议说明Page1
#define SlaverHeader 0x3c

#define Motor1 0x01 // HT4315电机1的地址
#define Motor2 0x02 // HT4315电机2的地址

#define MotorPosCtrl 0x55 // 电机绝对值位置闭环控制命令

uint16_t CRC16_MODBUS(uint8_t *Buff, uint8_t len);
HAL_StatusTypeDef RS485Trans(uint8_t sequence, uint8_t slaver_addr, uint8_t cmd, uint8_t data_len, uint8_t *data);
void set_Motor_angle(uint8_t slaver_addr, int16_t angle);
void vParseString(uint8_t *buff);
void restartRev1(void);
#endif

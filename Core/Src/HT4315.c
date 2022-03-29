/*
 * @Author: Tian Wei
 * @Date: 2021-11-05 20:01:54
 * @Description:
 * @Version: V1.0
 */
#include "HT4315.h"
#include "StepMotor.h"
uint8_t RS485TxBuff[20] = {0};
uint8_t RS485RxBuff[20] = {0};
int16_t Motor1Angle = 0;
int16_t Motor2Angle = 0;
const uint16_t crctalbeabs[] = {
    0x0000, 0xCC01, 0xD801, 0x1400, 0xF001, 0x3C00, 0x2800, 0xE401,
    0xA001, 0x6C00, 0x7800, 0xB401, 0x5000, 0x9C01, 0x8801, 0x4400};
uint16_t CRC16_MODBUS(uint8_t *Buff, uint8_t len)
{
    uint16_t crc = 0xffff;
    uint16_t i;
    uint8_t ch;
    for (i = 0; i < len; i++)
    {
        ch = *Buff++;
        crc = crctalbeabs[(ch ^ crc) & 15] ^ (crc >> 4);
        crc = crctalbeabs[((ch >> 4) ^ crc) & 15] ^ (crc >> 4);
    }
    return crc;
}
HAL_StatusTypeDef RS485Trans(uint8_t sequence, uint8_t slaver_addr, uint8_t cmd, uint8_t data_len, uint8_t *data)
{
    RS485TxBuff[0] = MasterHeader;
    RS485TxBuff[1] = sequence;
    RS485TxBuff[2] = slaver_addr;
    RS485TxBuff[3] = cmd;
    RS485TxBuff[4] = data_len;
    for (uint8_t i = 0; i < data_len; i++)
    {
        RS485TxBuff[5 + i] = data[i];
    }
    uint16_t crc16 = CRC16_MODBUS(RS485TxBuff, 5 + data_len);
    RS485TxBuff[5 + data_len] = (uint8_t)crc16;
    RS485TxBuff[6 + data_len] = (uint8_t)(crc16 >> 8);
    RS458DE;
    return HAL_UART_Transmit(&huart1, RS485TxBuff, 7 + data_len, 50);
}
void set_Motor_angle(uint8_t slaver_addr, int16_t angle)
{
    uint8_t data[4] = {0};
    int32_t count = angle;
		//int32_t count = angle * 16384 / 360;
    data[0] = (uint8_t)count;
    data[1] = (uint8_t)(count >> 8);
    data[2] = (uint8_t)(count >> 16);
    data[3] = (uint8_t)(count >> 24);
    RS485Trans(0, slaver_addr, MotorPosCtrl, 4, data);
    //RS458RE;
}
void Read_Motor_angle(int16_t *var)
{
	  uint8_t angleRev[15]={0};
	  //double  var[4] = {0};
		//int16_t  var[4] = {0};
	  uint16_t tmp=0;
    RS485Trans(0, 1, MotorPosRead, 0, NULL);
    RS458RE;
		HAL_UART_Receive(&huart1,angleRev,15,200);
    tmp |= angleRev[6];
		tmp<<=8;
		tmp|=angleRev[5];
		//var[0]=(double)tmp*360/16384;
		var[0]=tmp;
		var[1]=Motor1Angle;
		HAL_Delay(1);
	  RS485Trans(0, 2, MotorPosRead, 0, NULL);
    RS458RE;
		HAL_UART_Receive(&huart1,angleRev,15,200);
	  tmp=0;
    tmp|= angleRev[6];
		tmp<<=8;
		tmp|=angleRev[5];
	  //var[2]=(double)tmp*360/16384;
		var[2]=tmp;
		var[3]=Motor2Angle;
    //vcan_sendware((uint8_t *)var, sizeof(var));
}
void vParseString(uint8_t *buff)
{
    char *pBuffMotor1;
    char *pBuffMotor2;
    char *pBuffForce;
    int16_t setForce = 0;
    //获取电机字符串指针
    pBuffMotor1 = strstr((const char *)buff, ":");
    //指针+1，取出正确的头指针
    pBuffMotor1++;
    pBuffMotor2 = strstr((const char *)pBuffMotor1, ":");
    if (pBuffMotor1 != NULL)
    {
        Motor1Angle = atoi(strtok(pBuffMotor1, ","));
    }
    //指针+1，取出正确的头指针
    pBuffMotor2++;
    pBuffForce = strstr((const char *)pBuffMotor2, ":");
    if (pBuffMotor2 != NULL)
    {

        Motor2Angle = atoi(strtok(pBuffMotor2, ","));
    }
    pBuffForce++;
    if (pBuffForce != NULL)
    {
        setForce = atoi(strtok(pBuffForce, ";"));
    }
    SetPIDForce(setForce);
    if (Motor1Angle < 8192 && Motor1Angle > 0)
    {
        set_Motor_angle(Motor1, Motor1Angle);
    }
    if (Motor2Angle < 8192 && Motor2Angle > 0)
    {
        HAL_Delay(5);
        set_Motor_angle(Motor2, Motor2Angle);
    }
    // printf("%d %d",Motor1Angle,Motor2Angle);
}
void restartRev1(void)
{
    memset(rx_buffer, 0, rx_len);
    rx_len = 0;                                            //清除计数
    recv_end_flag = 0;                                     //清除接收结束标志
    HAL_UART_Receive_DMA(&huart2, rx_buffer, BUFFER_SIZE); //重新打开DMA接收
}

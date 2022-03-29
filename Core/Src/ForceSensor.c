/*
 * @Author: Tian Wei
 * @Date: 2021-11-16 13:00:42
 * @Description:
 * @Version: V1.0
 */
#include "ForceSensor.h"
#include "StepMotor.h"
#include "HT4315.h"
__IO extern uint8_t Force_Angle;
__IO int16_t realForce=0;
void getSensor(void)
{
    uint8_t sensorCmd[8] = {0x01U, 0x03U, 0U, 0U, 0U, 0x1U, 0x84U, 0x0aU};
    HAL_UART_Transmit(&huart3, (uint8_t *)sensorCmd, 8, 50);
}

int16_t vParseSensor(void)
{
    int16_t tmp=0;
    tmp |= rx_buffer2[3];
    tmp <<= 8;
    tmp |= rx_buffer2[4]; //单位0.1N，实际的力为force*0.1N=force*10g/1kg*10N/kg
	  realForce=tmp;
	return realForce;
    
}
int16_t getRealForce(void)
{
	return realForce;
}
void restartRev2(void)
{
    memset(rx_buffer2, 0, rx_len2);
    rx_len2 = 0;                                  //清除计数
    recv_end_flag2 = 0;                           //清除接收结束标志
    HAL_UART_Receive_DMA(&huart3, rx_buffer2, 8); //重新打开DMA接收
}

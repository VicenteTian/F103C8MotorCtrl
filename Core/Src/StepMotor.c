/*
 * @Author: Tian Wei
 * @Date: 2021-11-15 13:48:27
 * @Description:
 * @Version: V1.0
 */
#include "StepMotor.h"
#include "tim.h"
#include "key.h"

__IO uint16_t Toggle_Pulse = 500;
__IO uint8_t IsPIDEnable = 0;
typedef struct
{
  __IO float MotorSpeed;    // 控制电机的脉冲频率
  __IO int16_t SetPoint;    // 目标值  单位:0.1N
  __IO int16_t ActualPoint; // 实际值  单位:0.1N
  __IO int16_t Err;         // 偏差值  单位:0.1N  E[k]项
  __IO int16_t LastError;   // 前一次误差     E[k-1]项
  __IO int16_t PrevError;   // 前两次误差     E[k-2]项
  __IO float Kp, Ki, Kd;
} PID;
__IO static PID sPID;
void PID_init(void)
{
  sPID.MotorSpeed = 0.0;
  sPID.SetPoint = 400;
  sPID.ActualPoint = 0;
  sPID.Err = 0;
  sPID.LastError = 0;
  sPID.PrevError = 0;
  sPID.Kp = 4.5;
  sPID.Ki = 0.0;
  sPID.Kd = 5.5;
}
int16_t GetSetForce(void)
{
  return sPID.SetPoint;
}
void SetPIDForce(int16_t setForce)
{
  sPID.SetPoint = setForce;
}
void StempMotorPIDCtrol(int16_t RealForce)
{
  uint8_t dir = 0;
  float Exp_Val = 0;
  /*********增量式PID************************/
  float iIncpid = 0;
  sPID.ActualPoint = RealForce;
  sPID.Err = sPID.SetPoint - sPID.ActualPoint;    //当前误差
  iIncpid = sPID.Kp * (sPID.Err - sPID.LastError) //
            + sPID.Ki * sPID.Err                  //
            + sPID.Kd * (sPID.Err - 2 * sPID.LastError + sPID.PrevError);
  sPID.MotorSpeed += iIncpid;
  sPID.PrevError = sPID.LastError; // 存储误差，用于下次计算
  sPID.LastError = sPID.Err;
  /************************************/
  if (sPID.MotorSpeed >= 0)
  {
    dir = 1;
  }
  else
  {
    dir = 0;
  }
  Exp_Val = abs(sPID.MotorSpeed);
  if (IsPIDEnable)
  {
    STEPMOTOR_Motion_Ctrl(dir, Exp_Val);
  }
}
void StempMotorStateCtrol(void)
{
  static uint8_t dir = 1; // 0 ：下降  1：上升
  static uint8_t ena = 1; // 0 ：正常运行 1：停机
  uint8_t ucKeyCode;
  ucKeyCode = bsp_GetKey();
  if (ucKeyCode != KEY_NONE)
  {
    switch (ucKeyCode)
    {
    case KEY_2_DOWN://Center
			HAL_GPIO_TogglePin(GPIOB,LED_Green_Pin);
      if (ena == 1)
      {
        MotorEnable(); // 正常运行
        //TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_1, TIM_CCx_ENABLE);
        ena = 0;
      }
      else
      {
        MotorDisable(); // 停机
        //TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_1, TIM_CCx_DISABLE);
        ena = 1;
      }
      break;
    case KEY_1_DOWN://Left
			HAL_GPIO_TogglePin(GPIOB,LED_Green_Pin);
      if (dir == 0)
      {
        dir = 1;
      }
      else
      {
        dir = 0;
      }
      STEPMOTOR_Motion_Ctrl(dir, 200);
      break;
    case KEY_4_DOWN://up		
			HAL_GPIO_TogglePin(GPIOB,LED_Green_Pin);
      if (IsPIDEnable == 0)
      {
        IsPIDEnable = 1;
      }
      else
      {
        IsPIDEnable = 0;
      }
      break;
    }
  }
}
/**
 * 函数功能: 步进电机运动控制
 * 输入参数: Dir:步进电机运动方向 0:反转 1正转
 *           Frequency:步进电机频率,0:停止
 * 返 回 值: void
 * 说    明: 无
 */
void STEPMOTOR_Motion_Ctrl(uint8_t Dir, float Frequency)
{

  if (Dir == 1)
  {
		HAL_GPIO_WritePin(GPIOB, LED_Red_Pin, GPIO_PIN_RESET);
    MotorUp();
  }
  else
  {
		HAL_GPIO_WritePin(GPIOB, LED_Red_Pin, GPIO_PIN_SET);
    MotorDown();
  }
  if (Frequency > MAX_SPEED)
  {
    Frequency = MAX_SPEED;
  }
  if (Frequency < MIN_SPEED)
  {
    Frequency = MIN_SPEED;
  }
  Toggle_Pulse = (uint16_t)(FREQ_UINT / Frequency);
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  __IO uint32_t count;
  __IO uint32_t tmp;
  count = __HAL_TIM_GET_COUNTER(&htim1);
  tmp = STEPMOTOR_TIM_PERIOD & (count + Toggle_Pulse);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, tmp);
}

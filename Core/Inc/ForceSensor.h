/*
 * @Author: Tian Wei
 * @Date: 2021-11-16 13:00:54
 * @Description: 
 * @Version: V1.0
 */
#ifndef __FORCESENSOR_H
#define __FORCESENSOR_H

#include "usart.h"
#include "UART.h"
void getSensor(void);
int16_t getRealForce(void);
int16_t vParseSensor(void);
void restartRev2(void);
#endif


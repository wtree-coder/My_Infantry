#ifndef __MY_DVC_DR16_H
#define __MY_DVC_DR16_H

#include "stm32f4xx.h"
#define UART_MAX_SIZE 36

typedef enum 
{
    DR16_SWITCH_UP = 1,
    DR16_SWITCH_DOWN = 2,
    DR16_SWITCH_MID = 3,
} Enum_DR16_Switch_Status;

typedef struct
{
    uint8_t Tx_Buff[UART_MAX_SIZE];
    uint8_t Rx_Buff[UART_MAX_SIZE];
} DR16_t;

typedef struct
{
    float Right_X;
    float Right_Y;
    float Left_X;
    float Left_Y;
    float Yaw;

    Enum_DR16_Switch_Status Chassis_Switch;   // 遥控器右侧拨杆 (控制底盘模式)
    Enum_DR16_Switch_Status Gimbal_Switch;    // 遥控器左侧拨杆 (控制云台模式)

    uint8_t is_ok;
} DR16_Data_t;

#define DR16_DEADZONE 0.02f

void DR16_Init(UART_HandleTypeDef *huart);
void DR16_Data_Get(uint8_t *Rx_Buff, DR16_Data_t *Data);
void DR16_Check(void);
float DR16_XiaoZhun(float input);

extern DR16_t DR16;
extern DR16_Data_t DR16_Data;

#endif

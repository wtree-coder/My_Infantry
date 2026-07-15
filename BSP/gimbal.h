#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "stm32f4xx_hal.h"
#include "motor.h"
#include "drv_can.h"
#include "pid.h"

typedef enum
{
    GIMBAL_SHOOT = 0,
    GIMBAL_MOVE,
    GIMBAL_DISABLE,
    GIMBAL_HOMING,
} Gimbal_Mode;

typedef struct
{
    Motor_t motor_6020;
    PIDControllerTypedef pid_6020_angle;
    PIDControllerTypedef pid_6020_omega;
    float vx;
    float vy;
    float wz;
    float target_yaw;       // IMU yaw目标值 (rad), 遥控器累积

    uint8_t mode;
    uint8_t is_ok;
}Gimbal_t;


void Gimbal_Init(void);
void Gimbal_Check(void);
void Gimbal_Rx_Callback(CAN_Rx_Buffer_t *rx);
void Gimbal_Update(void);
void Gimbal_Pid_Calc(void);
void Gimbal_Update_From_DR16(void);

void Gimbal_Homing(void);
void Gimbal_1ms_Callback(void);
void Gimbal_Send_Callback(void);

extern Gimbal_t gimbal;

#endif

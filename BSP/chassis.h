#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "stm32f4xx_hal.h"
#include "motor.h"
#include "drv_can.h"
#include "pid.h"

#define WHEEL_RADIUS     0.076f    // 麦轮半径 (m)
#define CHASSIS_WIDTH    0.5f      // 宽度 (m)
#define CHASSIS_LENGTH   0.55f     // 长度 (m)

#define CHASSIS_R        ((CHASSIS_WIDTH + CHASSIS_LENGTH) * 0.5f)  // 旋转半径 (m)

#define CHASSIS_MAX_V            1.5f     // 最大平动速度 (m/s)
#define CHASSIS_MAX_WZ           3.0f     // 最大旋转速度 (rad/s)

//小陀螺
#define CHASSIS_WZ_XIAOTUOLUO    1.5f     // 小陀螺模式 wz (rad/s)
#define XIAOTUOLUO_STEP          0.03f    // 软启动步长 (rad/s per ms)

#define CHASSIS_FOLLOW_KP        3.0f     // 底盘跟随增益
#define CHASSIS_FILTER           0.10f    // 目标速度滤波系数

#define CHASSIS_KF               200

typedef enum
{    
    RU = 0,   //0x201
    LU,   //0x202
    LD,   //0x203
    RD,   //0x204
}Motor_Position;

typedef enum
{
    Chassis_Move = 0,
    Chassis_XiaoTuoLuo = 1,
    Chassis_Disable = 2,
}Chassis_Mode;

typedef struct
{
    uint8_t mode;

    Motor_t motor_chassis[4];
    PIDControllerTypedef pid_chassis[4];
    PIDControllerTypedef pid_follow[4];
    float KF[4];
    
    float vx;
    float vy;
    float wz;

    float filter_vx;
    float filter_vy;
    float filter_wz;

    float gimbal_vx;
    float gimbal_vy;

    int16_t out[4];
    uint8_t is_ok;
}Chassis_t;

void Chassis_Init(void);
void Chassis_Check(void);
void Mode_Choose(void);
void Chassis_1ms_Callback(void);
void Chassis_Rx_Callback(CAN_Rx_Buffer_t *rx);


extern Chassis_t chassis;

#endif

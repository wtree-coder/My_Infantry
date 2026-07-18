#include "gimbal.h"
#include "drv_can.h"
#include "motor.h"
#include "pid.h"
#include "chassis.h"
#include "DR16.h"
#include "ahrs.h"
#include <math.h>

Gimbal_t gimbal;

void Gimbal_Init(void)
{
    Motor_Init(&gimbal.motor_6020, MOTOR_6020_RATE, MOTOR_6020_MAX_RPM, MOTOR_6020_OUT_LIMIT);
    PID_Init(&gimbal.pid_6020_angle, 80.0f, 0.0f, 0.0f, 0.001f, MOTOR_6020_MAX_RPM * RPM_TO_RADPS, 50.0f, 0.0f, PID_MODE_POSITION);
    PID_Init(&gimbal.pid_6020_omega, 500.0f, 0.0f, 0.0f, 0.001f, MOTOR_6020_OUT_LIMIT, 5000.0f, 0.0f, PID_MODE_POSITION);
    gimbal.target_yaw = 0.0f;
    gimbal.mode = GIMBAL_HOMING;
}

void Gimbal_Check(void)
{
    Motor_Check(&gimbal.motor_6020);
    if(gimbal.motor_6020.is_ok)
    {
        gimbal.is_ok = 1;
    }
    else gimbal.is_ok = 0;
}

void Gimbal_Rx_Callback(CAN_Rx_Buffer_t *rx)
{
    Motor_Rx_Callback(&gimbal.motor_6020, rx->Data);
}

void Gimbal_Homing(void)
{
    static uint32_t stable_cnt = 0;

    if(!gimbal.is_ok)
    {
        gimbal.motor_6020.target_rad_s = 0.0f; //可有可无？
        stable_cnt = 0;
        return;
    }

    float home_angle = -1000.0f / 8192.0f * 2.0f * PI;
    float error = home_angle - gimbal.motor_6020.now_angle;

    if(fabsf(error) < 0.03f && fabsf(gimbal.motor_6020.now_rad_s) < 0.5f)
    {
        if(++stable_cnt >= 300)
        {
            stable_cnt = 300;
            gimbal.motor_6020.angle_offset = gimbal.motor_6020.now_angle;
            gimbal.target_yaw = -AHRS_Get_Yaw() * (PI / 180.0f);
            gimbal.mode = GIMBAL_DISABLE;
            return;
        }
    }
    else
    {
        stable_cnt = 0;
    }

    float out = PIDCompute(&gimbal.pid_6020_angle, gimbal.motor_6020.now_angle, home_angle);
    float limit = MOTOR_6020_MAX_RPM * RPM_TO_RADPS * 0.5f;
    if(out >  limit) out =  limit;
    if(out < -limit) out = -limit;
    gimbal.motor_6020.target_rad_s = out;
}


void Gimbal_Update(void)
{
    gimbal.target_yaw += DR16_XiaoZhun(DR16_Data.Right_X) * 0.005f;

    float yaw_rad = AHRS_Get_Yaw() * (PI / 180.0f);
    float target_rad_s = PIDCompute(&gimbal.pid_6020_angle, -yaw_rad, gimbal.target_yaw);
    float limit = MOTOR_6020_MAX_RPM * RPM_TO_RADPS;

    if(target_rad_s >  limit) target_rad_s =  limit;
    if(target_rad_s < -limit) target_rad_s = -limit;
    
    gimbal.motor_6020.target_rad_s = target_rad_s;
}

void Gimbal_Shoot_Update(void)
{
    Gimbal_Update();  // 目前和Move一样, 后续加弹道补偿
}

void Gimbal_Update_From_DR16(void)
{
    gimbal.motor_6020.target_rad_s = DR16_XiaoZhun(DR16_Data.Right_X) * (MOTOR_6020_MAX_RPM * RPM_TO_RADPS);
}

void Gimbal_Pid_Calc(void)
{
    gimbal.motor_6020.out = PIDCompute(&gimbal.pid_6020_omega, gimbal.motor_6020.now_rad_s, gimbal.motor_6020.target_rad_s);
    if(gimbal.motor_6020.out >  gimbal.motor_6020.output_limit) gimbal.motor_6020.out =  gimbal.motor_6020.output_limit;
    if(gimbal.motor_6020.out < -gimbal.motor_6020.output_limit) gimbal.motor_6020.out = -gimbal.motor_6020.output_limit;
}

void Gimbal_Send_Callback(void)
{
    if(!gimbal.is_ok) return;
    uint8_t tx_buf[8];
    tx_buf[6] = gimbal.motor_6020.out >> 8;
    tx_buf[7] = gimbal.motor_6020.out;
    CAN_Send_Data(&hcan1, 0x1FF, tx_buf, 8);
}

void Gimbal_1ms_Callback(void)
{
    switch(gimbal.mode)
    {
        case GIMBAL_HOMING:
            Gimbal_Homing();
            break;
        case GIMBAL_DISABLE:
            gimbal.motor_6020.target_rad_s = 0.0f;
            gimbal.target_yaw = -AHRS_Get_Yaw() * (PI / 180.0f);
            break;
        case GIMBAL_SHOOT:
            Gimbal_Shoot_Update();                // TODO: 射击特有逻辑(弹道补偿等)
            break;
        case GIMBAL_MOVE:
            Gimbal_Update();
            break;
        default:
            break;
    }
    Gimbal_Pid_Calc();
    Gimbal_Send_Callback();
}

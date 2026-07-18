#include "chassis.h"
#include "pid.h"
#include "motor.h"
#include "gimbal.h"
#include "math.h"
#include "ahrs.h"
#include "DR16.h"

Chassis_t chassis;

void Chassis_Init(void)
{
    chassis.mode = Chassis_Disable;
    chassis.vx   = 0.0f;
    chassis.vy   = 0.0f;
    chassis.wz   = 0.0f;
    chassis.filter_vx = 0.0f;
    chassis.filter_vy = 0.0f;
    chassis.filter_wz = 0.0f;
    chassis.gimbal_vx  = 0.0f;
    chassis.gimbal_vy  = 0.0f;
    chassis.is_ok = 0;

    for(int i = 0;i < 4;i++)
    {
        Motor_Init(&chassis.motor_chassis[i], MOTOR_3508_RATE, MOTOR_3508_MAX_RPM, MOTOR_3508_OUT_LIMIT);
        PID_Init(&chassis.pid_chassis[i], 500.0f, 0.0f, 0.0f, 0.001f, MOTOR_3508_OUT_LIMIT, 5000.0f, 0.0f, PID_MODE_POSITION);
    }
}

void Chassis_Check(void)
{
    for(int i = 0;i < 4;i++)
    {
        Motor_Check(&chassis.motor_chassis[i]);
        if(chassis.motor_chassis[i].is_ok) chassis.is_ok = 1;
        else
        {
            chassis.is_ok = 0;
            break;
        }
    }
}

void Chassis_Update(void)
{
    chassis.filter_vx = chassis.filter_vx * (1.0f - CHASSIS_FILTER) + chassis.vx * CHASSIS_FILTER;
    chassis.filter_vy = chassis.filter_vy * (1.0f - CHASSIS_FILTER) + chassis.vy * CHASSIS_FILTER;
    chassis.filter_wz = chassis.filter_wz * (1.0f - CHASSIS_FILTER) + chassis.wz * CHASSIS_FILTER;

    chassis.motor_chassis[LU].target_rad_s = ( chassis.filter_vx - chassis.filter_vy + chassis.filter_wz * CHASSIS_R ) / WHEEL_RADIUS;
    chassis.motor_chassis[LD].target_rad_s = ( chassis.filter_vx + chassis.filter_vy + chassis.filter_wz * CHASSIS_R ) / WHEEL_RADIUS;
    chassis.motor_chassis[RU].target_rad_s = (-chassis.filter_vx - chassis.filter_vy + chassis.filter_wz * CHASSIS_R ) / WHEEL_RADIUS;
    chassis.motor_chassis[RD].target_rad_s = (-chassis.filter_vx + chassis.filter_vy + chassis.filter_wz * CHASSIS_R ) / WHEEL_RADIUS;
}

void Chassis_Pid_Calc(float ff[4])
{
    for(int i = 0; i < 4;i++)
    {
        chassis.out[i] = PIDCompute(&chassis.pid_chassis[i], chassis.motor_chassis[i].now_rad_s, chassis.motor_chassis[i].target_rad_s) + ff[i];
        if(chassis.out[i] >  chassis.motor_chassis[i].output_limit) chassis.out[i] =  chassis.motor_chassis[i].output_limit;
        if(chassis.out[i] < -chassis.motor_chassis[i].output_limit) chassis.out[i] = -chassis.motor_chassis[i].output_limit;
    }
}

void Chassis_Send_Callback(void)
{
    if(!chassis.is_ok) return;
    uint8_t tx_buf[8];
    for(int i = 0;i < 4;i++)
    {
        tx_buf[i * 2]     = chassis.out[i] >> 8;
        tx_buf[i * 2 + 1] = chassis.out[i];
    }
    CAN_Send_Data(&hcan1, 0x200, tx_buf, 8);
}

void Gimbal_To_Chassis(float gimbal_vx, float gimbal_vy, float chassis_yaw, float *chassis_vx, float *chassis_vy)
{
    float c = cosf(chassis_yaw);
    float s = sinf(chassis_yaw);
    *chassis_vx =  gimbal_vx * c + gimbal_vy * s;
    *chassis_vy = -gimbal_vx * s + gimbal_vy * c;
}

float Get_Yaw(void)
{
    float yaw = AHRS_Get_Yaw() * (PI / 180.0f);
    return yaw - (gimbal.motor_6020.now_angle - gimbal.motor_6020.angle_offset);
}

void Chassis_Rx_Callback(CAN_Rx_Buffer_t *rx)
{
    if(rx == NULL) return;
    
    switch (rx->Header.StdId)
    {
        case 0x201: Motor_Rx_Callback(&chassis.motor_chassis[LU], rx->Data); break;
        case 0x202: Motor_Rx_Callback(&chassis.motor_chassis[LD], rx->Data); break;
        case 0x203: Motor_Rx_Callback(&chassis.motor_chassis[RU], rx->Data); break;
        case 0x204: Motor_Rx_Callback(&chassis.motor_chassis[RD], rx->Data); break;
        default: break;
    }
}

void Chassis_Calc_Forward(float chassis_vx, float chassis_vy, float chassis_wz, float KF[4])
{
    float dvx =  chassis_vy * chassis_wz;
    float dvy = -chassis_vx * chassis_wz;

    KF[LU] = CHASSIS_KF * ( dvx - dvy) / WHEEL_RADIUS;
    KF[LD] = CHASSIS_KF * ( dvx + dvy) / WHEEL_RADIUS;
    KF[RU] = CHASSIS_KF * (-dvx - dvy) / WHEEL_RADIUS;
    KF[RD] = CHASSIS_KF * (-dvx + dvy) / WHEEL_RADIUS;
}

void Chassis_Move_Calc(void)
{
    chassis.vx = -DR16_XiaoZhun(DR16_Data.Left_Y)  * CHASSIS_MAX_V;
    chassis.vy =  DR16_XiaoZhun(DR16_Data.Left_X)  * CHASSIS_MAX_V;

    float gimbal_angle = gimbal.motor_6020.now_angle - gimbal.motor_6020.angle_offset;
    gimbal_angle = atan2f(sinf(gimbal_angle), cosf(gimbal_angle));   // 最短路径
    chassis.wz = gimbal_angle * CHASSIS_FOLLOW_KP;
}

void Chassis_XiaoTuoLuo_Calc(void)
{
    chassis.gimbal_vx = -DR16_XiaoZhun(DR16_Data.Left_Y) * CHASSIS_MAX_V;
    chassis.gimbal_vy =  DR16_XiaoZhun(DR16_Data.Left_X) * CHASSIS_MAX_V;

    float gimbal_angle = gimbal.motor_6020.now_angle - gimbal.motor_6020.angle_offset;
    Gimbal_To_Chassis(chassis.gimbal_vx, chassis.gimbal_vy, gimbal_angle, &chassis.vx, &chassis.vy);
}

void Mode_Choose(void)
{
    if(DR16_Data.Chassis_Switch == DR16_SWITCH_UP)          chassis.mode = Chassis_XiaoTuoLuo;
    else if(DR16_Data.Chassis_Switch == DR16_SWITCH_MID)    chassis.mode = Chassis_Move;
    else                                                    chassis.mode = Chassis_Disable;

    if(gimbal.mode != GIMBAL_HOMING)
    {
        if(DR16_Data.Gimbal_Switch == DR16_SWITCH_UP)        gimbal.mode = GIMBAL_SHOOT;
        else if(DR16_Data.Gimbal_Switch == DR16_SWITCH_MID)  gimbal.mode = GIMBAL_MOVE;
        else                                                 gimbal.mode = GIMBAL_DISABLE;
    }
}

void Chassis_1ms_Callback(void)
{
    if(gimbal.mode == GIMBAL_HOMING) return;
    static float ramp_wz = 0.0f;

    if(chassis.mode == Chassis_Move)
    {
        ramp_wz = 0.0f;
        Chassis_Move_Calc();
    }
    else if(chassis.mode == Chassis_XiaoTuoLuo)
    {
        Chassis_XiaoTuoLuo_Calc();
        if(ramp_wz < CHASSIS_WZ_XIAOTUOLUO)
        {
            ramp_wz += XIAOTUOLUO_STEP;
            if(ramp_wz > CHASSIS_WZ_XIAOTUOLUO) ramp_wz = CHASSIS_WZ_XIAOTUOLUO;
        }
        chassis.wz = ramp_wz;
    }
    else
    {
        ramp_wz = 0.0f;
        chassis.vx = 0.0f;
        chassis.vy = 0.0f;
        chassis.wz = 0.0f;
    }
    Chassis_Update();
    //Chassis_Calc_Forward(chassis.filter_vx, chassis.filter_vy, chassis.filter_wz, chassis.KF);
    Chassis_Pid_Calc(chassis.KF);
    Chassis_Send_Callback();
}

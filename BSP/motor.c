#include "motor.h"
#include <string.h>

void Motor_Init(Motor_t *motor, float rate, float max_rpm, float output_limit)
{
    if (motor == NULL) return;

    memset(motor, 0, sizeof(Motor_t));

    motor->rate         = rate;
    motor->max_rpm      = max_rpm;
    motor->output_limit = output_limit;
}

void Motor_Check(Motor_t *motor)
{
    if(motor->check_count < 50)
    {
        motor->check_count++;
    }
    else
    {
        float rate         = motor->rate;
        float max_rpm      = motor->max_rpm;
        float output_limit = motor->output_limit;

        memset(motor, 0, sizeof(Motor_t));

        motor->rate         = rate;
        motor->max_rpm      = max_rpm;
        motor->output_limit = output_limit;
    }
}

void Motor_Rx_Callback(Motor_t *motor, uint8_t *Rx_Data)
{
    static int16_t pre_encoder;
    if (motor == NULL || Rx_Data == NULL) return;

    motor->check_count = 0;
    motor->is_ok = 1;
    pre_encoder = motor->rx_encoder;

    motor->rx_encoder   = (uint16_t)(Rx_Data[0] << 8  | Rx_Data[1]);            // [0:1] 编码器 0~8191
    motor->rx_rpm       = (int16_t)( Rx_Data[2] << 8  | Rx_Data[3]);            // [2:3] 转速 RPM (转子端)
    motor->rx_torque    = (int16_t)( Rx_Data[4] << 8  | Rx_Data[5]);            // [4:5] 转矩电流
    motor->temp         = Rx_Data[6];                                           // [6]   温度

    int16_t delta_encoder = motor->rx_encoder - pre_encoder;

    if (delta_encoder < -4096)
    {
        motor->total_round++;          // 正转跨越零点
    }
    else if (delta_encoder > 4096)
    {
        motor->total_round--;          // 反转跨越零点
    }

    motor->total_encoder = motor->total_round * 8192 + motor->rx_encoder;

    motor->now_angle = (float)motor->total_encoder / 8192.0f * 2.0f * PI / motor->rate;
    motor->now_rad_s = (float)motor->rx_rpm * RPM_TO_RADPS / motor->rate;

    motor->now_torque = (float)motor->rx_torque;
}


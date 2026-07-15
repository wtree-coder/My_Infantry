#ifndef __AHRS_H
#define __AHRS_H

#include "stm32f4xx_hal.h"

typedef struct
{
    float Kp;            // 加速度计校正比例增益 (典型 0.5~2.0)
    float Ki;            // 陀螺仪零偏估计积分增益 (典型 0.0~0.1)
    float sample_freq;   // 采样频率 Hz
    float halfT;         // 半采样周期 (内部自动计算)

    float roll;
    float pitch;
    float yaw;
    uint8_t is_ok;
} AHRS_t;

extern AHRS_t ahrs;

/**
 * @brief  初始化姿态解算器
 * @param  sample_freq  调用 AHRS_Update 的频率 (Hz), 应匹配 TIM6 中断频率
 * @param  Kp           比例增益 (默认 1.0f 即可, 太大 → 高频振动耦合)
 * @param  Ki           积分增益 (默认 0.005f, 用于消除陀螺仪常值零偏)
 */
void AHRS_Init(float sample_freq, float Kp, float Ki);

/**
 * @brief  输入 IMU 数据，更新姿态四元数（无磁力计）
 */
void AHRS_Update(float gx, float gy, float gz,
                 float ax, float ay, float az);

/**
 * @brief  输入 IMU + 磁力计数据，更新姿态四元数（yaw 由磁力计锁定）
 * @param  mx, my, mz  磁力计 (μT, 任意单位, 会被归一化)
 */
void AHRS_Update_With_Mag(float gx, float gy, float gz,
                           float ax, float ay, float az,
                           float mx, float my, float mz);

/**
 * @brief  便捷函数：直接从 BMI088 读取并更新
 * @note   在 1ms 中断中调用此函数即可
 */
void AHRS_Update_From_BMI088(void);

/* ── 角度获取 ── */
float AHRS_Get_Roll(void);
float AHRS_Get_Pitch(void);
float AHRS_Get_Yaw(void);

/**
 * @brief  用加速度计硬算初始 roll/pitch，直接赋四元数初值
 * @param  ax, ay, az  机体坐标系加速度 (m/s²), 静止时为重力分量
 * @note   替代 [1,0,0,0] 盲目启动，校准后调用一次即可
 */
void AHRS_Init_From_Accel(float ax, float ay, float az);

/**
 * @brief  重置 yaw 角为 0
 */
void AHRS_Reset_Yaw(void);

#endif /* __AHRS_H */

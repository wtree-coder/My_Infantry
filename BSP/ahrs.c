#include "ahrs.h"
#include "BMI088driver.h"
#include <math.h>

AHRS_t ahrs;

/* ── 内部状态 ── */
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;        // 四元数
static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;          // 误差积分 (零偏估计)
static float yaw_prev_raw = 0.0f;                               // yaw 解缠: 上一帧原始值
static int   yaw_rounds   = 0;                                  // yaw 解缠: 跨零点圈数
static float invSqrt(float x);

//快速平方根倒数 (Newton 迭代一次)
static float invSqrt(float x)
{
    float halfx = 0.5f * x;
    long i  = *(long*)&x;
    i  = 0x5f3759df - (i >> 1);
    x  = *(float*)&i;
    x  = x * (1.5f - halfx * x * x);
    return x;
}

void AHRS_Init(float sample_freq, float _Kp, float _Ki)
{
    ahrs.sample_freq = sample_freq;
    ahrs.halfT       = 0.5f / sample_freq;
    ahrs.Kp          = _Kp;
    ahrs.Ki          = _Ki;

    q0 = 1.0f;  q1 = 0.0f;  q2 = 0.0f;  q3 = 0.0f;
    exInt = 0.0f;  eyInt = 0.0f;  ezInt = 0.0f;

    ahrs.roll  = 0.0f;
    ahrs.pitch = 0.0f;
    ahrs.yaw   = 0.0f;
    ahrs.is_ok = 0;
}

//用加速度计硬算初始姿态, 直接赋四元数初值
void AHRS_Init_From_Accel(float ax, float ay, float az)
{
    float norm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= norm;  ay *= norm;  az *= norm;

    /* 由静止重力矢量硬算 roll / pitch */
    float roll  = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
    float yaw   = 0.0f;

    /* Euler (Z-Y-X, yaw=0) → 四元数 */
    float cy = cosf(yaw * 0.5f), sy = sinf(yaw * 0.5f);  // cy=1, sy=0
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f),  sr = sinf(roll * 0.5f);

    q0 = cp * cr;
    q1 = cp * sr;
    q2 = sp * cr;
    q3 = -sp * sr;

    /* 重置积分器: 零偏已由校准环节扣除, 这里从零开始 */
    exInt = 0.0f;  eyInt = 0.0f;  ezInt = 0.0f;

    ahrs.roll  = roll;
    ahrs.pitch = pitch;
    ahrs.yaw   = 0.0f;
    ahrs.is_ok = 0;
}

/* Mahony 互补滤波核心 */
void AHRS_Update(float gx, float gy, float gz,
                 float ax, float ay, float az)
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;

    /* ── 自适应增益: 甩动时屏蔽 KI, 静止/匀速时全增益 ── */
    float accel_sq = ax * ax + ay * ay + az * az;
    int   trusted  = (accel_sq > 86.0f && accel_sq < 106.0f); // 9.8² ±5%
    float Kp_cur = trusted ? ahrs.Kp : 0.1f;
    float Ki_cur = trusted ? ahrs.Ki : 0.0f;

    /* ── 归一化加速度计 ── */
    norm = invSqrt(accel_sq);
    ax *= norm;
    ay *= norm;
    az *= norm;

    /* ── 从当前姿态估算重力方向 ── */
    vx = 2.0f * (q1 * q3 - q0 * q2);
    vy = 2.0f * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    /* ── 加速度计测量值与估算重力叉积 → 姿态误差 ── */
    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;

    /* ── PI 校正陀螺仪零偏 ── */
    exInt += ex * Ki_cur;
    eyInt += ey * Ki_cur;
    ezInt += ez * Ki_cur;

    gx += Kp_cur * ex + exInt;
    gy += Kp_cur * ey + eyInt;
    gz += Kp_cur * ez + ezInt;

    /* ── 一阶龙格库塔积分四元数 ── */
    q0 += (-q1 * gx - q2 * gy - q3 * gz) * ahrs.halfT;
    q1 += ( q0 * gx + q2 * gz - q3 * gy) * ahrs.halfT;
    q2 += ( q0 * gy - q1 * gz + q3 * gx) * ahrs.halfT;
    q3 += ( q0 * gz + q1 * gy - q2 * gx) * ahrs.halfT;

    /* ── 归一化四元数 ── */
    norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= norm;
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;

    /* ── 四元数 → 欧拉角 (Z-Y-X 旋转顺序) ── */
    ahrs.roll  = atan2f(2.0f * (q0 * q1 + q2 * q3),
                        1.0f - 2.0f * (q1 * q1 + q2 * q2));
    ahrs.pitch = asinf(2.0f * (q0 * q2 - q3 * q1));
    ahrs.yaw   = atan2f(2.0f * (q0 * q3 + q1 * q2),
                        1.0f - 2.0f * (q2 * q2 + q3 * q3));

    ahrs.is_ok = 1;
}

// Mahony + 磁力计 (yaw 锁定)
void AHRS_Update_With_Mag(float gx, float gy, float gz,
                           float ax, float ay, float az,
                           float mx, float my, float mz)
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;

    /* ── 自适应增益: 甩动时屏蔽 KI, 静止/匀速时全增益 ── */
    float accel_sq = ax * ax + ay * ay + az * az;
    int   trusted  = (accel_sq > 86.0f && accel_sq < 106.0f); // 9.8² ±5%
    float Kp_cur = trusted ? ahrs.Kp : 0.1f;
    float Ki_cur = trusted ? ahrs.Ki : 0.0f;

    /* ── 归一化加速度计 ── */
    norm = invSqrt(accel_sq);
    ax *= norm;  ay *= norm;  az *= norm;

    /* ── 从当前姿态估算重力方向 ── */
    vx = 2.0f * (q1 * q3 - q0 * q2);
    vy = 2.0f * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    /* ── 加计误差 → 修正 pitch/roll ── */
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    /* ── 磁力计: 倾斜补偿后仅修正 yaw ── */
    {
        float cos_r = cosf(ahrs.roll), sin_r = sinf(ahrs.roll);
        float cos_p = cosf(ahrs.pitch), sin_p = sinf(ahrs.pitch);

        /* 将磁力计投影到水平面 */
        float m_hx = mx * cos_p + my * sin_r * sin_p + mz * cos_r * sin_p;
        float m_hy = my * cos_r - mz * sin_r;

        /* 水平面地磁方向 */
        float mag_yaw = atan2f(-m_hy, m_hx);

        /* yaw 误差: 磁力计朝向 vs 当前 AHRS yaw */
        float yaw_err = mag_yaw - ahrs.yaw;
        if      (yaw_err >  PI) yaw_err -= 2.0f * PI;
        else if (yaw_err < -PI) yaw_err += 2.0f * PI;

        /* 仅 Z 轴加磁修正 */
        float mag_correction = 2.0f * yaw_err;
        if      (mag_correction >  4.0f) mag_correction =  4.0f;
        else if (mag_correction < -4.0f) mag_correction = -4.0f;
        gz += mag_correction;
    }

    /* ── PI 校正 (仅加计误差) ── */
    exInt += ex * Ki_cur;
    eyInt += ey * Ki_cur;
    ezInt += ez * Ki_cur;

    gx += Kp_cur * ex + exInt;
    gy += Kp_cur * ey + eyInt;
    gz += Kp_cur * ez + ezInt;

    /* ── 一阶龙格库塔积分四元数 ── */
    q0 += (-q1 * gx - q2 * gy - q3 * gz) * ahrs.halfT;
    q1 += ( q0 * gx + q2 * gz - q3 * gy) * ahrs.halfT;
    q2 += ( q0 * gy - q1 * gz + q3 * gx) * ahrs.halfT;
    q3 += ( q0 * gz + q1 * gy - q2 * gx) * ahrs.halfT;

    /* ── 归一化四元数 ── */
    norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= norm;  q1 *= norm;  q2 *= norm;  q3 *= norm;

    /* ── 四元数 → 欧拉角 ── */
    ahrs.roll  = atan2f(2.0f * (q0 * q1 + q2 * q3),
                        1.0f - 2.0f * (q1 * q1 + q2 * q2));
    ahrs.pitch = asinf(2.0f * (q0 * q2 - q3 * q1));
    ahrs.yaw   = atan2f(2.0f * (q0 * q3 + q1 * q2),
                        1.0f - 2.0f * (q2 * q2 + q3 * q3));

    ahrs.is_ok = 1;
}

// 便捷函数: 自动从 BMI088 取数 + 轴映射 (纯6轴)
void AHRS_Update_From_BMI088(void)
{
    if (!bmi088_data.is_ok)
    {
        ahrs.is_ok = 0;
        return;
    }

    /* 轴映射: BMI088(X右 Y前 Z上) → 机体(X前 Y左 Z上) */
    float gx =  bmi088_data.gyro[1];
    float gy = -bmi088_data.gyro[0];
    float gz =  bmi088_data.gyro[2];

    float ax =  bmi088_data.accel[1];
    float ay = -bmi088_data.accel[0];
    float az =  bmi088_data.accel[2];

    AHRS_Update(gx, gy, gz, ax, ay, az);
}

//角度获取
float AHRS_Get_Roll(void)  { return ahrs.roll;  }
float AHRS_Get_Pitch(void) { return ahrs.pitch; }
float AHRS_Get_Yaw(void)
{
    float raw = ahrs.yaw * 57.2957795f;       // rad → deg, 值域 [-180, +180]

    /* 过零检测: 差值超过 180° 说明跨越了 ±180° 边界 */
    float delta = raw - yaw_prev_raw;
    if      (delta >  180.0f) yaw_rounds--;    // +180→-180 跳变, 等效正转一圈
    else if (delta < -180.0f) yaw_rounds++;    // -180→+180 跳变, 等效反转一圈
    yaw_prev_raw = raw;

    return raw + (float)yaw_rounds * 360.0f;   // 解缠后的连续角度
}

//重置 yaw (底盘上电时清零)
void AHRS_Reset_Yaw(void)
{
    /* 绕 Z 轴旋转四元数使 yaw=0 */
    float half_yaw = -0.5f * ahrs.yaw;
    float cos_half = cosf(half_yaw);
    float sin_half = sinf(half_yaw);

    float nq0 = q0 * cos_half - q3 * sin_half;
    float nq1 = q1 * cos_half - q2 * sin_half;
    float nq2 = q1 * sin_half + q2 * cos_half;
    float nq3 = q0 * sin_half + q3 * cos_half;

    q0 = nq0;  q1 = nq1;  q2 = nq2;  q3 = nq3;

    /* 重新归一化 */
    float norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= norm;  q1 *= norm;  q2 *= norm;  q3 *= norm;

    ahrs.yaw = 0.0f;
    yaw_prev_raw = 0.0f;                       // 同步清零解缠状态
    yaw_rounds   = 0;
}

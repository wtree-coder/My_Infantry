#ifndef __PID_H
#define __PID_H

#include "main.h" // 包含标准库或单片机硬件定义

/* --- PID 默认参数宏定义 --- */
#define PID_DEFAULT_INTEGRAL_SEP_THRESH  100.0f  // 默认积分分离阈值
#define PID_DEFAULT_DERIV_FILTER_FACTOR  0.5f    // 默认微分滤波系数 (0-1)

/* --- PID 工作模式枚举 --- */
typedef enum {
    PID_MODE_POSITION = 0, // 位置式 PID
    PID_MODE_DELTA         // 增量式 PID
} PIDModeTypedef;

/* --- PID 控制器结构体 --- */
typedef struct {
    // PID 增益参数
    float Kp;
    float Ki;
    float Kd;

    // 目标值与当前状态
    float targetValue;
    float error;
    float previous_error;
    float prev_prev_error;     // 用于增量式 PID
    float previous_measurement; // 用于计算微分项

    // 积分与微分中间量
    float integral;
    float derivative;
    float filteredDerivative;   // 经过滤波后的微分值
    
    // 算法配置参数
    float sampleTime;           // 采样时间 (s)
    float invSampleTime;        // 采样时间的倒数 (1/s)，优化计算性能
    
    float outputMax;            // 输出上限
    float outputMin;            // 输出下限
    
    float integralMax;          // 积分项上限
    float integralMin;          // 积分项下限
    
    float deadBand;             // 误差死区
    float integralSeparationThreshold; // 积分分离阈值
    float derivativeFilterFactor;      // 微分滤波系数
    
    PIDModeTypedef mode;        // 当前工作模式
} PIDControllerTypedef;

/* --- 函数声明 --- */

// 初始化与基础操作
void PID_Init(PIDControllerTypedef *pid, float Kp, float Ki, float Kd, 
              float sampleTime, float outputlimits, float integralLimits, 
              float deadBand, PIDModeTypedef mode);
              
float PIDCompute(PIDControllerTypedef *pid, float now_Value, float target_Value);
void PID_Reset(PIDControllerTypedef *pid);

// 参数动态设置接口
void PID_SetSampleTime(PIDControllerTypedef *pid, float sampleTime);
void PID_SetOutputLimits(PIDControllerTypedef *pid, float min, float max);
void PID_SetIntegralLimits(PIDControllerTypedef *pid, float min, float max);
void PID_SetDeadBand(PIDControllerTypedef *pid, float deadBand);
void PID_SetIntegralSeparationThreshold(PIDControllerTypedef *pid, float threshold);
void PID_SetDerivativeFilterFactor(PIDControllerTypedef *pid, float factor);
void PID_SetMode(PIDControllerTypedef *pid, uint8_t mode);

#endif /* __PID_H */
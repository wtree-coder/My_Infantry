#include "fun.h"
#include "vofa.h"
#include "tim.h"
#include "drv_can.h"
#include "DR16.h"
#include "chassis.h"
#include "gimbal.h"
#include "usart.h"
#include "BMI088driver.h"
#include "ahrs.h"

void Main_Init(void)
{    
    Chassis_Init();
    Gimbal_Init();

    BMI088_Init();
    BMI088_Gyro_Offset_Calibrate();
    BMI088_Read_Update();

    AHRS_Init(1000.0f, 1.0f, 0.0f);
    AHRS_Init_From_Accel(bmi088_data.accel[1], -bmi088_data.accel[0], bmi088_data.accel[2]);
    AHRS_Reset_Yaw();

    CAN_Filter_Mask_Config(&hcan1, CAN_FILTER(0)| CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
    CAN_Init(&hcan1);

    Vofa_Init(&huart1);
    DR16_Init(&huart3);
    
    HAL_TIM_Base_Start_IT(&htim6);
}


void While_Proc(void)
{
    
}
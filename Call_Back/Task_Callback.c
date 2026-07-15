#include "Task_Callback.h"
#include "drv_can.h"
#include "DR16.h"
#include "chassis.h"
#include "gimbal.h"
#include "vofa.h"
#include "usart.h"
#include "BMI088driver.h"
#include "ahrs.h"

//1ms中断
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM6)
    {
        DR16_Check();
        Chassis_Check();
        Gimbal_Check();

        BMI088_Read_Update();
        AHRS_Update_From_BMI088();

        Vofa_Send_Callback(&huart1);

        Mode_Choose();

        Chassis_1ms_Callback();
        Gimbal_1ms_Callback();
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart->Instance == USART3)
    {
        if(Size == 18)
        {
            DR16_Data_Get(DR16.Rx_Buff, &DR16_Data);            
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, DR16.Rx_Buff, UART_MAX_SIZE);
    }
}

//获取电调数据
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    static CAN_Rx_Buffer_t rx_buffer;
    if(hcan->Instance == CAN1)
    {
        //这是选择读哪个FIFO，而初始化是选择消息进哪个FIFO
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_buffer.Header, rx_buffer.Data);
        if (rx_buffer.Header.StdId >= 0x201 && rx_buffer.Header.StdId <= 0x204)
        {
            Chassis_Rx_Callback(&rx_buffer);
        }
        else if(rx_buffer.Header.StdId == 0x208)
        {
            Gimbal_Rx_Callback(&rx_buffer);
        }
    }
}

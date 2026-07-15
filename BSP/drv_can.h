#ifndef DRV_CAN_H
#define DRV_CAN_H

#include "stm32f4xx_hal.h"

// 滤波器编号
#define CAN_FILTER(x) ((x) << 3)

// 接收队列
#define CAN_FIFO_0 (0 << 2)
#define CAN_FIFO_1 (1 << 2)

// 标准帧或扩展帧
#define CAN_STDID (0 << 1)
#define CAN_EXTID (1 << 1)

// 数据帧或遥控帧
#define CAN_DATA_TYPE (0 << 0)
#define CAN_REMOTE_TYPE (1 << 0)

/**
 * @brief CAN接收信息结构体
 */
typedef struct
{
    CAN_RxHeaderTypeDef Header;
    uint8_t Data[8];
} CAN_Rx_Buffer_t;


extern CAN_HandleTypeDef hcan1;
//extern CAN_HandleTypeDef hcan2;

extern uint8_t CAN1_0x1ff_Tx_Data[8];
extern uint8_t CAN1_0x200_Tx_Data[8];

void CAN_Init(CAN_HandleTypeDef *hcan);
void CAN_Filter_Mask_Config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID);
uint8_t CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length);

//void TIM_CAN_PeriodElapsedCallback(void);

#endif
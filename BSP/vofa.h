#ifndef VOFA_H
#define VOFA_H

#include "stm32f4xx_hal.h"
#include <stdarg.h>

#define VOFA_RX_BUFF_SIZE 128
#define VOFA_TX_BUFF_SIZE 128

/**
 * @brief VOFA+ 数据核心结构体
 */
typedef struct
{
    // 缓冲区
    uint8_t Rx_Buff[VOFA_RX_BUFF_SIZE];
    uint8_t Tx_Buff[VOFA_TX_BUFF_SIZE];

    const float *Tx_Data_Ptrs[12]; // 保存要发送的浮点数指针
    uint8_t Tx_Data_Num;
} Vofa_t;

extern Vofa_t vofa;

void Vofa_Init(UART_HandleTypeDef *huart);
void Vofa_Set_Data(int Number, ...);
void Vofa_Send_Data(UART_HandleTypeDef *huart);
void Vofa_Send_Callback(UART_HandleTypeDef *huart);

#endif
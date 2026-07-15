#include "vofa.h"
#include <string.h>
#include "usart.h"
#include "chassis.h"
#include "gimbal.h"
#include "ahrs.h"

Vofa_t vofa;

void Vofa_Init(UART_HandleTypeDef *huart)
{
    HAL_UARTEx_ReceiveToIdle_DMA(huart, vofa.Rx_Buff, VOFA_RX_BUFF_SIZE);
}

void Vofa_Set_Data(int Number, ...)
{
    va_list data_ptr;
    va_start(data_ptr, Number);
    
    if (Number > 12) Number = 12; // 保护数组不越界
    
    for (int i = 0; i < Number; i++)
    {
        // va_arg: 用 void* 取出再强转为 const float*
        // 可变参数传的是指针，用 void* 是因为不检查具体类型，传 float*、int* 等任意指针都能通用接住
        vofa.Tx_Data_Ptrs[i] = (const float *)va_arg(data_ptr, void *);
    }
    vofa.Tx_Data_Num = Number;
    
    va_end(data_ptr);
}

void Vofa_Send_Data(UART_HandleTypeDef *huart)
{
    if (vofa.Tx_Data_Num == 0) return;

    for (int i = 0; i < vofa.Tx_Data_Num; i++)
    {
        // Tx_Buff 是数组首地址(指针), + i*4 即偏移 i 个 float 的位置(每个 float 占 4 字节)
        memcpy(vofa.Tx_Buff + i * sizeof(float), vofa.Tx_Data_Ptrs[i], sizeof(float));
    }

    uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};
    memcpy(vofa.Tx_Buff + vofa.Tx_Data_Num * sizeof(float), tail, 4);

    uint16_t send_len = vofa.Tx_Data_Num * sizeof(float) + 4;
    HAL_UART_Transmit_DMA(huart, vofa.Tx_Buff, send_len);
}


void Vofa_Send_Callback(UART_HandleTypeDef *huart)
{
    static uint8_t send_count = 0;

    if (++send_count >= 20)
    {
        send_count = 0;
        float roll  = AHRS_Get_Roll();
        float pitch = AHRS_Get_Pitch();
        float yaw   = AHRS_Get_Yaw();
        float total_encoder = (float)gimbal.motor_6020.total_encoder;
        float motor_6020_now_angle = (float)gimbal.motor_6020.now_angle;
        Vofa_Set_Data(5, &roll, &pitch, &yaw, &total_encoder, &motor_6020_now_angle);
        Vofa_Send_Data(&huart1);
        /*
        Vofa_Set_Data(5,
            &chassis.motor_chassis[0].now_rad_s,
            &chassis.motor_chassis[1].now_rad_s,
            &chassis.motor_chassis[2].now_rad_s,
            &chassis.motor_chassis[3].now_rad_s,
            &gimbal.motor_6020.now_rad_s);
        
            Vofa_Send_Data(huart);
        */
    }
}
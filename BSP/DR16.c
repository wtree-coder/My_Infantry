#include "DR16.h"
#include "usart.h"

DR16_t DR16;
DR16_Data_t DR16_Data;

static volatile uint16_t check_count = 0;

void DR16_Init(UART_HandleTypeDef *huart)
{
    HAL_UARTEx_ReceiveToIdle_DMA(huart, DR16.Rx_Buff, UART_MAX_SIZE);
}

void DR16_Data_Get(uint8_t *Rx_Buff, DR16_Data_t *Data)
{
    if (Rx_Buff == NULL || Data == NULL) return;

    check_count = 0;

    Data->is_ok = 1;

    int16_t ch;
    //11 个 1
    ch = ((Rx_Buff[0] | (Rx_Buff[1] << 8)) & 0x07FF) - 1024;
    Data->Right_X = (float)ch / 660.0f;

    ch = (((Rx_Buff[1] >> 3) | (Rx_Buff[2] << 5)) & 0x07FF) - 1024;
    Data->Right_Y = (float)ch / 660.0f;

    ch = (((Rx_Buff[2] >> 6) | (Rx_Buff[3] << 2) | (Rx_Buff[4] << 10)) & 0x07FF) - 1024;
    Data->Left_X = (float)ch / 660.0f;

    ch = (((Rx_Buff[4] >> 1) | (Rx_Buff[5] << 7)) & 0x07FF) - 1024;
    Data->Left_Y = (float)ch / 660.0f;

    Data->Chassis_Switch = (Enum_DR16_Switch_Status)((Rx_Buff[5] >> 4) & 0x03);
    Data->Gimbal_Switch  = (Enum_DR16_Switch_Status)((Rx_Buff[5] >> 6) & 0x03);

    ch = ((Rx_Buff[16] | (Rx_Buff[17] << 8)) & 0x07FF) - 1024;
    Data->Yaw = (float)ch / 660.0f;
}

float DR16_XiaoZhun(float input)
{
    if(input > -DR16_DEADZONE && input < DR16_DEADZONE) return 0.0f;
    return input;
}

void DR16_Check(void)
{
    if (check_count < 50)
    {
        check_count++;
    }
    else
    {
        DR16_Data.is_ok = 0;

        DR16_Data.Right_X = 0.0f;
        DR16_Data.Right_Y = 0.0f;
        DR16_Data.Left_X  = 0.0f;
        DR16_Data.Left_Y  = 0.0f;
        DR16_Data.Yaw     = 0.0f;

        DR16_Data.Chassis_Switch = DR16_SWITCH_DOWN;
        DR16_Data.Gimbal_Switch  = DR16_SWITCH_DOWN;
    }
}

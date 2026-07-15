#ifndef BMI088DRIVER_H
#define BMI088DRIVER_H

#include "main.h"

#ifndef PI
#define PI 3.1415926535f
#endif

#define BMI088_USE_SPI
//#define BMI088_USE_IIC

/* ═════════════════════════════════════════════════════════════════
 *  SPI 片选宏
 * ═════════════════════════════════════════════════════════════════ */
#define BMI088_ACCEL_CS_LOW()   HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_RESET)
#define BMI088_ACCEL_CS_HIGH()  HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_SET)
#define BMI088_GYRO_CS_LOW()    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET)
#define BMI088_GYRO_CS_HIGH()   HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET)

#define BMI088_TEMP_FACTOR 0.125f
#define BMI088_TEMP_OFFSET 23.0f

#define BMI088_WRITE_ACCEL_REG_NUM  6
#define BMI088_WRITE_GYRO_REG_NUM   6

#define BMI088_GYRO_DATA_READY_BIT          0
#define BMI088_ACCEL_DATA_READY_BIT         1
#define BMI088_ACCEL_TEMP_DATA_READY_BIT    2

#define BMI088_LONG_DELAY_TIME      80
#define BMI088_COM_WAIT_SENSOR_TIME 150


#define BMI088_ACCEL_IIC_ADDRESSE   (0x18 << 1)
#define BMI088_GYRO_IIC_ADDRESSE    (0x68 << 1)

#define BMI088_ACCEL_RANGE_3G
//#define BMI088_ACCEL_RANGE_6G
//#define BMI088_ACCEL_RANGE_12G
//#define BMI088_ACCEL_RANGE_24G

#define BMI088_GYRO_RANGE_2000
//#define BMI088_GYRO_RANGE_1000
//#define BMI088_GYRO_RANGE_500
//#define BMI088_GYRO_RANGE_250
//#define BMI088_GYRO_RANGE_125


#define BMI088_ACCEL_3G_SEN     0.0008974358974f
#define BMI088_ACCEL_6G_SEN     0.00179443359375f
#define BMI088_ACCEL_12G_SEN    0.0035888671875f
#define BMI088_ACCEL_24G_SEN    0.007177734375f


#define BMI088_GYRO_2000_SEN    0.00106526443603169529841533860381f
#define BMI088_GYRO_1000_SEN    0.00053263221801584764920766930190693f
#define BMI088_GYRO_500_SEN     0.00026631610900792382460383465095346f
#define BMI088_GYRO_250_SEN     0.00013315805450396191230191732547673f
#define BMI088_GYRO_125_SEN     0.000066579027251980956150958662738366f


typedef struct __attribute__((packed)) BMI088_RAW_DATA
{
    uint8_t status;
    int16_t accel[3];
    int16_t temp;
    int16_t gyro[3];
} bmi088_raw_data_t;

typedef struct BMI088_REAL_DATA
{
    uint8_t status;
    float accel[3];
    float temp;
    float gyro[3];
    float time;
} bmi088_real_data_t;


enum
{
    BMI088_NO_ERROR                     = 0x00,
    BMI088_ACC_PWR_CTRL_ERROR           = 0x01,
    BMI088_ACC_PWR_CONF_ERROR           = 0x02,
    BMI088_ACC_CONF_ERROR               = 0x03,
    BMI088_ACC_SELF_TEST_ERROR          = 0x04,
    BMI088_ACC_RANGE_ERROR              = 0x05,
    BMI088_INT1_IO_CTRL_ERROR           = 0x06,
    BMI088_INT_MAP_DATA_ERROR           = 0x07,
    BMI088_GYRO_RANGE_ERROR             = 0x08,
    BMI088_GYRO_BANDWIDTH_ERROR         = 0x09,
    BMI088_GYRO_LPM1_ERROR              = 0x0A,
    BMI088_GYRO_CTRL_ERROR              = 0x0B,
    BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
    BMI088_GYRO_INT3_INT4_IO_MAP_ERROR  = 0x0D,

    BMI088_SELF_TEST_ACCEL_ERROR        = 0x80,
    BMI088_SELF_TEST_GYRO_ERROR         = 0x40,
    BMI088_NO_SENSOR                    = 0xFF,
};






extern uint8_t BMI088_Init(void);
extern uint8_t bmi088_accel_self_test(void);
extern uint8_t bmi088_gyro_self_test(void);
extern uint8_t bmi088_accel_init(void);
extern uint8_t bmi088_gyro_init(void);

extern void BMI088_accel_read_over(uint8_t *rx_buf, float accel[3], float *time);
extern void BMI088_gyro_read_over(uint8_t *rx_buf, float gyro[3]);
extern void BMI088_temperature_read_over(uint8_t *rx_buf, float *temperate);
extern void BMI088_read(float gyro[3], float accel[3], float *temperate);
extern uint32_t get_BMI088_sensor_time(void);
extern float get_BMI088_temperate(void);
extern void get_BMI088_gyro(int16_t gyro[3]);
extern void get_BMI088_accel(float accel[3]);


extern void BMI088_read_gyro_who_am_i(void);
extern void BMI088_read_accel_who_am_i(void);

/* ── 全局数据容器 (供 AHRS / 回调 跨文件访问) ── */
typedef struct
{
    float gyro[3];        // rad/s, 最新陀螺仪数据 (已扣零偏+过死区)
    float accel[3];       // m/s², 最新加速度计数据
    float temp;           // °C,   最新温度
    float gyro_offset[3]; // rad/s, 陀螺零偏 (上电静止校准)
    uint8_t is_ok;       // 1 = 传感器正常
} bmi088_data_t;

extern bmi088_data_t bmi088_data;

#define BMI088_OFFSET_SAMPLES      2000    // 零偏校准采样次数
#define BMI088_GYRO_DEADBAND       0.001f  // rad/s 死区 (~0.06 deg/s)

void BMI088_Read_Update(void);              // 阻塞读取 → 更新 bmi088_data
void BMI088_Gyro_Offset_Calibrate(void);    // 静止零偏校准 (上电调用)

#endif

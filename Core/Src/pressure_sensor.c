#include "pressure_sensor.h"
#include "modbus_crc.h"

extern UART_HandleTypeDef huart3;

/* 调试变量 */
volatile uint8_t  debug_raw[10] = {0};
volatile uint16_t debug_len = 0;
volatile HAL_StatusTypeDef debug_rx_status = HAL_ERROR;

/* 传感器数据 */
volatile int16_t  pressure_raw = 0;
volatile float    pressure_value = 0.0f;

volatile uint8_t  sensor_state = 0;
volatile uint32_t sensor_error_cnt = 0;

static float pressure_filtered = 0;    
static uint8_t first_reading = 1;      

/* 读取一次压力值（阻塞方式，约50ms内完成） */
void Sensor_ReadOnce(void)
{
    uint8_t tx_cmd[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A};
    uint8_t rx_buf[10] = {0};
    HAL_StatusTypeDef status;

    HAL_GPIO_WritePin(RS485_DE_RE_3_GPIO_Port, RS485_DE_RE_3_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_UART_Transmit(&huart3, tx_cmd, 8, 50);
    HAL_Delay(1);

    HAL_GPIO_WritePin(RS485_DE_RE_3_GPIO_Port, RS485_DE_RE_3_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);

    status = HAL_UART_Receive(&huart3, rx_buf, 7, 100);

    debug_rx_status = (uint8_t)status;
    debug_len = 7;
    for (int i = 0; i < 7; i++) {
        debug_raw[i] = rx_buf[i];
    }

    if (status == HAL_OK &&
        rx_buf[0] == 0x01 &&      // 地址正确
        rx_buf[1] == 0x03 &&      // 功能码正确
        rx_buf[2] == 0x02)        // 数据长度正确
    {
        uint16_t raw = (rx_buf[3] << 8) | rx_buf[4];

    if (raw <= 5000)
    {
        float new_value = raw * 0.1f;

            /* 低通滤波: 新值 = 70%旧值 + 30%新读数 */
        if (first_reading) {
            pressure_filtered = new_value;
            first_reading = 0;
        } 
        else {
            pressure_filtered = 0.7f * pressure_filtered + 0.3f * new_value;
        }

        pressure_raw = raw;
        pressure_value = pressure_filtered;
    }
        /* 不合理的值直接丢弃，保持上一次的 pressure_value */
    }
    /* 通信失败也丢弃，保持上一次的值 */
}

void Sensor_Init(void)
{
    RS485_RECEIVE();
    sensor_state = 0;
    sensor_error_cnt = 0;
}
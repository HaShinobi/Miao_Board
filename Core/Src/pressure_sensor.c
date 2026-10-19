#include "pressure_sensor.h"
#include "modbus_crc.h"

extern UART_HandleTypeDef huart3;

/* 调试变量 */
volatile uint8_t  debug_raw[10] = {0};
volatile uint16_t debug_len = 0;
volatile HAL_StatusTypeDef debug_rx_status = HAL_ERROR;

/* 传感器数据 - two T3000 sensors share RS485-U3, told apart by Modbus address */
volatile int16_t  pressure_raw = 0;          // base,    address 1
volatile float    pressure_value = 0.0f;
volatile int16_t  pressure_tip_raw = 0;      // end-tip, address 2
volatile float    pressure_tip_value = 0.0f;
volatile uint8_t  pressure_tip_enable = 1;   // 0 = skip the tip poll (sensor not fitted)

volatile uint8_t  sensor_state = 0;
volatile uint32_t sensor_error_cnt = 0;      // base: no/bad reply count
volatile uint32_t sensor_tip_error_cnt = 0;  // end-tip: no/bad reply count

typedef struct {
    float   filtered;
    uint8_t first_reading;
} PressureFilter_t;

static PressureFilter_t filt_base = {0.0f, 1};
static PressureFilter_t filt_tip  = {0.0f, 1};

/* One Modbus RTU transaction on RS485-U3, blocking. `req` is the 6-byte
   request without CRC (address, function, 2-byte register, 2-byte value);
   the CRC is appended here. Waits for `rx_len` reply bytes (0 = no reply
   expected, e.g. a broadcast) and returns 1 only if they arrive within
   `timeout_ms`, come from req[0] with the same function code and pass the
   CRC. Shared by the pressure poll and the ID tool (pressure_id.c). */
uint8_t Sensor_Modbus(const uint8_t req[6], uint8_t *rx, uint8_t rx_len,
                      uint32_t timeout_ms)
{
    uint8_t tx_cmd[8];
    HAL_StatusTypeDef status;

    for (int i = 0; i < 6; i++) tx_cmd[i] = req[i];
    uint16_t crc = Modbus_CRC16(tx_cmd, 6);
    tx_cmd[6] = crc & 0xFF;
    tx_cmd[7] = crc >> 8;

    /* Modbus needs >= 3.5 char times (~4 ms at 9600) of silence between
       frames, or the other sensor's reply and this request run together. */
    HAL_Delay(4);

    /* Drop anything left over from a late reply so the bytes line up. */
    __HAL_UART_CLEAR_FLAG(&huart3, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                   UART_CLEAR_FEF  | UART_CLEAR_PEF);
    __HAL_UART_SEND_REQ(&huart3, UART_RXDATA_FLUSH_REQUEST);

    HAL_GPIO_WritePin(RS485_DE_RE_3_GPIO_Port, RS485_DE_RE_3_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_UART_Transmit(&huart3, tx_cmd, 8, 50);
    HAL_Delay(1);

    HAL_GPIO_WritePin(RS485_DE_RE_3_GPIO_Port, RS485_DE_RE_3_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);

    if (rx_len == 0) return 1;

    status = HAL_UART_Receive(&huart3, rx, rx_len, timeout_ms);

    debug_rx_status = (uint8_t)status;       // last transaction, either sensor
    debug_len = rx_len;
    for (int i = 0; i < rx_len && i < 10; i++) {
        debug_raw[i] = rx[i];
    }

    if (status != HAL_OK ||
        rx[0] != req[0] ||        // 地址正确
        rx[1] != req[1])          // 功能码正确 (0x83/0x86 = sensor error reply)
        return 0;

    if (Modbus_CRC16(rx, rx_len - 2) !=
        (uint16_t)(rx[rx_len - 2] | (rx[rx_len - 1] << 8)))
        return 0;

    return 1;
}

/* One 0x03 read of register 0x0000 (pressure) from `addr`, ~25 ms with a
   reply, 100 ms on timeout. */
static uint8_t Sensor_ReadRaw(uint8_t addr, uint16_t *raw_out)
{
    const uint8_t req[6] = {addr, 0x03, 0x00, 0x00, 0x00, 0x01};
    uint8_t rx_buf[7] = {0};

    if (!Sensor_Modbus(req, rx_buf, 7, 100) ||
        rx_buf[2] != 0x02)        // 数据长度正确
        return 0;

    *raw_out = (rx_buf[3] << 8) | rx_buf[4];
    return 1;
}

/* 低通滤波: 新值 = 70%旧值 + 30%新读数. Out-of-range values are dropped
   and the last value kept. */
static uint8_t Sensor_Filter(PressureFilter_t *f, uint16_t raw)
{
    if (raw > 5000) return 0;

    float new_value = raw * 0.1f;
    if (f->first_reading) {
        f->filtered = new_value;
        f->first_reading = 0;
    } else {
        f->filtered = 0.7f * f->filtered + 0.3f * new_value;
    }
    return 1;
}

/* 读取一次基座压力值 (address 1) -> pressure_value */
void Sensor_ReadOnce(void)
{
    uint16_t raw;
    if (Sensor_ReadRaw(PRESSURE_ADDR_BASE, &raw) && Sensor_Filter(&filt_base, raw)) {
        pressure_raw = raw;
        pressure_value = filt_base.filtered;
    } else {
        sensor_error_cnt++;                  /* 通信失败: 保持上一次的值 */
    }
}

/* 读取一次末端压力值 (address 2) -> pressure_tip_value */
void Sensor_ReadTip(void)
{
    if (!pressure_tip_enable) return;

    uint16_t raw;
    if (Sensor_ReadRaw(PRESSURE_ADDR_TIP, &raw) && Sensor_Filter(&filt_tip, raw)) {
        pressure_tip_raw = raw;
        pressure_tip_value = filt_tip.filtered;
    } else {
        sensor_tip_error_cnt++;
    }
}

void Sensor_Init(void)
{
    RS485_RECEIVE();
    sensor_state = 0;
    sensor_error_cnt = 0;
}
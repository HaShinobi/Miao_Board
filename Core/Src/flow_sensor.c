#include "flow_sensor.h"
#include "modbus_crc.h"
#include "string.h"

// Flow sensor is now on USART2 (RS485-U2: PD5 TX, PD6 RX, PD4 DE)
extern UART_HandleTypeDef huart2;

#define RS485_DE_RE_GPIO_Port   RS485_DE_RE_2_GPIO_Port   // GPIOD
#define RS485_DE_RE_Pin         RS485_DE_RE_2_Pin         // GPIO_PIN_4

#define MODBUS_ADDR     0x01
#define MODBUS_FC03     0x03
#define FLOW_REG        0x0002
#define FLOW_REG_COUNT  0x0002

static uint8_t tx_buf[8];
static uint8_t rx_buf[9];

// ---- Dry-test flow simulation (Ozone-tunable) ----
// flow_sim_enable = 1: FlowSensor_ReadFlowRate_m3s returns flow_sim_Lmin
// (signed, L/min — the real sensor's unit; + = fill, − = drain) and SKIPS the
// blocking Modbus transaction entirely. Lets arm_state 2 etc. run without
// water. Reference: 1.0 L/min = PC_Q_MAX = 16.67 mL/s.
volatile uint8_t flow_sim_enable = 0;
volatile float   flow_sim_Lmin   = 0.0f;

void FlowSensor_Init(void)
{
    // USART2 and PD4 already initialized in main.c / stm32h7xx_hal_msp.c
}

float FlowSensor_ReadFlowRate_m3s(float dt)
{
    (void)dt;

    if (flow_sim_enable) {
        return flow_sim_Lmin / 3600000.0f;   // same conversion as the real path
    }

    tx_buf[0] = MODBUS_ADDR;
    tx_buf[1] = MODBUS_FC03;
    tx_buf[2] = (FLOW_REG >> 8) & 0xFF;
    tx_buf[3] = FLOW_REG & 0xFF;
    tx_buf[4] = (FLOW_REG_COUNT >> 8) & 0xFF;
    tx_buf[5] = FLOW_REG_COUNT & 0xFF;
    uint16_t crc = Modbus_CRC16(tx_buf, 6);
    tx_buf[6] = crc & 0xFF;
    tx_buf[7] = (crc >> 8) & 0xFF;

    HAL_GPIO_WritePin(RS485_DE_RE_GPIO_Port, RS485_DE_RE_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_UART_Transmit(&huart2, tx_buf, 8, 100);
    HAL_Delay(1);
    HAL_GPIO_WritePin(RS485_DE_RE_GPIO_Port, RS485_DE_RE_Pin, GPIO_PIN_RESET);

    memset(rx_buf, 0, sizeof(rx_buf));
    HAL_StatusTypeDef status = HAL_UART_Receive(&huart2, rx_buf, 9, 200);

    if (status != HAL_OK) return 0.0f;

    uint16_t rx_crc = (rx_buf[8] << 8) | rx_buf[7];
    if (Modbus_CRC16(rx_buf, 7) != rx_crc) return 0.0f;

    uint32_t raw = ((uint32_t)rx_buf[3] << 24) |
                   ((uint32_t)rx_buf[4] << 16) |
                   ((uint32_t)rx_buf[5] << 8)  |
                    (uint32_t)rx_buf[6];
    float flow_Lmin;
    memcpy(&flow_Lmin, &raw, sizeof(float));

    return flow_Lmin / 3600000.0f;
}

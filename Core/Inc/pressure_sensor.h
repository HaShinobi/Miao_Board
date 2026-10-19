#ifndef PRESSURE_SENSOR_H
#define PRESSURE_SENSOR_H

#include "main.h"
#include <string.h>

#define RS485_TRANSMIT()  HAL_GPIO_WritePin(RS485_DE_RE_3_GPIO_Port, RS485_DE_RE_3_Pin, GPIO_PIN_SET)
#define RS485_RECEIVE()   HAL_GPIO_WritePin(RS485_DE_RE_3_GPIO_Port, RS485_DE_RE_3_Pin, GPIO_PIN_RESET)

/* RS485-U3 Modbus addresses (register 0x0005 on the T3000, factory default 1) */
#define PRESSURE_ADDR_BASE  0x01
#define PRESSURE_ADDR_TIP   0x02

/* Filtered reading from the last Sensor_ReadOnce(), [kPa]. Was previously
   only reachable via a hand-written extern in main.c - declared here so
   every consumer (e.g. arm_states.c) agrees on the type with the definition
   in pressure_sensor.c. */
extern volatile float pressure_value;        // base chamber    (address 1)
extern volatile float pressure_tip_value;    // end-tip chamber (address 2) [kPa]
extern volatile uint8_t pressure_tip_enable; // 0 = skip the tip poll

void Sensor_Init(void);
void Sensor_ReadOnce(void);   // base sensor -> pressure_value
void Sensor_ReadTip(void);    // end-tip sensor -> pressure_tip_value
uint8_t Sensor_TipFresh(void); // 1 = pressure_tip_value is live (good read < 1 s ago)
uint8_t Sensor_Modbus(const uint8_t req[6], uint8_t *rx, uint8_t rx_len,
                      uint32_t timeout_ms);   // raw transaction, CRC added/checked

#endif
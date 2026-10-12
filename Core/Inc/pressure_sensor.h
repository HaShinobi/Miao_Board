#ifndef PRESSURE_SENSOR_H
#define PRESSURE_SENSOR_H

#include "main.h"
#include <string.h>

#define RS485_TRANSMIT()  HAL_GPIO_WritePin(RS485_DE_RE_3_GPIO_Port, RS485_DE_RE_3_Pin, GPIO_PIN_SET)
#define RS485_RECEIVE()   HAL_GPIO_WritePin(RS485_DE_RE_3_GPIO_Port, RS485_DE_RE_3_Pin, GPIO_PIN_RESET)

#define SENSOR_SLAVE_ID   0x01

/* Filtered reading from the last Sensor_ReadOnce(), [kPa]. Was previously
   only reachable via a hand-written extern in main.c - declared here so
   every consumer (e.g. arm_states.c) agrees on the type with the definition
   in pressure_sensor.c. */
extern volatile float pressure_value;

void Sensor_Init(void);
void Sensor_ReadOnce(void);

#endif
// file: modbus_crc.h
// Modbus RTU CRC-16 (poly 0xA001, init 0xFFFF), shared by the flow and
// pressure sensors - both talk Modbus over RS485. Header-only so there is
// no extra .c to add to the build.

#ifndef INC_MODBUS_CRC_H_
#define INC_MODBUS_CRC_H_

#include <stdint.h>

static inline uint16_t Modbus_CRC16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 0x0001) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
        }
    }
    return crc;
}

#endif /* INC_MODBUS_CRC_H_ */

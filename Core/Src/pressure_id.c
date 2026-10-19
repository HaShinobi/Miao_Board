// file: pressure_id.c
// Pressure sensor ID tool - see pressure_id.h for how to drive it.

#include "pressure_id.h"
#include "pressure_sensor.h"
#include "pump_application_function.h"

#define PSID_REG_ADDR   0x05    // register 0x0005: RS485 slave address
#define PSID_ADDR_MAX   247     // highest legal Modbus slave address
#define PSID_SCAN_MS    50      // per-address reply timeout while scanning
#define PSID_REPLY_MS   100     // reply timeout for the change and verify

volatile uint8_t psid_cmd     = 0;
volatile uint8_t psid_old     = 1;
volatile uint8_t psid_new     = 2;
volatile uint8_t psid_status  = PSID_IDLE;
volatile uint8_t psid_found   = 0;
volatile uint8_t psid_scan_at = 0;

/* Read register 0x0005 from `addr`: 1 if it answered, its stored address
   in *val. */
static uint8_t PSID_ReadAddr(uint8_t addr, uint32_t timeout_ms, uint8_t *val)
{
  const uint8_t req[6] = {addr, 0x03, 0x00, PSID_REG_ADDR, 0x00, 0x01};
  uint8_t rx[7] = {0};

  if (!Sensor_Modbus(req, rx, 7, timeout_ms) || rx[2] != 0x02) return 0;
  *val = rx[4];
  return 1;
}

static void PSID_Scan(void)
{
  uint8_t val;
  for (uint16_t a = 1; a <= PSID_ADDR_MAX; a++) {
    psid_scan_at = (uint8_t)a;
    if (PSID_ReadAddr((uint8_t)a, PSID_SCAN_MS, &val)) {
      psid_found  = (uint8_t)a;
      psid_status = PSID_FOUND;
      return;
    }
  }
  psid_status = PSID_NONE_FOUND;
}

/* Datasheet 0x06 write, e.g. 1 -> 2:  01 06 00 05 00 02 18 0A.
   The sensor echoes the request on success; a broadcast (address 0) gets
   no reply, so it is only checked by the read-back. */
static void PSID_Change(uint8_t old_addr, uint8_t new_addr)
{
  const uint8_t req[6] = {old_addr, 0x06, 0x00, PSID_REG_ADDR, 0x00, new_addr};
  uint8_t rx[8] = {0};
  uint8_t val;

  if (old_addr == 0) {
    Sensor_Modbus(req, rx, 0, 0);
  } else {
    if (!Sensor_Modbus(req, rx, 8, PSID_REPLY_MS)) { psid_status = PSID_NO_ECHO; return; }
    for (int i = 2; i < 6; i++)
      if (rx[i] != req[i]) { psid_status = PSID_NO_ECHO; return; }
  }

  HAL_Delay(100);   // let the sensor store and switch to the new address

  if (PSID_ReadAddr(new_addr, PSID_REPLY_MS, &val) && val == new_addr) {
    psid_found  = new_addr;
    psid_status = PSID_OK;
  } else {
    psid_status = PSID_VERIFY_FAIL;
  }
}

void PressureID_Task(void)
{
  uint8_t cmd = psid_cmd;
  if (cmd == 0) return;

  psid_found  = 0;
  psid_status = PSID_BUSY;

  /* The tool blocks the loop - nothing should be pumping meanwhile. */
  PumpStopAll();
  TipPumpStopAll();

  if (cmd == 2) {
    PSID_Scan();
  } else if (cmd == 1) {
    uint8_t o = psid_old, n = psid_new;
    if (n < 1 || n > PSID_ADDR_MAX || o > PSID_ADDR_MAX || n == o)
      psid_status = PSID_BAD_ARGS;
    else
      PSID_Change(o, n);
  } else {
    psid_status = PSID_BAD_ARGS;
  }

  psid_cmd = 0;
}

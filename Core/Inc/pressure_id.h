// file: pressure_id.h
//
// Pressure sensor ID tool: finds or changes the Modbus address of a T2800/
// T3000/T3100 sensor on RS485-U3 (datasheet "通讯协议 V1.3": address lives
// in register 0x0005, range 1..255, factory default 1).
//
// Use from Ozone, with ONLY the sensor being changed wired to RS485-U3 and
// the arm idle - the scan blocks the main loop for up to ~15 s and both
// commands stop the pumps first.
//
//   Scan:    psid_cmd = 2                -> psid_found = its address
//   Change:  psid_old = 1, psid_new = 2,
//            psid_cmd = 1                -> psid_status = PSID_OK
//            (psid_old = 0 if the current address is unknown: sent as a
//             broadcast, which re-addresses EVERY sensor on the bus)
//
// psid_cmd clears itself when the command has run; read psid_status.

#ifndef INC_PRESSURE_ID_H_
#define INC_PRESSURE_ID_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

enum {
  PSID_IDLE        = 0,
  PSID_BUSY        = 1,
  PSID_OK          = 2,  // address written and read back at psid_new
  PSID_NO_ECHO     = 3,  // no/bad reply at psid_old - wrong address or wiring
  PSID_VERIFY_FAIL = 4,  // written, but no reply at psid_new - power-cycle and scan
  PSID_BAD_ARGS    = 5,  // psid_new not 1..247, or equal to psid_old
  PSID_FOUND       = 6,  // scan: psid_found = address
  PSID_NONE_FOUND  = 7   // scan: nothing answered 1..247
};

extern volatile uint8_t psid_cmd;     // 1 = change address, 2 = scan
extern volatile uint8_t psid_old;     // current address, 0 = unknown (broadcast)
extern volatile uint8_t psid_new;     // address to set, 1..247
extern volatile uint8_t psid_status;  // readback, PSID_*
extern volatile uint8_t psid_found;   // readback: scanned / verified address
extern volatile uint8_t psid_scan_at; // readback: address the scan is on

void PressureID_Task(void);  // call once per main loop pass

#ifdef __cplusplus
}
#endif

#endif /* INC_PRESSURE_ID_H_ */

// file: pusirobot.h
//
// Driver for the PUSIROBOT PMC006C3 integrated closed-loop stepper controller
// over CANopen (classic CAN 2.0, expedited SDO).
//   FDCAN1 = PUSIROBOT CANopen (this driver), 125 kbps, standard ids.
//   FDCAN2 = SDM8070JA1 servo (1 Mbps) - different driver, different protocol.
//
// All four motors are PUSIROBOT: nodes 3 and 4 (slider string spools, joint
// rotation and slider separation), nodes 7 and 8 (grasping).
//
// ---- CANopen transport (from the PMC006C3 manual, verified frames) ----
// SDO request  COB-ID = 0x600 + NodeID   (master -> drive)
// SDO response COB-ID = 0x580 + NodeID   (drive  -> master)
// Expedited (single-frame, <=4 byte) transfers only - every numeric object fits.
//   Read  request : 40 [idxL] [idxH] [sub] 00 00 00 00
//   Read  reply   : 4F/4B/47/43 [idxL][idxH][sub] [d0..d3]  (cmd picks length 1/2/3/4)
//   Write request : 2F/2B/27/23 [idxL][idxH][sub] [d0..d3]  (cmd picks length 1/2/3/4)
//   Write reply   : 60 ... = OK ;  80 ... [abort32] = FAIL
//
// The read/write helpers BLOCK (spin-wait with timeout) for the matching
// 0x580+node reply, which is captured in the FDCAN1 RX interrupt via
// PUSI_HandleSdoResp(). At 125 kbps an expedited round-trip is ~1-2 ms; the
// 200 ms main loop absorbs that easily.

#ifndef INC_PUSIROBOT_H_
#define INC_PUSIROBOT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdint.h>

/* ------------------------------------------------------------------ */
/* CANopen object dictionary (subset used here)                        */
/* ------------------------------------------------------------------ */
#define PUSI_OBJ_WORK_MODE     0x6005u  // U8 : 0 = position mode, 1 = speed mode
#define PUSI_OBJ_DIRECTION     0x6002u  // U8 : 1 = forward, 0 = reverse (position mode)
#define PUSI_OBJ_SPEED         0x6003u  // S32: target/run speed [pps]; shared by speed+pos mode
#define PUSI_OBJ_ACCEL         0x6008u  // U16: position accel coeff 0..8 (1 fastest, 8 slowest)
#define PUSI_OBJ_DECEL         0x6009u  // U16: position decel coeff 0..8
#define PUSI_OBJ_MOVE_REL      0x6004u  // U32: relative move, step count (0 = error)
#define PUSI_OBJ_ERROR         0x6000u  // U8 : motor error register (write 0xFF to clear all bits)
#define PUSI_OBJ_STATUS        0x6001u  // U8 : controller status bitfield (see PUSI_ST_*)
#define PUSI_OBJ_STOP          0x6020u  // U8 : write 0 = immediate HARD stop (no ramp)
#define PUSI_OBJ_STALL_PARAM   0x6017u  // U16: open-loop stall threshold (bits0..6, ~3-5)
#define PUSI_OBJ_STALL_STOP    0x601Bu  // U8 : 1 = stop task on stall, 0 = ignore
#define PUSI_OBJ_ENC_POS       0x6035u  // S32 ro: raw encoder position (ABSOLUTE encoders only)
#define PUSI_OBJ_MOTOR_POS     0x600Cu  // S32 rw: motor position (units by mode)
#define PUSI_OBJ_RT_SPEED      0x6030u  // S32 ro: real-time speed from feedback
#define PUSI_OBJ_ENC_RES       0x6021u  // U16: 14 = absolute (16384/rev), 200..8000 = incremental PPR
#define PUSI_OBJ_WIRE_DIR      0x6033u  // U8 : bit0 motor dir, bit1 encoder dir
#define PUSI_OBJ_STALL_LEN     0x6028u  // U16: closed-loop stall length threshold
#define PUSI_OBJ_NODE_ID       0x2002u  // U8 : node id 1..127 (ROM)
#define PUSI_OBJ_BAUD          0x2003u  // U8 : baud index (4=125k default, 8=1M) (ROM)
#define PUSI_OBJ_SAVE          0x2007u  // save command (write 2 = save & power-down persist)
#define PUSI_OBJ_MICROSTEP     0x600Au  // U16: microstep 0/2/4/8/16/32/64/128
#define PUSI_OBJ_RUN_CURRENT   0x600Bu  // U16: run current [mA] (<= 6000)
#define PUSI_OBJ_ENABLE        0x600Eu  // U8 : 1 = ENABLE (energize) motor, 0 = disable

/* Controller status (0x6001) bit helpers */
#define PUSI_ST_ESTOP1(s)      ((s) & 0x01u)
#define PUSI_ST_ESTOP2(s)      (((s) >> 1) & 0x01u)
#define PUSI_ST_STALLED(s)     (((s) >> 2) & 0x01u)   // general-purpose stall flag
#define PUSI_ST_BUSY(s)        (((s) >> 3) & 0x01u)   // read-only, hardware driven
#define PUSI_ST_HOMING(s)      (((s) >> 4) & 0x01u)

#define PUSI_SDO_TIMEOUT_MS    20u      // spin-wait budget for an SDO reply

/* Work-mode / direction constants */
#define PUSI_MODE_POSITION     0u
#define PUSI_MODE_SPEED        1u
/* Rotation sense. Direction actually rides on the SIGN OF THE SPEED (0x6003);
   the direction object 0x6002 alone does not reverse these drives, so both are
   always written consistently.
     -1 = CW  = PUSI_DIR_REVERSE
     +1 = CCW = PUSI_DIR_FORWARD
   That is the sense AT THE DRIVE. Spools mounted mirrored will turn opposite
   ways physically for the same commanded sign - see string_dir_* / joint_dir_*. */
#define PUSI_DIR_FORWARD       1u    /* CCW */
#define PUSI_DIR_REVERSE       0u    /* CW  */
#define PUSI_CW              (-1)
#define PUSI_CCW             (+1)

/* ------------------------------------------------------------------ */
/* Last SDO reply / diagnostics (Ozone-visible)                        */
/* ------------------------------------------------------------------ */
extern volatile uint32_t g_pusi_sdo_abort;   // last SDO abort code (0 = last op OK)
extern volatile uint32_t g_pusi_tx_err;      // TX submit failures
extern volatile uint32_t g_pusi_rx_count;    // SDO replies captured
extern volatile uint32_t g_pusi_timeout_cnt; // SDO reply timeouts

/* ------------------------------------------------------------------ */
/* Setup + transport                                                   */
/* ------------------------------------------------------------------ */
void PUSI_Init(FDCAN_HandleTypeDef *hfdcan);   // store the bus (FDCAN1)

// Called from HAL_FDCAN_RxFifo0Callback for STANDARD-id FDCAN1 frames.
// Captures a 0x580..0x5FF SDO reply for the blocking helpers.
void PUSI_HandleSdoResp(uint32_t id, const uint8_t *data, uint8_t len);

// Expedited SDO read: returns HAL_OK and writes the value (raw, little-endian,
// zero-extended). Caller interprets width/sign per the object type.
HAL_StatusTypeDef PUSI_SdoRead (uint8_t node, uint16_t index, uint8_t sub, int32_t *value_out);
// Expedited SDO write: nbytes = 1..4 (matches the object's declared type).
HAL_StatusTypeDef PUSI_SdoWrite(uint8_t node, uint16_t index, uint8_t sub, uint32_t value, uint8_t nbytes);

/* ------------------------------------------------------------------ */
/* High-level API (the four functions)                                 */
/* ------------------------------------------------------------------ */
// 1) Rotate at a speed: enter speed mode, run continuously at speed_pps
//    (sign = direction; positive = dir 1). Change on the fly, no stop needed.
HAL_StatusTypeDef PUSI_SetSpeed(uint8_t node, int32_t speed_pps);

// Motor enable - energize the driver (0x600E). REQUIRED for continuous running:
// without it the drive spins up then stops. Called inside SetSpeed/MoveRel.
HAL_StatusTypeDef PUSI_EnableMotor  (uint8_t node, uint8_t enable);
HAL_StatusTypeDef PUSI_SetMicrostep (uint8_t node, uint16_t microstep);   // 0x600A
HAL_StatusTypeDef PUSI_SetRunCurrent(uint8_t node, uint16_t current_mA);  // 0x600B

// 2a) Rotate a relative number of STEPS at a set speed (position mode). dir:
//     PUSI_DIR_FORWARD/REVERSE. accel/decel 0..8 (pass 0xFFFF to skip). Requires
//     the drive to be idle (busy bit clear) - returns HAL_BUSY otherwise.
HAL_StatusTypeDef PUSI_MoveRel(uint8_t node, uint32_t steps, uint8_t dir,
                               int32_t speed_pps, uint16_t accel, uint16_t decel);
// 3) Stall detection: read status, test stall bit, clear it, configure thresholds.
HAL_StatusTypeDef PUSI_ReadStatus (uint8_t node, uint8_t *status_out);
HAL_StatusTypeDef PUSI_ClearStall (uint8_t node);              // write bit2 to clear
HAL_StatusTypeDef PUSI_ConfigStall(uint8_t node, uint16_t open_loop_thresh, uint8_t stop_on_stall);

// 4) Stop: immediate hard stop (0x6020 = 0, no deceleration ramp).
HAL_StatusTypeDef PUSI_Stop      (uint8_t node);

/* Feedback reads */
HAL_StatusTypeDef PUSI_ReadSpeed  (uint8_t node, int32_t *spd_out);  // 0x6030

#ifdef __cplusplus
}
#endif

#endif /* INC_PUSIROBOT_H_ */

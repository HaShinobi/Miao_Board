// file: sdm8070.h
//
// Driver for the SDM8070JA1 integrated BLDC servo (IP68, 36:1 gearbox,
// 14-bit absolute encoder, FOC). COMPLETELY different manufacturer and
// protocol from the UIROBOT UIM242s - nothing in uim242.c applies.
//
// Bus: its own FDCAN2 port at 1 Mbps, STANDARD 11-bit CAN ids (the UIM242 bus
// on FDCAN1 stays at 800 kbps with 29-bit extended ids and is untouched).
//
// ---- Protocol summary (from the manufacturer's CAN spec) ----
// Motor ID 1..7 set by DIP switches 1-3 (0 = invalid). DIP 4 = 120R term.
//
// COMMAND frames (host -> motor). These are GROUP frames: one frame carries
// several motors' setpoints, so unused slots MUST be filled with the
// "invalid" marker or those motors would be commanded too:
//   position : 0x200 (motors 1,2)  0x201 (3,4)  0x204 (5,6)  0x205 (7)
//              int32 BIG-ENDIAN, unit 0.01 deg, at the GEARBOX OUTPUT,
//              max 500000 deg. Invalid marker = 0x7FFFFFFF.
//   speed    : 0x202 (motors 1..4) 0x203 (5..7)
//              int16 BIG-ENDIAN, unit RPM at the gearbox output,
//              range -2048..2047. Invalid marker = 0xFFFF.
//   torque   : 0x206 (1..4) 0x207 (5..7)  int16 BE, unit = value*10mA at the
//              MOTOR shaft. Invalid marker = 0x7FFF. (not wrapped here)
//   config   : 0x300, one byte per motor (DATA0 = motor 1 ... DATA6 = motor 7)
//              0x00 activate, 0x01 disable, 0x02 angle calibration,
//              0x03 calibration auto fine-tune, 0x04 set current pos = 0,
//              0x05 emergency brake, 0x06 factory reset, 0x07 software reset,
//              0x08 fault reset, 0xFF = no action for that motor.
//   params   : 0x301, DATA0 = motor id, DATA1 = command id,
//              DATA2/3 = value (BE16), DATA6 = save flag (1 = persist).
//              cmd 0x00 stall protect (current 10mA units / time ms),
//              cmd 0x01 speed-mode acceleration (max 4000, default 4000),
//              cmd 0x02 position-mode max speed (default 5000),
//              cmd 0x04 gear ratio (value/100, default 36).
//
// FEEDBACK frames (motor -> host), broadcast at 50 Hz:
//   0x206 + 2*id : DATA0-3 position int32 BE (0.01 deg, +/-18000 = +/-180),
//                  DATA4-5 speed int16 BE (rpm), DATA6-7 current int16 BE.
//   0x207 + 2*id : DATA0 supply voltage (V), DATA1 driver temp (C, signed),
//                  DATA2 status byte, DATA3 motor temp (C, signed).
//
// Status byte bits:
//   bit7 driver activated      bit6 motor calibrated
//   bit5 STALLED (latched - needs config 0x08 fault reset to clear)
//   bit4..3 mode: 0 open-loop voltage, 1 closed-loop speed,
//                 2 closed-loop position, 3 closed-loop torque
//   bit2..0 fault: 0 none, 1 current fb, 2 encoder fb, 3 power device,
//                  4 over-temp, 5 under-volt, 6 over-volt, 7 over-current

#ifndef INC_SDM8070_H_
#define INC_SDM8070_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdint.h>

#define SDM_ID_MIN            1u
#define SDM_ID_MAX            7u
#define SDM_MOTOR_COUNT       7u

/* Invalid markers: written into the slots of motors we are NOT commanding, so
   a group frame only affects the intended motor. */
#define SDM_POS_INVALID       0x7FFFFFFF
#define SDM_SPD_INVALID       0xFFFF

/* Frame ids */
#define SDM_FRAME_CONFIG      0x300u
#define SDM_FRAME_PARAM       0x301u

/* Config command bytes (frame 0x300) */
#define SDM_CFG_ACTIVATE      0x00u
#define SDM_CFG_DISABLE       0x01u
#define SDM_CFG_CALIBRATE     0x02u
#define SDM_CFG_CAL_TUNE      0x03u
#define SDM_CFG_ZERO_POS      0x04u   /* current position becomes 0 (not saved) */
#define SDM_CFG_EBRAKE        0x05u
#define SDM_CFG_FACTORY_RST   0x06u
#define SDM_CFG_SOFT_RESET    0x07u
#define SDM_CFG_FAULT_RESET   0x08u   /* required to clear a latched stall */
#define SDM_CFG_NONE          0xFFu

/* Param command ids (frame 0x301) */
#define SDM_PARAM_STALL       0x00u
#define SDM_PARAM_SPD_ACCEL   0x01u
#define SDM_PARAM_POS_MAXSPD  0x02u
#define SDM_PARAM_GEAR_RATIO  0x04u

/* Status byte helpers */
#define SDM_ST_ACTIVATED(s)   (((s) >> 7) & 0x01u)
#define SDM_ST_CALIBRATED(s)  (((s) >> 6) & 0x01u)
#define SDM_ST_STALLED(s)     (((s) >> 5) & 0x01u)
#define SDM_ST_MODE(s)        (((s) >> 3) & 0x03u)   /* 0 volt,1 spd,2 pos,3 trq */
#define SDM_ST_FAULT(s)       ((s) & 0x07u)

/* ---- Per-motor feedback (index 0 = motor id 1) ---- */
typedef struct {
    uint8_t  id;
    int32_t  position_raw;    // 0.01 deg units, +/-18000
    float    position_deg;    // position_raw / 100
    int16_t  speed_rpm;       // gearbox output rpm
    int16_t  current_raw;     // 12-bit ADC units
    uint8_t  voltage_v;
    int8_t   drv_temp_c;
    int8_t   motor_temp_c;
    uint8_t  status;          // raw status byte (see bit helpers)
    uint32_t last_rx_tick;
} SDM_Motor_t;

extern SDM_Motor_t g_sdm_motors[SDM_MOTOR_COUNT];

/* Bus sniffer: every frame seen on FDCAN2, readable in Ozone. */
#define SDM_RX_LOG_LEN 32
typedef struct {
    uint32_t id;
    uint8_t  data[8];
    uint8_t  len;
    uint32_t tick;
} SDM_RxLog_t;

extern volatile SDM_RxLog_t g_sdm_rx_log[SDM_RX_LOG_LEN];
extern volatile uint8_t     g_sdm_rx_log_idx;
extern volatile uint32_t    g_sdm_rx_count;
extern volatile uint32_t    g_sdm_tx_drop_count;

// The datasheet contradicts itself on the POSITION frame id: section 2.3 shows
// 0x200 (motors 1,2) but the summary table in 1.2 lists position control as
// 0x204/0x205 and gives 0x200/0x201 to VOLTAGE control. Set this non-zero to
// override the frame id used by SDM_SetPositionDeg so both can be tried live
// from Ozone (0 = use the built-in mapping).
extern volatile uint32_t    g_sdm_pos_frame_override;
extern volatile uint32_t    g_sdm_last_rx_id;
extern volatile uint8_t     g_sdm_last_rx_data[8];
extern volatile uint8_t     g_sdm_last_rx_len;

/* ---- API ---- */

// Move motor to an ABSOLUTE angle (degrees, gearbox output). Other motors in
// the same group frame are marked invalid so they are unaffected.
HAL_StatusTypeDef SDM_SetPositionDeg(FDCAN_HandleTypeDef *hfdcan, uint8_t id, float degrees);

// Closed-loop speed, RPM at the gearbox output (-2048..2047).
HAL_StatusTypeDef SDM_SetSpeedRPM(FDCAN_HandleTypeDef *hfdcan, uint8_t id, int16_t rpm);

// Max travel speed used by position moves (RPM). save=1 persists it.
HAL_StatusTypeDef SDM_SetPositionMaxSpeed(FDCAN_HandleTypeDef *hfdcan, uint8_t id,
                                          uint16_t rpm, uint8_t save);

// Generic parameter write (frame 0x301).
HAL_StatusTypeDef SDM_SetParam(FDCAN_HandleTypeDef *hfdcan, uint8_t id,
                               uint8_t cmd_id, uint16_t value, uint8_t save);

// Stall protection: current in 10mA units, time in ms (0..60000).
// Default at power-up is 30 A / 2000 ms. Lowering the current makes the drive
// declare a stall gently - which is what makes hard-stop homing safe on a
// motor that can otherwise push 35 Nm into an end stop.
HAL_StatusTypeDef SDM_SetStallProtect(FDCAN_HandleTypeDef *hfdcan, uint8_t id,
                                      uint16_t current_10mA, uint16_t time_ms,
                                      uint8_t save);

// Config command (frame 0x300): activate / disable / zero / fault reset / ...
HAL_StatusTypeDef SDM_Config(FDCAN_HandleTypeDef *hfdcan, uint8_t id, uint8_t cfg);

// Convenience wrappers
HAL_StatusTypeDef SDM_Activate(FDCAN_HandleTypeDef *hfdcan, uint8_t id);
HAL_StatusTypeDef SDM_Disable(FDCAN_HandleTypeDef *hfdcan, uint8_t id);
HAL_StatusTypeDef SDM_ZeroPosition(FDCAN_HandleTypeDef *hfdcan, uint8_t id);
HAL_StatusTypeDef SDM_FaultReset(FDCAN_HandleTypeDef *hfdcan, uint8_t id);

// Cached feedback for a motor id (NULL if id out of range).
SDM_Motor_t *SDM_GetMotor(uint8_t id);

// Called from HAL_FDCAN_RxFifo0Callback for frames arriving on FDCAN2.
void SDM_HandleRxFrame(uint32_t id, const uint8_t *data, uint8_t len);

/* ---- Hard-stop homing (blocking) ----
   Tuning/config for SDM_Home(). All the knobs live here so the caller can fill
   this from its own Ozone-visible globals. */
typedef struct {
    int16_t  home_rpm;        // seek speed [rpm]; sign sets direction (+ = into stop)
    uint16_t home_cur_10ma;   // gentle stall current WHILE seeking (10mA units)
    uint16_t home_time_ms;    // stall-trip time while seeking
    uint16_t run_cur_10ma;    // normal stall current restored after homing
    uint16_t run_time_ms;     // normal stall time restored after homing
    float    min_deg;         // deg/loop below which the axis counts as "not moving"
    uint8_t  stuck_n;         // consecutive "not moving" loops = stop found
    uint8_t  grace_n;         // spin-up grace loops before the stop detector arms
    uint16_t timeout;         // max loops (~200 ms each) before giving up
    float    backoff_deg;     // back off this far (CCW) after hitting the stop
    uint8_t  skip_backoff;    // 1 = go straight from the stop to done (no back-off)
    uint16_t maxspeed_rpm;    // position-move max speed used for the back-off
} SDM_HomeCfg_t;

// Blocking hard-stop home: seeks slowly into the mechanical stop, detects it by
// the angle no longer advancing (or the drive's stall bit), zeros the position
// there, restores the normal stall current, then optionally backs off.
// Returns 1 = homed OK, 0 = timed out. Takes a few seconds; the 50 Hz feedback
// keeps updating via the RX interrupt during the internal HAL_Delay waits.
uint8_t SDM_Home(FDCAN_HandleTypeDef *hfdcan, uint8_t id, const SDM_HomeCfg_t *cfg);

// Arm servo "move on change": issues a HELD absolute-angle move to target_deg,
// but only when target_deg differs from the previous call - so the caller just
// updates the target and the move fires automatically (no apply flag). Keeps
// its own last-target memory; call it every loop from the arm's servo state.
// The closed-loop travel/hold is carried out by SDM_ServoTask via the sdm_*
// command globals. speed_rpm = travel speed, tol_deg = hold deadband.
void SDM_MoveOnChange(float target_deg, int16_t speed_rpm, float tol_deg);

/* ==================================================================== */
/*  SERVO HARNESS (runs every loop, in every program_state)             */
/*  Position moves are ABSOLUTE degrees at the gearbox output.          */
/* ==================================================================== */
extern volatile uint8_t  sdm_id;             // DIP-switch motor id (1..7)
extern volatile float    sdm_target_deg;     // absolute target [deg], user frame
extern volatile uint16_t sdm_speed_rpm;      // travel speed [rpm]
extern volatile uint8_t  sdm_move_apply;     // 1 = start the move (auto-clears)

/* The drive has a comms watchdog: a one-shot command moves it briefly and
   stops. While sdm_move_run is 1 the target is re-sent every loop. Cleared
   on arrival unless sdm_move_hold is set. */
extern volatile uint8_t  sdm_move_run;
extern volatile float    sdm_move_tol_deg;   // "arrived" window [deg]
extern volatile uint8_t  sdm_move_hold;      // 1 = stay engaged and correct drift
extern volatile float    sdm_move_reengage_deg; // re-engage past THIS; kills wobble
extern volatile float    sdm_move_correction_deg; // moves land ~3 deg short; aim past
extern volatile uint8_t  sdm_send_maxspeed;  // 0 = skip the position-mode speed write

/* Firmware position control. The drive's native position frame (0x200) never
   took - sdm_mode never leaves 0 - so the loop is closed here using only what
   is proven on this motor: speed commands plus the 50 Hz angle feedback.
   1 = firmware (speed-based), 0 = native 0x200 frame. */
extern volatile uint8_t  sdm_softpos_enable;
extern volatile float    sdm_softpos_kp;     // rpm per degree of error
extern volatile float    sdm_softpos_kd;     // rpm per (deg/loop); damps hunting
extern volatile int16_t  sdm_softpos_min_rpm;// floor so it still creeps in
extern volatile int16_t  sdm_softpos_rpm;    // READ-ONLY: speed being commanded

extern volatile int16_t  sdm_jog_rpm;        // signed speed setpoint [rpm]
extern volatile uint8_t  sdm_jog_apply;      // 1 = start jogging (auto-clears)
extern volatile uint8_t  sdm_jog_run;        // 1 = keep re-sending sdm_jog_rpm
extern volatile uint8_t  sdm_cfg_apply;      // 1 activate, 2 disable, 3 zero, 4 fault reset

/* read-only mirrors of the 50 Hz feedback */
extern volatile float    sdm_pos_deg;        // as the DRIVE reports it [deg]
extern volatile float    sdm_pos_user_deg;   // drive * sdm_dir + offset [deg]
extern volatile int32_t  sdm_pos_raw;        // [0.01 deg units]
extern volatile int16_t  sdm_speed_fb_rpm;
extern volatile uint8_t  sdm_status;         // see SDM_ST_* helpers
extern volatile uint8_t  sdm_mode;           // 0 volt, 1 speed, 2 pos, 3 torque
extern volatile uint8_t  sdm_stalled;
extern volatile uint8_t  sdm_fault;
extern volatile uint8_t  sdm_voltage_v;
extern volatile int8_t   sdm_drv_temp_c;

/* ---- Hard-stop homing ----
   The drive zeroes wherever it sits at power-up and config 0x04 is not saved
   through power loss, so "0 deg" has no physical meaning until we establish
   it. Homing creeps into a mechanical stop and calls that point zero. */
extern volatile uint8_t  sdm_home_apply;     // 1 = start homing (auto-clears)
extern volatile uint8_t  sdm_home_state;     // 0 idle,1 setup,2 seek,3 backoff,4 DONE,5 TIMEOUT
extern volatile int16_t  sdm_home_rpm;       // POSITIVE = clockwise, into the stop

/* Rotation polarity in YOUR frame:
     user_deg  = sdm_dir * drive_deg + sdm_zero_offset_deg
     drive_cmd = sdm_dir * (user_target - sdm_zero_offset_deg)

   THESE TWO MUST FLIP TOGETHER. Homing drives positive into the stop and
   zeroes there, so at the stop drive = 0 and user = offset. The post-home
   move to user 0 must command a NEGATIVE drive angle (away from the stop):

     dir +1, offset +92 :  drive = +1*(0 - 92) = -92   away from stop  OK
     dir -1, offset -92 :  drive = -1*(0 + 92) = -92   away from stop  OK
     dir -1, offset +92 :  drive = -1*(0 - 92) = +92   INTO the stop   BAD

   That last combination is what broke arm_state 1 homing. */
extern volatile int8_t   sdm_dir;
extern volatile float    sdm_zero_offset_deg;   // angle assigned to the hard stop
extern volatile float    sdm_home_backoff_deg;  // back off this far (CCW) after hitting it

/* Stall-trip current used WHILE homing. Tripping this LATCHES and cuts the
   output until a fault reset, so it must sit ABOVE the current needed to
   drive the load - otherwise the motor stops mid-travel and the angle
   detector mistakes that for the stop. The stop is found by angle, not by
   this trip; safety comes from the slow seek speed. 750 = 7.5 A. */
extern volatile uint16_t sdm_home_cur_10ma;
extern volatile uint16_t sdm_home_time_ms;
extern volatile uint16_t sdm_run_cur_10ma;   // restored after homing (3000 = 30 A)
extern volatile uint16_t sdm_run_time_ms;
extern volatile uint16_t sdm_home_timeout;   // loops (~200 ms each) before giving up
extern volatile uint8_t  sdm_home_skip_backoff; // 1 = go straight from stop to DONE

/* Firmware stop-detection: the drive's stall bit trips at 30 A, which a slow
   creep may never reach. Instead, if we are commanding motion but the angle
   stops changing, we have arrived. At 2 rpm a 200 ms loop moves ~2.4 deg. */
extern volatile float    sdm_home_min_deg;   // min deg/loop that counts as moving
extern volatile uint8_t  sdm_home_stuck_n;   // consecutive stuck loops = stop found
extern volatile float    sdm_home_moved_deg; // READ-ONLY: deg moved last loop
extern volatile uint8_t  sdm_home_grace_n;   // ignore the detector during spin-up

void SDM_DoHome(void);      // blocking home using the sdm_home_* settings
void SDM_ServoTask(void);   // run once per loop, in every program_state

#ifdef __cplusplus
}
#endif

#endif /* INC_SDM8070_H_ */

// file: sdm8070.c
// SDM8070JA1 servo driver on FDCAN2 (1 Mbps, standard 11-bit ids).
// See sdm8070.h for the protocol summary.

#include "main.h"          /* hfdcan2 */
#include "sdm8070.h"
#include <string.h>

/* ---- servo harness state (see sdm8070.h) ---- */
volatile uint8_t  sdm_id           = 1;
volatile float    sdm_target_deg   = 0.0f;
volatile uint16_t sdm_speed_rpm    = 30;
volatile uint8_t  sdm_move_apply   = 0;
volatile uint8_t  sdm_move_run     = 0;
volatile float    sdm_move_tol_deg = 0.3f;
volatile uint8_t  sdm_move_hold    = 0;
volatile float    sdm_move_reengage_deg   = 4.0f;
volatile float    sdm_move_correction_deg = 3.0f;
volatile uint8_t  sdm_send_maxspeed = 1;

volatile uint8_t  sdm_softpos_enable  = 1;
volatile float    sdm_softpos_kp      = 1.0f;
volatile float    sdm_softpos_kd      = 1.0f;
volatile int16_t  sdm_softpos_min_rpm = 1;
volatile int16_t  sdm_softpos_rpm     = 0;

volatile int16_t  sdm_jog_rpm      = 0;
volatile uint8_t  sdm_jog_apply    = 0;
volatile uint8_t  sdm_jog_run      = 0;
volatile uint8_t  sdm_cfg_apply    = 0;

volatile float    sdm_pos_deg      = 0.0f;
volatile float    sdm_pos_user_deg = 0.0f;
volatile int32_t  sdm_pos_raw      = 0;
volatile int16_t  sdm_speed_fb_rpm = 0;
volatile uint8_t  sdm_status       = 0;
volatile uint8_t  sdm_mode         = 0;
volatile uint8_t  sdm_stalled      = 0;
volatile uint8_t  sdm_fault        = 0;
volatile uint8_t  sdm_voltage_v    = 0;
volatile int8_t   sdm_drv_temp_c   = 0;

volatile uint8_t  sdm_home_apply   = 0;
volatile uint8_t  sdm_home_state   = 0;
volatile int16_t  sdm_home_rpm     = +2;
volatile int8_t   sdm_dir          = +1;
volatile float    sdm_zero_offset_deg  = 92.0f;
volatile float    sdm_home_backoff_deg = 3.0f;
volatile uint16_t sdm_home_cur_10ma = 750;
volatile uint16_t sdm_home_time_ms  = 300;
volatile uint16_t sdm_run_cur_10ma  = 3000;
volatile uint16_t sdm_run_time_ms   = 2000;
volatile uint16_t sdm_home_timeout  = 100;
volatile uint8_t  sdm_home_skip_backoff = 0;
volatile float    sdm_home_min_deg   = 0.05f;
volatile uint8_t  sdm_home_stuck_n   = 5;
volatile float    sdm_home_moved_deg = 0.0f;
volatile uint8_t  sdm_home_grace_n   = 10;

SDM_Motor_t g_sdm_motors[SDM_MOTOR_COUNT];

volatile SDM_RxLog_t g_sdm_rx_log[SDM_RX_LOG_LEN];
volatile uint8_t     g_sdm_rx_log_idx = 0;
volatile uint32_t    g_sdm_rx_count   = 0;
volatile uint32_t    g_sdm_tx_drop_count = 0;
volatile uint32_t    g_sdm_pos_frame_override = 0;   /* 0 = use built-in mapping */
volatile uint32_t    g_sdm_last_rx_id = 0;
volatile uint8_t     g_sdm_last_rx_data[8] = {0};
volatile uint8_t     g_sdm_last_rx_len = 0;

/* ------------------------------------------------------------------ */
/* Internal: send one standard-id frame                                */
/* ------------------------------------------------------------------ */
static HAL_StatusTypeDef sdm_tx(FDCAN_HandleTypeDef *hfdcan, uint32_t std_id,
                                const uint8_t *data8)
{
    FDCAN_TxHeaderTypeDef TxHeader;

    TxHeader.Identifier          = std_id;
    TxHeader.IdType              = FDCAN_STANDARD_ID;   /* 11-bit, not extended */
    TxHeader.TxFrameType         = FDCAN_DATA_FRAME;
    TxHeader.DataLength          = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch       = FDCAN_BRS_OFF;
    TxHeader.FDFormat            = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker       = 0;

    /* Wait briefly for a free TX FIFO slot rather than dropping silently. */
    uint32_t t0 = HAL_GetTick();
    while (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0U) {
        if ((HAL_GetTick() - t0) > 5U) {
            g_sdm_tx_drop_count++;
            return HAL_TIMEOUT;
        }
    }

    HAL_StatusTypeDef st = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, (uint8_t *)data8);
    if (st != HAL_OK) g_sdm_tx_drop_count++;
    return st;
}

/* Big-endian packers (this protocol is BE, unlike the UIM242's LE) */
static inline void be32(uint8_t *p, int32_t v)
{
    p[0] = (uint8_t)((uint32_t)v >> 24);
    p[1] = (uint8_t)((uint32_t)v >> 16);
    p[2] = (uint8_t)((uint32_t)v >> 8);
    p[3] = (uint8_t)((uint32_t)v);
}
static inline void be16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)(v);
}
static inline int32_t rd_be32(const uint8_t *p)
{
    return (int32_t)(((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
                     ((uint32_t)p[2] << 8)  |  (uint32_t)p[3]);
}
static inline int16_t rd_be16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

/* ------------------------------------------------------------------ */
/* Position command (frame 0x200/0x201/0x204/0x205)                    */
/*                                                                     */
/* Group frame: two motors per frame. The slot for the motor we are NOT */
/* addressing is filled with SDM_POS_INVALID so it is left alone -      */
/* zero-filling would command that motor to 0 degrees.                 */
/* ------------------------------------------------------------------ */
HAL_StatusTypeDef SDM_SetPositionDeg(FDCAN_HandleTypeDef *hfdcan, uint8_t id, float degrees)
{
    if (id < SDM_ID_MIN || id > SDM_ID_MAX) return HAL_ERROR;

    /* degrees -> 0.01 deg units, rounded */
    float centi_f = degrees * 100.0f;
    if (centi_f >  50000000.0f) centi_f =  50000000.0f;   /* spec max 500000 deg */
    if (centi_f < -50000000.0f) centi_f = -50000000.0f;
    int32_t centi = (int32_t)((centi_f >= 0.0f) ? (centi_f + 0.5f) : (centi_f - 0.5f));

    uint32_t frame;
    uint8_t  slot;              /* 0 = DATA0-3, 1 = DATA4-7 */
    switch (id) {
        case 1: frame = 0x200; slot = 0; break;
        case 2: frame = 0x200; slot = 1; break;
        case 3: frame = 0x201; slot = 0; break;
        case 4: frame = 0x201; slot = 1; break;
        case 5: frame = 0x204; slot = 0; break;
        case 6: frame = 0x204; slot = 1; break;
        case 7: frame = 0x205; slot = 0; break;
        default: return HAL_ERROR;
    }

    /* Live override for resolving the 0x200-vs-0x204 ambiguity on the bench */
    if (g_sdm_pos_frame_override != 0u) frame = g_sdm_pos_frame_override;

    uint8_t d[8];
    be32(&d[0], SDM_POS_INVALID);
    be32(&d[4], SDM_POS_INVALID);
    be32(&d[slot ? 4 : 0], centi);

    return sdm_tx(hfdcan, frame, d);
}

/* ------------------------------------------------------------------ */
/* Speed command (frame 0x202 motors 1-4, 0x203 motors 5-7)            */
/* int16 BE, RPM at the gearbox output. Unused slots = SDM_SPD_INVALID. */
/* ------------------------------------------------------------------ */
HAL_StatusTypeDef SDM_SetSpeedRPM(FDCAN_HandleTypeDef *hfdcan, uint8_t id, int16_t rpm)
{
    if (id < SDM_ID_MIN || id > SDM_ID_MAX) return HAL_ERROR;
    if (rpm >  2047) rpm =  2047;
    if (rpm < -2048) rpm = -2048;

    uint32_t frame;
    uint8_t  slot;              /* byte offset = slot * 2 */
    if (id <= 4) { frame = 0x202; slot = (uint8_t)(id - 1); }
    else         { frame = 0x203; slot = (uint8_t)(id - 5); }

    uint8_t d[8];
    for (uint8_t i = 0; i < 4; i++) be16(&d[i * 2], SDM_SPD_INVALID);
    be16(&d[slot * 2], (uint16_t)rpm);

    return sdm_tx(hfdcan, frame, d);
}

/* ------------------------------------------------------------------ */
/* Config command (frame 0x300): one byte per motor, 0xFF = no action  */
/* ------------------------------------------------------------------ */
HAL_StatusTypeDef SDM_Config(FDCAN_HandleTypeDef *hfdcan, uint8_t id, uint8_t cfg)
{
    if (id < SDM_ID_MIN || id > SDM_ID_MAX) return HAL_ERROR;

    uint8_t d[8];
    for (uint8_t i = 0; i < 8; i++) d[i] = SDM_CFG_NONE;   /* leave others alone */
    d[id - 1] = cfg;

    return sdm_tx(hfdcan, SDM_FRAME_CONFIG, d);
}

HAL_StatusTypeDef SDM_Activate(FDCAN_HandleTypeDef *h, uint8_t id)
{ return SDM_Config(h, id, SDM_CFG_ACTIVATE); }

HAL_StatusTypeDef SDM_Disable(FDCAN_HandleTypeDef *h, uint8_t id)
{ return SDM_Config(h, id, SDM_CFG_DISABLE); }

HAL_StatusTypeDef SDM_ZeroPosition(FDCAN_HandleTypeDef *h, uint8_t id)
{ return SDM_Config(h, id, SDM_CFG_ZERO_POS); }

HAL_StatusTypeDef SDM_FaultReset(FDCAN_HandleTypeDef *h, uint8_t id)
{ return SDM_Config(h, id, SDM_CFG_FAULT_RESET); }

/* ------------------------------------------------------------------ */
/* Parameter write (frame 0x301) - per-motor, not a group frame        */
/* ------------------------------------------------------------------ */
HAL_StatusTypeDef SDM_SetParam(FDCAN_HandleTypeDef *hfdcan, uint8_t id,
                               uint8_t cmd_id, uint16_t value, uint8_t save)
{
    if (id < SDM_ID_MIN || id > SDM_ID_MAX) return HAL_ERROR;

    uint8_t d[8] = {0};
    d[0] = id;
    d[1] = cmd_id;
    be16(&d[2], value);
    d[6] = save ? 1u : 0u;      /* 1 = persist through power cycle */

    return sdm_tx(hfdcan, SDM_FRAME_PARAM, d);
}

/* Stall protection (frame 0x301, cmd 0x00): DATA2/3 = current (10mA units),
   DATA4/5 = time (ms), DATA6 = save flag. Needs its own packer because it
   carries two 16-bit values, unlike the generic single-value param write. */
HAL_StatusTypeDef SDM_SetStallProtect(FDCAN_HandleTypeDef *hfdcan, uint8_t id,
                                      uint16_t current_10mA, uint16_t time_ms,
                                      uint8_t save)
{
    if (id < SDM_ID_MIN || id > SDM_ID_MAX) return HAL_ERROR;
    if (time_ms > 60000u) time_ms = 60000u;

    uint8_t d[8] = {0};
    d[0] = id;
    d[1] = SDM_PARAM_STALL;
    be16(&d[2], current_10mA);
    be16(&d[4], time_ms);
    d[6] = save ? 1u : 0u;

    return sdm_tx(hfdcan, SDM_FRAME_PARAM, d);
}

HAL_StatusTypeDef SDM_SetPositionMaxSpeed(FDCAN_HandleTypeDef *hfdcan, uint8_t id,
                                          uint16_t rpm, uint8_t save)
{
    /* Position-mode travel speed limit. Datasheet default 5000; note the doc
       is ambiguous between the "max speed" and "ramp time" forms of cmd 0x02,
       so verify the effect on the bench before trusting a specific number. */
    return SDM_SetParam(hfdcan, id, SDM_PARAM_POS_MAXSPD, rpm, save);
}

/* ------------------------------------------------------------------ */
/* Feedback state access                                               */
/* ------------------------------------------------------------------ */
SDM_Motor_t *SDM_GetMotor(uint8_t id)
{
    if (id < SDM_ID_MIN || id > SDM_ID_MAX) return NULL;
    SDM_Motor_t *m = &g_sdm_motors[id - 1];
    m->id = id;
    return m;
}

/* ------------------------------------------------------------------ */
/* RX: decode the 50 Hz feedback broadcast                             */
/*   0x206 + 2*id : position / speed / current                         */
/*   0x207 + 2*id : voltage / driver temp / status / motor temp        */
/* ------------------------------------------------------------------ */
void SDM_HandleRxFrame(uint32_t id, const uint8_t *data, uint8_t len)
{
    if (len > 8) len = 8;

    g_sdm_rx_count++;
    g_sdm_last_rx_id  = id;
    g_sdm_last_rx_len = len;
    for (uint8_t i = 0; i < 8; i++)
        g_sdm_last_rx_data[i] = (i < len && data) ? data[i] : 0;

    /* Sniffer ring (oldest overwritten first) */
    volatile SDM_RxLog_t *e = &g_sdm_rx_log[g_sdm_rx_log_idx];
    e->id   = id;
    e->len  = len;
    e->tick = HAL_GetTick();
    for (uint8_t i = 0; i < 8; i++)
        e->data[i] = (i < len && data) ? data[i] : 0;
    g_sdm_rx_log_idx = (uint8_t)((g_sdm_rx_log_idx + 1u) % SDM_RX_LOG_LEN);

    if (!data) return;

    /* Motion feedback: id = 0x206 + 2*motor_id  (0x208,0x20A,...,0x214) */
    if (id >= 0x208u && id <= 0x214u && ((id - 0x206u) % 2u) == 0u) {
        uint8_t mid = (uint8_t)((id - 0x206u) / 2u);
        SDM_Motor_t *m = SDM_GetMotor(mid);
        if (m && len >= 8) {
            m->position_raw  = rd_be32(&data[0]);          /* 0.01 deg */
            m->position_deg  = (float)m->position_raw / 100.0f;
            m->speed_rpm     = rd_be16(&data[4]);
            m->current_raw   = rd_be16(&data[6]);
            m->last_rx_tick  = HAL_GetTick();
        }
        return;
    }

    /* Status feedback: id = 0x207 + 2*motor_id (0x209,0x20B,...,0x215) */
    if (id >= 0x209u && id <= 0x215u && ((id - 0x207u) % 2u) == 0u) {
        uint8_t mid = (uint8_t)((id - 0x207u) / 2u);
        SDM_Motor_t *m = SDM_GetMotor(mid);
        if (m && len >= 3) {
            m->voltage_v    = data[0];
            m->drv_temp_c   = (int8_t)data[1];
            m->status       = data[2];
            if (len >= 4) m->motor_temp_c = (int8_t)data[3];
            m->last_rx_tick = HAL_GetTick();
        }
        return;
    }
}

/* ------------------------------------------------------------------ */
/* Hard-stop homing (blocking)                                         */
/* Was the state machine inside main.c's SDM_ServoTask; moved here so   */
/* the caller can just invoke it. Blocking: it runs the whole           */
/* seek/zero/back-off sequence with its own HAL_Delay waits (the 50 Hz  */
/* feedback still updates via the RX interrupt).                        */
/* ------------------------------------------------------------------ */
uint8_t SDM_Home(FDCAN_HandleTypeDef *hfdcan, uint8_t id, const SDM_HomeCfg_t *cfg)
{
    if (cfg == NULL) return 0;
    SDM_Motor_t *sm = SDM_GetMotor(id);

    /* --- Setup: clear stall, lower trip current, activate, start seeking. --- */
    SDM_FaultReset(hfdcan, id);                                          HAL_Delay(3);
    SDM_SetStallProtect(hfdcan, id, cfg->home_cur_10ma, cfg->home_time_ms, 0);
    HAL_Delay(3);
    SDM_Activate(hfdcan, id);                                            HAL_Delay(3);
    SDM_SetSpeedRPM(hfdcan, id, cfg->home_rpm);

    uint16_t loops    = 0;
    uint8_t  stuck    = 0;
    uint8_t  grace    = cfg->grace_n;
    float    last_deg = (sm != NULL) ? sm->position_deg : 0.0f;

    /* --- Seek: stop found when the drive stalls OR the angle stops advancing.
       One step per 200 ms so the 50 Hz feedback refreshes between checks. --- */
    for (;;) {
        HAL_Delay(200);
        uint8_t stalled = (sm != NULL) ? (uint8_t)SDM_ST_STALLED(sm->status) : 0u;
        if (sm != NULL) {
            float moved = sm->position_deg - last_deg;
            if (moved < 0.0f) moved = -moved;
            last_deg = sm->position_deg;
            sdm_home_moved_deg = moved;   /* Ozone readout: set sdm_home_min_deg
                                             well BELOW what this shows while
                                             the motor is actually turning */
            if (grace > 0) { grace--; stuck = 0; stalled = 0; }   /* spin-up grace */
            else if (moved < cfg->min_deg) { if (++stuck >= cfg->stuck_n) stalled = 1; }
            else stuck = 0;
        }
        if (stalled) break;                                       /* hit the stop */
        if (++loops > cfg->timeout) {                             /* gave up */
            SDM_SetSpeedRPM(hfdcan, id, 0);
            SDM_SetStallProtect(hfdcan, id, cfg->run_cur_10ma, cfg->run_time_ms, 0);
            return 0;
        }
        SDM_SetSpeedRPM(hfdcan, id, cfg->home_rpm);               /* refresh (watchdog) */
    }

    /* --- At the stop: stop, zero here, clear stall, re-activate, restore torque. --- */
    SDM_SetSpeedRPM(hfdcan, id, 0);                                      HAL_Delay(5);
    SDM_ZeroPosition(hfdcan, id);                                        HAL_Delay(5);
    SDM_FaultReset(hfdcan, id);                                          HAL_Delay(5);
    SDM_Activate(hfdcan, id);                                            HAL_Delay(5);
    SDM_SetStallProtect(hfdcan, id, cfg->run_cur_10ma, cfg->run_time_ms, 0);
    HAL_Delay(5);

    /* --- Optional back-off from the stop. --- */
    if (!cfg->skip_backoff && cfg->backoff_deg > 0.0f) {
        SDM_SetPositionMaxSpeed(hfdcan, id, cfg->maxspeed_rpm, 0);       HAL_Delay(3);
        uint16_t bl = 0;
        for (;;) {
            SDM_SetPositionDeg(hfdcan, id, -cfg->backoff_deg);
            HAL_Delay(200);
            float err = (sm != NULL) ? (sm->position_deg + cfg->backoff_deg) : 0.0f;
            if (err < 0.0f) err = -err;
            if (err < 0.5f || ++bl > cfg->timeout) break;
        }
    }
    return 1;
}

/* ------------------------------------------------------------------ */
/* Arm servo auto-move. Only the move COMMAND lives here; the closed-   */
/* loop travel and hold run in SDM_ServoTask below.                     */
/* ------------------------------------------------------------------ */
void SDM_MoveOnChange(float target_deg, int16_t speed_rpm, float tol_deg)
{
    static float last_target = 3.4e38f;   /* sentinel: forces the first move */
    if (target_deg != last_target) {
        sdm_speed_rpm    = (uint16_t)speed_rpm;
        sdm_move_tol_deg = tol_deg;
        sdm_target_deg   = target_deg;    /* absolute angle (user frame) */
        sdm_move_hold    = 1;             /* HOLD at the target */
        sdm_move_apply   = 1;             /* SDM_ServoTask executes/holds it */
        last_target      = target_deg;
    }
}

/* ------------------------------------------------------------------ */
/* Blocking hard-stop home, using the sdm_home_* settings.              */
/* ------------------------------------------------------------------ */
void SDM_DoHome(void)
{
  SDM_HomeCfg_t cfg;
  cfg.home_rpm      = sdm_home_rpm;
  cfg.home_cur_10ma = sdm_home_cur_10ma;
  cfg.home_time_ms  = sdm_home_time_ms;
  cfg.run_cur_10ma  = sdm_run_cur_10ma;
  cfg.run_time_ms   = sdm_run_time_ms;
  cfg.min_deg       = sdm_home_min_deg;
  cfg.stuck_n       = sdm_home_stuck_n;
  cfg.grace_n       = sdm_home_grace_n;
  cfg.timeout       = sdm_home_timeout;
  cfg.backoff_deg   = sdm_home_backoff_deg;
  cfg.skip_backoff  = sdm_home_skip_backoff;
  cfg.maxspeed_rpm  = sdm_speed_rpm;
  sdm_home_state = SDM_Home(&hfdcan2, sdm_id, &cfg) ? 4 : 5;
}

/* ------------------------------------------------------------------ */
/* Servo task. MUST run every loop in every program_state: it drives    */
/* homing, the feedback mirror, jog, moves and holds.                   */
/* ------------------------------------------------------------------ */
void SDM_ServoTask(void)
{
  uint8_t sid = sdm_id;
  SDM_Motor_t *sm = SDM_GetMotor(sid);   /* cached feedback for this id */
  static float   sp_prev_err = 0.0f;     /* soft-pos derivative memory */
  static uint8_t sp_settled  = 0;        /* soft-pos hysteresis: 1 = parked */

  /* Config one-shots */
  if (sdm_cfg_apply) {
    switch (sdm_cfg_apply) {
      case 1: SDM_Activate(&hfdcan2, sid);      break;
      case 2: SDM_Disable(&hfdcan2, sid);       break;
      case 3: SDM_ZeroPosition(&hfdcan2, sid);  break;  /* current pos = 0 deg   */
      case 4: SDM_FaultReset(&hfdcan2, sid);    break;  /* clears a latched stall */
      default: break;
    }
    sdm_cfg_apply = 0;
  }

  /* Position move. sdm_target_deg is in the USER frame (stop = 92 deg), so
     the offset comes off before sending. */
  if (sdm_move_apply) {
    /* Activate first: after a fault reset the drive sits deactivated and
       silently ignores position commands. Harmless if already active. */
    SDM_Activate(&hfdcan2, sid);
    HAL_Delay(2);
    if (sdm_send_maxspeed) {
      SDM_SetPositionMaxSpeed(&hfdcan2, sid, sdm_speed_rpm, 0);
      HAL_Delay(2);
    }
    sdm_move_run   = 1;      /* start refreshing the target each loop */
    sp_settled     = 0;      /* fresh move: re-arm the hysteresis     */
    sp_prev_err    = 0.0f;
    sdm_move_apply = 0;
  }
  if (sdm_move_run) {
    if (sdm_softpos_enable) {
      /* ---- firmware position loop, driven by speed commands ---- */
      float here   = (sm != NULL)
                     ? ((float)sdm_dir * sm->position_deg + sdm_zero_offset_deg)
                     : sdm_target_deg;
      float target = sdm_target_deg + sdm_move_correction_deg;  /* aim past to land on */
      float err    = target - here;            /* + = need a larger angle */
      float aerr   = (err < 0.0f) ? -err : err;

      /* Hysteresis: once inside the deadband, LATCH settled and hold stopped;
         only re-engage past a larger threshold. Without this the drive's coast
         past the deadband edge keeps re-triggering the controller = wobble. */
      if (aerr < sdm_move_tol_deg)           sp_settled = 1;
      else if (aerr > sdm_move_reengage_deg) sp_settled = 0;

      if (sp_settled) {
        SDM_SetSpeedRPM(&hfdcan2, sid, 0);    /* parked: hold stopped */
        sdm_softpos_rpm = 0;
        if (!sdm_move_hold) sdm_move_run = 0; /* one-shot done; hold stays engaged */
      } else {
        float derr = err - sp_prev_err;       /* PD: pull + damping */
        sp_prev_err = err;

        float rpm_f = sdm_softpos_kp * err + sdm_softpos_kd * derr;
        float maxr  = (float)sdm_speed_rpm;
        if (rpm_f >  maxr) rpm_f =  maxr;
        if (rpm_f < -maxr) rpm_f = -maxr;
        float minr = (float)sdm_softpos_min_rpm;
        if (rpm_f > 0.0f && rpm_f <  minr) rpm_f =  minr;
        if (rpm_f < 0.0f && rpm_f > -minr) rpm_f = -minr;

        /* err is in the user frame, so the speed sign needs the same flip */
        sdm_softpos_rpm = (int16_t)rpm_f;
        SDM_SetSpeedRPM(&hfdcan2, sid, (int16_t)(sdm_dir * sdm_softpos_rpm));
      }
    } else {
      /* ---- native position frame ---- */
      SDM_SetPositionDeg(&hfdcan2, sid,
                         (float)sdm_dir *
                         (sdm_target_deg + sdm_move_correction_deg - sdm_zero_offset_deg));
      if (sm != NULL) {
        float perr = ((float)sdm_dir * sm->position_deg + sdm_zero_offset_deg) - sdm_target_deg;
        if (perr < 0.0f) perr = -perr;
        if (perr < sdm_move_tol_deg) sdm_move_run = 0;
      }
    }
  }

  /* Jog: sdm_jog_apply starts it, then the setpoint is refreshed every loop
     for as long as sdm_jog_run stays 1 (the drive stops if commands stop). */
  if (sdm_jog_apply) {
    SDM_Activate(&hfdcan2, sid);
    HAL_Delay(2);
    sdm_jog_run   = 1;
    sdm_jog_apply = 0;
  }
  if (sdm_jog_run) {
    SDM_SetSpeedRPM(&hfdcan2, sid, sdm_jog_rpm);
    if (sdm_jog_rpm == 0) sdm_jog_run = 0;   /* commanded stop: done */
  }

  /* Mirror the 50 Hz feedback into flat Ozone-friendly variables */
  if (sm) {
    sdm_pos_deg      = sm->position_deg;
    sdm_pos_user_deg = (float)sdm_dir * sm->position_deg + sdm_zero_offset_deg;
    sdm_pos_raw      = sm->position_raw;
    sdm_speed_fb_rpm = sm->speed_rpm;
    sdm_status       = sm->status;
    sdm_mode         = (uint8_t)SDM_ST_MODE(sm->status);
    sdm_stalled      = (uint8_t)SDM_ST_STALLED(sm->status);
    sdm_fault        = (uint8_t)SDM_ST_FAULT(sm->status);
    sdm_voltage_v    = sm->voltage_v;
    sdm_drv_temp_c   = sm->drv_temp_c;
  }

  /* Ozone bench trigger; arm_state 1 calls SDM_DoHome() via Arm_StartupHoming. */
  if (sdm_home_apply) { sdm_home_apply = 0; SDM_DoHome(); }
}



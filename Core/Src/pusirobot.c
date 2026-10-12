// file: pusirobot.c
// PUSIROBOT PMC006C3 CANopen driver (classic CAN 2.0, expedited SDO) on FDCAN1.
// See pusirobot.h for the protocol summary and object dictionary.
// Drives nodes 3/4 (string spools + joint rotation) and 7/8 (grasping).

#include "pusirobot.h"
#include <string.h>

/* ---- diagnostics (Ozone-visible) ---- */
volatile uint32_t g_pusi_sdo_abort   = 0;
volatile uint32_t g_pusi_tx_err      = 0;
volatile uint32_t g_pusi_rx_count    = 0;
volatile uint32_t g_pusi_timeout_cnt = 0;

/* ---- bus + last-reply capture (written by the RX ISR) ---- */
static FDCAN_HandleTypeDef *pusi_hfdcan = NULL;

static volatile uint8_t  pusi_resp_flag = 0;    // 1 = a reply arrived since the request
static volatile uint8_t  pusi_resp_node = 0;
static volatile uint8_t  pusi_resp_cmd  = 0;    // 0x4F/4B/47/43 read, 0x60 wr-ok, 0x80 abort
static volatile uint16_t pusi_resp_idx  = 0;
static volatile uint8_t  pusi_resp_sub  = 0;
static volatile uint8_t  pusi_resp_d[4]  = {0,0,0,0};

void PUSI_Init(FDCAN_HandleTypeDef *hfdcan)
{
    pusi_hfdcan = hfdcan;
}

/* Called from the FDCAN1 RX interrupt for STANDARD-id frames. Only 0x580..0x5FF
   are SDO replies; everything else on the bus (0xC1-0xC4 motor drives, etc.) is
   ignored here. Transactions are serialized by the blocking helpers, so a single
   last-reply slot is sufficient. */
void PUSI_HandleSdoResp(uint32_t id, const uint8_t *data, uint8_t len)
{
    if (id < 0x580u || id > 0x5FFu) return;   // not an SDO response COB-ID
    if (!data || len < 8u) return;

    pusi_resp_node = (uint8_t)(id - 0x580u);
    pusi_resp_cmd  = data[0];
    pusi_resp_idx  = (uint16_t)(data[1] | ((uint16_t)data[2] << 8));
    pusi_resp_sub  = data[3];
    pusi_resp_d[0] = data[4];
    pusi_resp_d[1] = data[5];
    pusi_resp_d[2] = data[6];
    pusi_resp_d[3] = data[7];
    g_pusi_rx_count++;
    pusi_resp_flag = 1;
}

/* ------------------------------------------------------------------ */
/* Low-level: send one 8-byte SDO frame, then wait for the reply.      */
/* ------------------------------------------------------------------ */
static HAL_StatusTypeDef pusi_tx8(uint8_t node, const uint8_t *d8)
{
    if (pusi_hfdcan == NULL) return HAL_ERROR;

    FDCAN_TxHeaderTypeDef h;
    h.Identifier          = 0x600u + node;
    h.IdType              = FDCAN_STANDARD_ID;
    h.TxFrameType         = FDCAN_DATA_FRAME;
    h.DataLength          = FDCAN_DLC_BYTES_8;
    h.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    h.BitRateSwitch       = FDCAN_BRS_OFF;
    h.FDFormat            = FDCAN_CLASSIC_CAN;
    h.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    h.MessageMarker       = 0;

    HAL_StatusTypeDef s = HAL_FDCAN_AddMessageToTxFifoQ(pusi_hfdcan, &h, (uint8_t *)d8);
    if (s != HAL_OK) g_pusi_tx_err++;
    return s;
}

/* Wait (spin) for the matching reply. Returns HAL_OK / HAL_TIMEOUT. */
static HAL_StatusTypeDef pusi_wait_reply(uint8_t node, uint16_t index)
{
    uint32_t t0 = HAL_GetTick();
    while (!pusi_resp_flag) {
        if ((HAL_GetTick() - t0) > PUSI_SDO_TIMEOUT_MS) {
            g_pusi_timeout_cnt++;
            return HAL_TIMEOUT;
        }
    }
    /* Guard against a stale/foreign reply landing in the slot. */
    if (pusi_resp_node != node || pusi_resp_idx != index) {
        g_pusi_timeout_cnt++;
        return HAL_TIMEOUT;
    }
    return HAL_OK;
}

static uint8_t pusi_read_len(uint8_t cmd)
{
    switch (cmd) {
        case 0x4Fu: return 1;
        case 0x4Bu: return 2;
        case 0x47u: return 3;
        case 0x43u: return 4;
        default:    return 0;
    }
}

HAL_StatusTypeDef PUSI_SdoRead(uint8_t node, uint16_t index, uint8_t sub, int32_t *value_out)
{
    uint8_t req[8] = { 0x40u, (uint8_t)(index & 0xFF), (uint8_t)(index >> 8), sub, 0, 0, 0, 0 };
    pusi_resp_flag = 0;
    if (pusi_tx8(node, req) != HAL_OK) return HAL_ERROR;
    if (pusi_wait_reply(node, index) != HAL_OK) return HAL_TIMEOUT;

    if (pusi_resp_cmd == 0x80u) {                    /* SDO abort */
        g_pusi_sdo_abort = (uint32_t)pusi_resp_d[0]        | ((uint32_t)pusi_resp_d[1] << 8)
                        | ((uint32_t)pusi_resp_d[2] << 16) | ((uint32_t)pusi_resp_d[3] << 24);
        return HAL_ERROR;
    }
    uint8_t n = pusi_read_len(pusi_resp_cmd);
    if (n == 0) return HAL_ERROR;

    int32_t v = 0;
    for (uint8_t i = 0; i < n; i++) v |= (int32_t)pusi_resp_d[i] << (8 * i);
    g_pusi_sdo_abort = 0;
    if (value_out) *value_out = v;
    return HAL_OK;
}

HAL_StatusTypeDef PUSI_SdoWrite(uint8_t node, uint16_t index, uint8_t sub, uint32_t value, uint8_t nbytes)
{
    uint8_t cmd;
    switch (nbytes) {
        case 1: cmd = 0x2Fu; break;
        case 2: cmd = 0x2Bu; break;
        case 3: cmd = 0x27u; break;
        default: cmd = 0x23u; nbytes = 4; break;
    }
    uint8_t req[8] = { cmd, (uint8_t)(index & 0xFF), (uint8_t)(index >> 8), sub,
                       (uint8_t)(value & 0xFF), (uint8_t)((value >> 8) & 0xFF),
                       (uint8_t)((value >> 16) & 0xFF), (uint8_t)((value >> 24) & 0xFF) };
    pusi_resp_flag = 0;
    if (pusi_tx8(node, req) != HAL_OK) return HAL_ERROR;
    if (pusi_wait_reply(node, index) != HAL_OK) return HAL_TIMEOUT;

    if (pusi_resp_cmd == 0x80u) {                    /* SDO abort */
        g_pusi_sdo_abort = (uint32_t)pusi_resp_d[0]        | ((uint32_t)pusi_resp_d[1] << 8)
                        | ((uint32_t)pusi_resp_d[2] << 16) | ((uint32_t)pusi_resp_d[3] << 24);
        return HAL_ERROR;
    }
    /* 0x60 = write confirmed */
    g_pusi_sdo_abort = 0;
    return (pusi_resp_cmd == 0x60u) ? HAL_OK : HAL_ERROR;
}

/* ------------------------------------------------------------------ */
/* Motor enable / microstep / current (0x600E / 0x600A / 0x600B)       */
/* ------------------------------------------------------------------ */
HAL_StatusTypeDef PUSI_EnableMotor(uint8_t node, uint8_t enable)
{
    return PUSI_SdoWrite(node, PUSI_OBJ_ENABLE, 0, enable ? 1u : 0u, 1);
}

HAL_StatusTypeDef PUSI_SetMicrostep(uint8_t node, uint16_t microstep)
{
    return PUSI_SdoWrite(node, PUSI_OBJ_MICROSTEP, 0, microstep, 2);
}

HAL_StatusTypeDef PUSI_SetRunCurrent(uint8_t node, uint16_t current_mA)
{
    if (current_mA > 6000u) return HAL_ERROR;
    return PUSI_SdoWrite(node, PUSI_OBJ_RUN_CURRENT, 0, current_mA, 2);
}

/* ------------------------------------------------------------------ */
/* 1) Rotate at a speed (velocity mode)                                */
/* ------------------------------------------------------------------ */
HAL_StatusTypeDef PUSI_SetSpeed(uint8_t node, int32_t speed_pps)
{
    /* Mode + enable are BEST-EFFORT. When changing speed on the fly the motor is
       already running and the drive may REJECT a mode/enable re-write (device-
       state abort); that must not stop the new speed from being written. Only
       0x6003 changes the running speed, and the manual allows rewriting it live. */
    (void)PUSI_SdoWrite(node, PUSI_OBJ_WORK_MODE, 0, PUSI_MODE_SPEED, 1);
    (void)PUSI_EnableMotor(node, 1);
    /* 0x6003 is signed; sign selects direction. */
    return PUSI_SdoWrite(node, PUSI_OBJ_SPEED, 0, (uint32_t)speed_pps, 4);
}

/* ------------------------------------------------------------------ */
/* 2) Rotate a relative amount (position mode)                         */
/* ------------------------------------------------------------------ */
HAL_StatusTypeDef PUSI_MoveRel(uint8_t node, uint32_t steps, uint8_t dir,
                              int32_t speed_pps, uint16_t accel, uint16_t decel)
{
    if (steps == 0u) return HAL_ERROR;   /* 0x6004 = 0 is an error per the manual */

    /* Position mode only accepts new step/speed/dir commands while IDLE - if the
       busy bit is set the writes are silently dropped. Bail so the caller retries. */
    int32_t st;
    if (PUSI_SdoRead(node, PUSI_OBJ_STATUS, 0, &st) == HAL_OK && PUSI_ST_BUSY((uint8_t)st))
        return HAL_BUSY;

    HAL_StatusTypeDef s;
    s = PUSI_SdoWrite(node, PUSI_OBJ_WORK_MODE, 0, PUSI_MODE_POSITION, 1);          if (s != HAL_OK) return s;
    s = PUSI_EnableMotor(node, 1);                                               if (s != HAL_OK) return s;  /* energize - required or it stalls/stops */
    s = PUSI_SdoWrite(node, PUSI_OBJ_DIRECTION, 0, (dir ? 1u : 0u), 1);            if (s != HAL_OK) return s;
    s = PUSI_SdoWrite(node, PUSI_OBJ_SPEED,     0, (uint32_t)speed_pps, 4);        if (s != HAL_OK) return s;
    /* Accel/decel (0x6008/0x6009): range 0..8 -> 1-byte (U8). The reference
       listed them as U16, but writing 2 bytes returned abort 0x06070010 (length
       mismatch), so send 1 byte. Best-effort: a bad width here must NOT block the
       move, so we do NOT return on failure - the 0x6004 trigger still fires.
       Set accel/decel > 8 to skip the write entirely. */
    if (accel <= 8u) (void)PUSI_SdoWrite(node, PUSI_OBJ_ACCEL, 0, accel, 1);
    if (decel <= 8u) (void)PUSI_SdoWrite(node, PUSI_OBJ_DECEL, 0, decel, 1);
    /* Writing the relative-move count triggers the move. */
    return PUSI_SdoWrite(node, PUSI_OBJ_MOVE_REL, 0, steps, 4);
}

/* ------------------------------------------------------------------ */
/* 3) Stall detection                                                  */
/* ------------------------------------------------------------------ */
HAL_StatusTypeDef PUSI_ReadStatus(uint8_t node, uint8_t *status_out)
{
    int32_t v;
    HAL_StatusTypeDef s = PUSI_SdoRead(node, PUSI_OBJ_STATUS, 0, &v);
    if (s == HAL_OK && status_out) *status_out = (uint8_t)v;
    return s;
}

HAL_StatusTypeDef PUSI_ClearStall(uint8_t node)
{
    /* Full fault/stall reset. Write 0xFF (write-1-to-clear) to BOTH the motor
       error register (0x6000) and the controller status (0x6001). A stall can
       latch error bits in 0x6000 as well; clearing only 0x6001 bit2 leaves those
       set and the drive will not move again until 0x6000 is cleared too. */
    HAL_StatusTypeDef s = PUSI_SdoWrite(node, PUSI_OBJ_ERROR, 0, 0xFFu, 1);   /* 0x6000 = FF */
    if (s != HAL_OK) return s;
    return PUSI_SdoWrite(node, PUSI_OBJ_STATUS, 0, 0xFFu, 1);                  /* 0x6001 = FF */
}

HAL_StatusTypeDef PUSI_ConfigStall(uint8_t node, uint16_t open_loop_thresh, uint8_t stop_on_stall)
{
    HAL_StatusTypeDef s = PUSI_SdoWrite(node, PUSI_OBJ_STALL_PARAM, 0, (open_loop_thresh & 0x7Fu), 2);
    if (s != HAL_OK) return s;
    return PUSI_SdoWrite(node, PUSI_OBJ_STALL_STOP, 0, (stop_on_stall ? 1u : 0u), 1);
}

/* ------------------------------------------------------------------ */
/* 4) Stop                                                             */
/* ------------------------------------------------------------------ */
HAL_StatusTypeDef PUSI_Stop(uint8_t node)
{
    /* 0x6020 = 0 -> immediate stop, NO deceleration ramp (hard stop). */
    return PUSI_SdoWrite(node, PUSI_OBJ_STOP, 0, 0u, 1);
}

/* ------------------------------------------------------------------ */
/* Feedback reads                                                      */
/* ------------------------------------------------------------------ */
HAL_StatusTypeDef PUSI_ReadSpeed(uint8_t node, int32_t *spd_out)
{
    return PUSI_SdoRead(node, PUSI_OBJ_RT_SPEED, 0, spd_out);
}

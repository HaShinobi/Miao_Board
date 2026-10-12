// file: arm_control.c
// Spool motion for the soft arm. See arm_control.h for the layout.

#include "main.h"          /* r_motor, and the CAN handles */
#include "arm_control.h"
#include "pusirobot.h"
#include "sdm8070.h"       /* sdm_pos_user_deg: base angle for the tendon reference */
#include <math.h>

/* Drive settings shared by all four spools. 0x600A microstep gives
   steps/rev = 200 * microstep; 0x6017 is the open-loop stall threshold. */
volatile uint16_t spool_microstep    = 16;
volatile uint16_t spool_stall_thresh = 4;

/* ==================================================================== */
/*  STRING PAYOUT                                                       */
/* ==================================================================== */
volatile uint8_t  string_enable       = 1;
volatile float    string_excess_mm    = 80.0f;   // > the 78.54 mm prewind, so tracking
                                                 // engages within ~3.5 mm of travel
volatile float    string_step_mm      = 5.0f;
volatile float    string_paid_mm      = 0.0f;
volatile float    string_pos_mm       = 0.0f;
volatile float    string_sep_pos_mm   = 0.0f;
volatile float    string_reel_mm      = 0.0f;
volatile float    string_sep_delay_ms = 800.0f;

/* ---- state-5 reel-back geometry --------------------------------------
   P1 = tendon connect point, P2 = joint-forming point, O = base reference.
   ΔL = |P1P2| and L0 = |P1O| are fixed mechanical offsets measured on the
   assembly. With X = string_sep_pos_mm (slider's tracked travel) and
   L_re = string_paid_mm (tendon length actually released during fill):

     L_positioning = X + L0                 (P1' to O once the slider moved)
     M             = midpoint of L_positioning
     P2'O          = X + L0 - ΔL
     geometric pull-back = P2'O - M = (X + L0)/2 - ΔL
     slack still in the tendon = L_re - X   (released minus travelled)

     string_reel_mm = geometric pull-back + slack + string_reel_extra_mm

   See ArmState_SeparateSliders() in arm_states.c for where this is used. */
volatile float    sep_delta_L_mm        = 81.25f;   // ΔL = |P1P2| [mm]
volatile float    sep_L0_mm             = 174.26f;  // L0 = |P1O| at the reference pose [mm]
volatile float    sep_geometric_pull_mm = 0.0f;     // readback: (X+L0)/2 - ΔL
volatile float    sep_slack_mm          = 0.0f;     // readback: L_re - X

volatile uint8_t  string_node_up      = 3;
volatile uint8_t  string_node_down    = 4;
/* Payout sense per node, at the drive: +1 = CW, -1 = CCW. The spools are
   mounted mirrored, so opposite commanded signs give the SAME physical
   rotation - that is why these are not identical.
   Judge polarity during arm_state 2 payout only: the sense inverts on
   reel-in, so state 5 always looks backwards.                          */
volatile int8_t   string_dir_up       = PUSI_CW;    /* -1 */
volatile int8_t   string_dir_down     = PUSI_CCW;   /* +1 */

volatile float    string_prewind_rev  = 1.0f;    // 1 rev = 2*pi*12.5 = 78.54 mm
volatile float    string_prewind_mm   = 0.0f;
volatile uint8_t  string_prewind_pending = 0;
volatile int32_t  string_speed_pps    = 2000;
volatile uint16_t string_accel        = 5;
volatile uint16_t string_decel        = 5;
volatile uint16_t string_steps_per_rev = 4000;
volatile uint16_t string_current_ma   = 1000;
volatile uint8_t  string_stall_stop   = 0;

volatile uint16_t string_reel_current_ma  = 1500;
volatile float    string_chunk_mm         = 20.0f;
volatile uint32_t string_chunk_timeout_ms = 4000;
volatile float    string_reel_extra_mm    = 1.0f;
volatile float    string_reel_done_mm     = 0.0f;
volatile uint8_t  string_reel_abort       = 0;

volatile uint8_t  string_init_done    = 0;
volatile uint8_t  string_err_up       = 0;
volatile uint8_t  string_err_down     = 0;
volatile float    string_last_deg     = 0.0f;
volatile int32_t  string_spd_up       = 0;
volatile int32_t  string_spd_down     = 0;

/* ---- second payout pair: nodes 5 / 6 ----------------------------------
   Mirrors nodes 3/4 during the fill and does nothing else - no reel-in, no
   joint rotation, no tensioning home. Shares the slack policy
   (string_excess_mm, string_step_mm), the spool radius and all the drive
   settings, so it always pays out exactly the same length.
   Directions default to the same signs as pair 0; flip in Ozone during a
   state-2 payout if a spool winds the wrong way.                       */
volatile uint8_t  string2_enable      = 1;
volatile uint8_t  string2_node_up     = 5;
volatile uint8_t  string2_node_down   = 6;
/* Same length, radius, steps/rev and speed as pair 0 - only the rotation
   senses differ. Bench-set: node 5 CCW, node 6 CW, which is the OPPOSITE
   of nodes 3/4 (3 = CW, 4 = CCW). Judge polarity during payout only.  */
volatile int8_t   string2_dir_up      = PUSI_CCW;   /* +1, node 5 */
volatile int8_t   string2_dir_down    = PUSI_CW;    /* -1, node 6 */
/* Only trim if 5/6 measurably travel a different distance for the same
   command: 1.0 = identical, 0.5 = half as far. Everything else (radius,
   steps/rev, speed, accel) is shared with pair 0.                      */
volatile float    string2_scale       = 1.0f;
volatile float    string2_paid_mm     = 0.0f;
volatile uint8_t  string2_init_done   = 0;
volatile uint8_t  string2_err_up      = 0;
volatile uint8_t  string2_err_down    = 0;
volatile int32_t  string2_spd_up      = 0;
volatile int32_t  string2_spd_down    = 0;

/* 0x6005 as read back after the last Pair_Config, for the pair configured
   most recently. 0 = position mode (wanted), 1 = speed mode (the motor will
   run continuously), 0xFF = the read failed. Nodes 5/6 are configured after
   3/4 in arm_state 2, so during a fill these show pair 1.               */
volatile uint8_t pair_mode_up   = 0xFF;
volatile uint8_t pair_mode_down = 0xFF;

/* Wait until RT_SPEED (0x6030) settles near zero. The busy bit (0x6001) only
   gates POSITION-mode command acceptance (see PUSI_MoveRel) and is not
   documented to reflect SPEED-mode motion, so it cannot be trusted to mean
   "stopped" - RT_SPEED is the only signal that actually does. A work-mode
   change attempted while the drive still thinks it is running is silently
   rejected, which is how nodes 5/6 were found stuck in speed mode. */
static uint8_t Pusi_WaitStopped(uint8_t node, uint32_t timeout_ms)
{
  uint32_t t0 = HAL_GetTick();
  for (;;) {
    int32_t spd;
    if (PUSI_ReadSpeed(node, &spd) == HAL_OK) {
      if (spd < 0) spd = -spd;
      if (spd < 5) return 1;
    }
    if ((HAL_GetTick() - t0) > timeout_ms) return 0;
    HAL_Delay(5);
  }
}

/* Put a spool pair into a known state: drop any latched trip, set the stall
   policy, set run current, energize. Every path that touches a spool pair
   goes through here, so init, the reel-in chunks and the joint home can
   never disagree about drive configuration.
   0x6017 max (0x7F) + stop_on_stall 0 = "ignore stall entirely".        */
static void Pair_Config(uint8_t nu, uint8_t nd,
                        uint16_t current_ma, uint16_t thresh, uint8_t stop_on_stall)
{
  const uint8_t n[2] = { nu, nd };
  for (int i = 0; i < 2; i++) {
    /* ORDER MATTERS. In SPEED mode the drive reads the 0x6003 write inside
       PUSI_MoveRel as "run at this speed forever" and ignores the step count
       in 0x6004 - the motor never stops, which is what nodes 5/6 do.

       The drive will not accept a work-mode change while it is turning, and
       apparently not while it is ENERGIZED either - the same trap the UIM242
       drives had with their config writes. So the sequence is:
         stop -> speed 0 -> DISABLE -> wait for RT_SPEED to settle ->
         set mode (retry + verify) -> re-enable.
       pair_mode_* records what 0x6005 reads back afterwards so the result
       can be seen in Ozone: 0 = position (wanted), 1 = speed, 0xFF = the
       read itself failed.                                                */
    PUSI_Stop(n[i]);                                            /* 0x6020 = 0 */
    (void)PUSI_SdoWrite(n[i], PUSI_OBJ_SPEED,     0, 0u, 4);    /* 0x6003 = 0 */
    (void)PUSI_EnableMotor(n[i], 0);                            /* 0x600E = 0 */

    (void)Pusi_WaitStopped(n[i], 200);   /* confirm it's actually stopped */

    /* Retry + verify: a single unverified write can race the drive's
       internal state and fail with no error surfaced anywhere else. */
    uint8_t m = 0xFFu;
    for (uint8_t attempt = 0; attempt < 5; attempt++) {
      (void)PUSI_SdoWrite(n[i], PUSI_OBJ_WORK_MODE, 0, PUSI_MODE_POSITION, 1);
      int32_t v;
      m = (PUSI_SdoRead(n[i], PUSI_OBJ_WORK_MODE, 0, &v) == HAL_OK)
          ? (uint8_t)v : 0xFFu;
      if (m == PUSI_MODE_POSITION) break;
      HAL_Delay(10);
    }
    if (i == 0) pair_mode_up = m; else pair_mode_down = m;
    (void)PUSI_EnableMotor(n[i], 1);                            /* 0x600E = 1 */

    PUSI_ClearStall(n[i]);                             /* 0x6000/0x6001 = FF */
    PUSI_ConfigStall(n[i], thresh, stop_on_stall);     /* 0x6017 + 0x601B    */
    PUSI_SetRunCurrent(n[i], current_ma);              /* 0x600B             */
    PUSI_EnableMotor(n[i], 1);                         /* 0x600E - required  */
  }
}

/* Hard stop a spool pair and leave it in position mode, still energized. */
static void Pair_Stop(uint8_t nu, uint8_t nd)
{
  const uint8_t n[2] = { nu, nd };
  for (int i = 0; i < 2; i++) {
    PUSI_Stop(n[i]);                                           /* 0x6020 = 0 */
    (void)PUSI_SdoWrite(n[i], PUSI_OBJ_SPEED,     0, 0u, 4);
    (void)PUSI_SdoWrite(n[i], PUSI_OBJ_WORK_MODE, 0, PUSI_MODE_POSITION, 1);
    PUSI_EnableMotor(n[i], 1);
  }
}

/* Stop all four spools - nodes 3/4 and 5/6. */
void String_StopAll(void)
{
  Pair_Stop(string_node_up,  string_node_down);
  Pair_Stop(string2_node_up, string2_node_down);
}

/* Stop nodes 5/6 only. Pair 1 pays out during arm_state 2 and is used by
   nothing else, so every state change stops it - it must never be left
   turning once the fill ends.                                          */
void String2_Stop(void)
{
  Pair_Stop(string2_node_up, string2_node_down);
}

static void String_ConfigNodes(uint16_t current_ma, uint16_t thresh, uint8_t stop_on_stall)
{
  Pair_Config(string_node_up, string_node_down, current_ma, thresh, stop_on_stall);
}

/* Move one spool pair by length_mm (+ pays out, - reels in). Direction is
   the SIGN OF THE SPEED (0x6003); the step count is always positive because
   0x6004 is unsigned and zero is an error. Readbacks go to the caller's
   variables so both pairs stay independently visible in Ozone.          */
static void Pair_Move(uint8_t nu, uint8_t nd, int8_t du, int8_t dd,
                      float length_mm,
                      float radius_mm, uint16_t steps_rev, int32_t speed_pps,
                      volatile uint8_t *err_up,  volatile uint8_t *err_down,
                      volatile int32_t *spd_up,  volatile int32_t *spd_down)
{
  float d   = (length_mm < 0.0f) ? -length_mm : length_mm;
  float deg = (d / radius_mm) * 180.0f / 3.1415926f;
  if (deg < 0.05f) return;

  uint32_t steps = (uint32_t)((deg / 360.0f) * (float)steps_rev + 0.5f);
  if (steps == 0u) return;
  string_last_deg = deg;

  float  sgn = (length_mm >= 0.0f) ? 1.0f : -1.0f;
  int8_t su  = ((sgn * (float)du) >= 0.0f) ? 1 : -1;
  int8_t sd  = ((sgn * (float)dd) >= 0.0f) ? 1 : -1;

  *spd_up   = speed_pps * su;
  *spd_down = speed_pps * sd;

  PUSI_ClearStall(nu);                            /* drop any latched trip */
  PUSI_ClearStall(nd);

  *err_up   = (uint8_t)PUSI_MoveRel(nu, steps,
                                    (su > 0) ? PUSI_DIR_FORWARD : PUSI_DIR_REVERSE,
                                    *spd_up, string_accel, string_decel);
  *err_down = (uint8_t)PUSI_MoveRel(nd, steps,
                                    (sd > 0) ? PUSI_DIR_FORWARD : PUSI_DIR_REVERSE,
                                    *spd_down, string_accel, string_decel);
}

void String_PmcInit(void)
{
  /* Stop before configuring: a spinning drive rejects microstep and
     work-mode writes, and changing microstep mid-run would jump. */
  Pair_Stop(string_node_up, string_node_down);
  PUSI_SetMicrostep(string_node_up,   spool_microstep);  /* 0x600A, once */
  PUSI_SetMicrostep(string_node_down, spool_microstep);
  String_ConfigNodes(string_current_ma, spool_stall_thresh, string_stall_stop);
  string_init_done = 1;
}

/* Pair 0 (nodes 3/4). + = pay out, - = reel in. */
void String_Move(float length_mm)
{
  if (!string_init_done) String_PmcInit();
  Pair_Move(string_node_up, string_node_down, string_dir_up, string_dir_down,
            length_mm, r_motor, string_steps_per_rev, string_speed_pps,
            &string_err_up, &string_err_down,
            &string_spd_up, &string_spd_down);
}

/* Pair 1 (nodes 5/6) - payout only, never called for reel-in. */
void String2_Init(void)
{
  /* Stop first - see String_PmcInit. Nodes 5/6 are the pair that was found
     spinning in speed mode, so this ordering matters most here. */
  Pair_Stop(string2_node_up, string2_node_down);
  PUSI_SetMicrostep(string2_node_up,   spool_microstep);
  PUSI_SetMicrostep(string2_node_down, spool_microstep);
  Pair_Config(string2_node_up, string2_node_down,
              string_current_ma, spool_stall_thresh, string_stall_stop);
  string2_init_done = 1;
}

void String2_Move(float length_mm)
{
  if (!string2_enable) return;
  if (!string2_init_done) String2_Init();
  /* Identical constants to pair 0 - same radius, steps/rev and speed - so
     nodes 5/6 pay out exactly what nodes 3/4 do. */
  Pair_Move(string2_node_up, string2_node_down, string2_dir_up, string2_dir_down,
            length_mm * string2_scale, r_motor, string_steps_per_rev, string_speed_pps,
            &string2_err_up, &string2_err_down,
            &string2_spd_up, &string2_spd_down);
}

/* 1 = idle, 0 = timed out. */
static uint8_t String_WaitIdle(uint8_t node, uint32_t timeout_ms)
{
  uint32_t t0 = HAL_GetTick();
  for (;;) {
    int32_t st;
    if (PUSI_SdoRead(node, PUSI_OBJ_STATUS, 0, &st) == HAL_OK &&
        !PUSI_ST_BUSY((uint8_t)st)) return 1;
    if ((HAL_GetTick() - t0) > timeout_ms) return 0;
    HAL_Delay(10);
  }
}

/* Loaded pull (arm_state 5). Chunked so a latched fault costs one chunk,
   not the whole move, and each chunk is verified idle before the next. */
void String_ReelIn(float total_mm)
{
  string_reel_done_mm = 0.0f;
  string_reel_abort   = 0;
  if (total_mm <= 0.05f) return;

  float remaining = total_mm;
  while (remaining > 0.05f) {
    float chunk = (remaining > string_chunk_mm) ? string_chunk_mm : remaining;

    /* Re-assert torque + "ignore stall" before EVERY chunk - the string is in
       tension here and a latched trip would abort the move part-way. */
    String_ConfigNodes(string_reel_current_ma, 0x7Fu, 0u);
    String_Move(-chunk);

    if (!String_WaitIdle(string_node_up,   string_chunk_timeout_ms) ||
        !String_WaitIdle(string_node_down, string_chunk_timeout_ms)) {
      string_reel_abort = 1;                       /* jammed - leave the rest undone */
      break;
    }
    remaining           -= chunk;
    string_reel_done_mm += chunk;
    string_paid_mm      -= chunk;
  }

  String_ConfigNodes(string_current_ma, spool_stall_thresh, string_stall_stop);
}

/* Pair 1 (nodes 5/6) loaded reel-in, for arm_state 14. on = 1 sets the reel
   current and ignores stall for the whole sequence; on = 0 restores the
   normal config. Set once per sequence, not per pull: every Pair_Config
   briefly de-energises the spools while the chamber is pushing on them. */
void String2_ReelMode(uint8_t on)
{
  if (!string2_init_done) String2_Init();
  if (on) Pair_Config(string2_node_up, string2_node_down,
                      string_reel_current_ma, 0x7Fu, 0u);
  else    Pair_Config(string2_node_up, string2_node_down,
                      string_current_ma, spool_stall_thresh, string_stall_stop);
}

/* Reel nodes 5/6 in by length_mm and wait for both to finish.
   1 = done, 0 = a spool was still busy after string_chunk_timeout_ms. */
uint8_t String2_ReelIn(float length_mm)
{
  if (length_mm <= 0.05f) return 1;
  String2_Move(-length_mm);
  if (!String_WaitIdle(string2_node_up,   string_chunk_timeout_ms) ||
      !String_WaitIdle(string2_node_down, string_chunk_timeout_ms)) return 0;
  string2_paid_mm -= length_mm;
  return 1;
}

/* ==================================================================== */
/*  JOINT ROTATION                                                      */
/* ==================================================================== */
/* Polarity trim per node, at the drive: -1 = CW, +1 = CCW. Both CCW is
   bench-confirmed - the mirrored mounting already gives the differential. */
volatile int8_t   joint_dir_up    = PUSI_CCW;   /* +1 */
volatile int8_t   joint_dir_down  = PUSI_CCW;   /* +1 */
/* Which way phi moves for dir 0. -1 = bench-corrected so dir 0 goes UP.
   NOT a rotation sense - this maps the dir argument onto the pose angle. */
volatile int8_t   joint_phi_sense = -1;
/* Software travel limits on the tracked pose. dL/dphi goes as sin(phi), so
   the slope - and therefore the pull direction for a given dir - reverses
   at 0 and 180 deg. Clamping phi here is what stops the joint running away
   backwards once it passes a limit. Narrow these to the real mechanical
   range if it is tighter than 0..180.                                   */
volatile float    joint_phi_min_deg = 0.0f;
volatile float    joint_phi_max_deg = 180.0f;
volatile uint8_t  joint_limit_hit   = 0;    /* 1 = a limit was reached */
volatile int32_t  joint_speed_pps = 2000;
volatile uint16_t joint_accel     = 5;
volatile uint16_t joint_decel     = 5;
volatile uint8_t  joint_err_up    = 0;
volatile uint8_t  joint_err_down  = 0;
volatile int32_t  joint_spd_up    = 0;
volatile int32_t  joint_spd_down  = 0;
volatile float    joint_motor_deg = 0.0f;
volatile uint8_t  joint_skip      = 0;
volatile uint32_t joint_moves     = 0;

/* Geometry from CAD. phi = angle at the joint centre between O->anchor
   and O->fixed point. MEASURED at the homed pose, not estimated.      */
volatile float    joint_Ra           = 78.55f;   // 157.10 / 2
volatile float    joint_Bx           = 68.62f;
volatile float    joint_By           = 33.40f;
volatile float    joint_phi_home_deg = 143.5f;   // implies X = 147.1 mm
volatile float    joint_phi_deg      = 143.5f;
volatile float    joint_dX_mm        = 0.0f;
volatile float    joint_X_mm         = 0.0f;
volatile float    joint_gain         = 1.0f;     // geometry is exact; no fudge needed
volatile float    joint_slack_deg    = 0.0f;
volatile float    joint_cmd_deg      = 0.0f;

volatile uint8_t  joint_home_apply      = 0;
volatile int32_t  joint_home_pps        = 400;
volatile uint16_t joint_home_current_ma = 500;
volatile uint16_t joint_home_thresh     = 4;
volatile uint32_t joint_home_timeout_ms = 8000;
volatile uint8_t  joint_home_state      = 0;
volatile uint8_t  joint_homed           = 0;
volatile uint32_t joint_home_ms         = 0;

/* ---- RT_SPEED stall fallback - small slider (nodes 5/6) ONLY ----------
   Bench-observed: the small slider's open-loop stall bit (0x6017) does not
   reliably trip at the mechanical stop - the current reading actually DROPS
   as it approaches the stop rather than rising, so PUSI_ST_STALLED never
   fires and the tendon just keeps sliding. As a fallback, also watch
   RT_SPEED (0x6030): if a motor's measured speed stays near zero for
   string2_stall_stuck_n polls in a row, treat that as stalled too -
   independent of whatever the drive itself reports. Same idea as
   SDM_Home()'s "stop found by the angle no longer advancing".
   Opt-in via Pair_TensionHome()'s speed_fallback argument - ONLY
   init_small_slider() enables it. Nodes 3/4 and the arm_state 1/2 startup
   home are unaffected. */
volatile int32_t  string2_stall_speed_pps = 5;    // |RT_SPEED| below this = "not moving" [pps]
volatile uint8_t  string2_stall_stuck_n   = 5;    // consecutive not-moving polls (~10ms apart) = stalled
volatile uint16_t string2_stall_grace_n   = 50;   // ignore the detector for this many polls after start (~0.5s)

/* ---- RT_SPEED-confirmed stall - joint-forming pair, DOWN node only ----
   Bench-observed: node 4 (string_node_down) consistently trips its
   open-loop stall bit EARLIER than node 3, before the tendon is actually
   taut, leaving that side slightly slack and tilting the slider up. This is
   the opposite problem from the small slider above (over-triggering, not
   under-triggering), so the fix runs the other way: when node 4's stall
   bit trips, don't accept it immediately - clear the fault and keep
   pulling, only accepting the stop once RT_SPEED ALSO confirms near-zero
   speed for a sustained stretch. Capped at string_down_confirm_max_retries
   clear-and-resume attempts so a genuine jam can't retry forever - once
   exhausted, the drive's own stall bit is accepted as-is.
   Opt-in via Pair_TensionHome()'s confirm_down_stall argument. */
volatile int32_t  string_down_confirm_speed_pps   = 5;   // |RT_SPEED| below this = "not moving" [pps]
volatile uint8_t  string_down_confirm_stuck_n     = 5;   // consecutive not-moving polls = confirmed stall
volatile uint16_t string_down_confirm_grace_n     = 50;  // ignore the confirm check this many polls after each retry
volatile uint8_t  string_down_confirm_max_retries = 3;   // give up and accept the drive's stall bit after this many

/* ---- Pair_TensionHome readbacks (Ozone) -------------------------------
   For the arm_state 13 "motor never stops" investigation. [0] = up node,
   [1] = down node of whichever pair is homing. th_active stays 1 while the
   seek loop runs: if it is still 1 after the slider has locked, the loop
   never saw a stop, and th_status / th_speed / th_pos show which signal
   failed to change. Speed and position are only read on the small-slider
   call (speed_fallback), so other homes keep their timing. */
volatile uint8_t  th_active    = 0;
volatile uint8_t  th_node[2]   = { 0, 0 };
volatile uint8_t  th_done[2]   = { 0, 0 };
volatile uint8_t  th_status[2] = { 0, 0 };   // last 0x6001 read (bit2 = stalled)
volatile int32_t  th_speed[2]  = { 0, 0 };   // last RT_SPEED (0x6030) read
volatile int32_t  th_pos[2]    = { 0, 0 };   // last motor position (0x600C) read
volatile uint32_t th_loops     = 0;          // seek-loop passes so far

/* Law of cosines - the tendon is a chord, not a wrap. */
float Joint_TendonLen(float phi_deg)
{
  float Rb = sqrtf(joint_Bx * joint_Bx + joint_By * joint_By);
  float c  = cosf(phi_deg * 3.1415926f / 180.0f);
  float v  = joint_Ra * joint_Ra + Rb * Rb - 2.0f * joint_Ra * Rb * c;
  return (v > 0.0f) ? sqrtf(v) : 0.0f;
}

static uint8_t Pair_TensionHome(uint8_t nu, uint8_t nd, uint32_t timeout_ms,
                                uint8_t speed_fallback, uint8_t confirm_down_stall)
{
  const uint8_t n[2] = { nu, nd };
  uint8_t  done[2]  = { 0, 0 };
  uint8_t  stuck[2] = { 0, 0 };   /* consecutive near-zero RT_SPEED polls, per motor (speed_fallback) */
  uint8_t  ok = 1;
  uint32_t t0 = HAL_GetTick();
  uint16_t poll = 0;              /* loop passes since start - grace window gate (speed_fallback) */

  /* Per-motor "is this stall real" state - only meaningful for
     n[i] == string_node_down when confirm_down_stall is set. */
  uint8_t  confirming[2]      = { 0, 0 };
  uint8_t  confirm_stuck[2]   = { 0, 0 };
  uint16_t confirm_poll[2]    = { 0, 0 };
  uint8_t  confirm_retries[2] = { 0, 0 };

  th_active  = 1;
  th_loops   = 0;
  th_node[0] = nu;  th_node[1] = nd;
  th_done[0] = 0;   th_done[1] = 0;

  Pair_Config(nu, nd, joint_home_current_ma, joint_home_thresh, 1u);
   for (int i = 0; i < 2; i++) {
    int32_t speed = (int32_t)joint_home_pps;

    if (n[i] == string_node_down || n[i] == string2_node_up) {
      speed = -speed;
    }

    (void)PUSI_SetSpeed(n[i], speed);
  }

  /* the two will not go taut at the same moment */
  while (!(done[0] && done[1])) {
    for (int i = 0; i < 2; i++) {
      if (done[i]) continue;

      uint8_t is_confirm_node = confirm_down_stall && (n[i] == string_node_down);

      uint8_t st;
      if (PUSI_ReadStatus(n[i], &st) == HAL_OK) {
        th_status[i] = st;
        if (PUSI_ST_STALLED(st)) {
          if (is_confirm_node && confirm_retries[i] < string_down_confirm_max_retries) {
            /* Possibly premature - clear the fault and keep pulling. This
               node is only accepted as done once RT_SPEED confirms it,
               below - a repeat stall trip just resets this same retry. */
            PUSI_ClearStall(n[i]);
            int32_t speed = (int32_t)joint_home_pps;
            if (n[i] == string_node_down || n[i] == string2_node_up) speed = -speed;
            (void)PUSI_SetSpeed(n[i], speed);
            confirming[i]      = 1;
            confirm_stuck[i]   = 0;
            confirm_poll[i]    = 0;
            confirm_retries[i]++;
          } else {
            PUSI_Stop(n[i]);
            (void)PUSI_SdoWrite(n[i], PUSI_OBJ_SPEED, 0, 0u, 4);
            done[i] = 1;
          }
        } else {
          t0 = HAL_GetTick();
        }
      }
      if (done[i]) continue;

      /* Fallback (small slider only - see string2_stall_* above): the
         drive's own stall bit isn't reliable here, so also declare stalled
         if RT_SPEED sits near zero for string2_stall_stuck_n polls in a
         row, once past the startup grace window. */
      if (speed_fallback) {
        int32_t pos;
        if (PUSI_SdoRead(n[i], PUSI_OBJ_MOTOR_POS, 0, &pos) == HAL_OK) th_pos[i] = pos;
      }
      if (speed_fallback) {
        int32_t spd;
        if (PUSI_ReadSpeed(n[i], &spd) == HAL_OK) {
          th_speed[i] = spd;
          if (spd < 0) spd = -spd;
          if (poll < string2_stall_grace_n) {
            /* still inside the startup grace window - readback only */
          } else if (spd < string2_stall_speed_pps) {
            if (stuck[i] < 255u) stuck[i]++;
            if (stuck[i] >= string2_stall_stuck_n) {
              PUSI_Stop(n[i]);
              (void)PUSI_SdoWrite(n[i], PUSI_OBJ_SPEED, 0, 0u, 4);
              done[i] = 1;
            }
          } else {
            stuck[i] = 0;
          }
        }
      }
      if (done[i]) continue;

      /* Confirm phase (down node only - see string_down_confirm_* above):
         waiting to see if this retry ends in a genuine, sustained stop. */
      if (is_confirm_node && confirming[i]) {
        if (confirm_poll[i] < 0xFFFFu) confirm_poll[i]++;
        if (confirm_poll[i] >= string_down_confirm_grace_n) {
          int32_t spd;
          if (PUSI_ReadSpeed(n[i], &spd) == HAL_OK) {
            if (spd < 0) spd = -spd;
            if (spd < string_down_confirm_speed_pps) {
              if (confirm_stuck[i] < 255u) confirm_stuck[i]++;
              if (confirm_stuck[i] >= string_down_confirm_stuck_n) {
                PUSI_Stop(n[i]);
                (void)PUSI_SdoWrite(n[i], PUSI_OBJ_SPEED, 0, 0u, 4);
                done[i] = 1;
              }
            } else {
              confirm_stuck[i] = 0;
            }
          }
        }
      }
    }

    th_done[0] = done[0];  th_done[1] = done[1];
    th_loops++;

    if ((HAL_GetTick() - t0) > timeout_ms) {
      ok = 0;
      break;
    }
    if (poll < 0xFFFFu) poll++;
    HAL_Delay(10);
  }
  th_active = 0;

  for (int i = 0; i < 2; i++) {
    PUSI_Stop(n[i]);
    (void)PUSI_SdoWrite(n[i], PUSI_OBJ_SPEED,     0, 0u, 4);
    (void)PUSI_SdoWrite(n[i], PUSI_OBJ_WORK_MODE, 0, PUSI_MODE_POSITION, 1);
  }
  Pair_Config(nu, nd, string_current_ma, spool_stall_thresh, string_stall_stop);
  for (int i = 0; i < 2; i++) {
    (void)PUSI_SdoWrite(n[i], PUSI_OBJ_MOTOR_POS, 0, 0u, 4);   /* zero here */
    PUSI_ClearStall(n[i]);
  }
  return ok;
}

/* Reel BOTH spools in until each stalls on a taut tendon, zero there.
   The one place stall detection is wanted ON; restored afterwards.   */
void Joint_HomeSpools(void)
{
  uint32_t t0 = HAL_GetTick();
  joint_home_state = 0;
  joint_homed      = 0;

  uint8_t ok = Pair_TensionHome(string_node_up, string_node_down,
                                joint_home_timeout_ms, 0, 0);
  joint_home_ms = HAL_GetTick() - t0;

  string_paid_mm   = 0.0f;
  string_init_done = 1;
  joint_phi_deg    = joint_phi_home_deg;
  joint_X_mm       = Joint_TendonLen(joint_phi_deg);
  joint_home_state = ok ? 1u : 2u;
  joint_homed      = ok;
}

/* TASK 1 - arm_state 1. Tension-home BOTH pairs so every tendon is in a
   known physical state before reaching starts. Sequential, not parallel:
   two pairs stalling at once draws heavy current and the seek loops would
   contend for the bus. Blocks up to 2 x joint_home_timeout_ms.        */
void String_InitBothPairs(void)
{
  /* confirm_down_stall=1: node 4 was observed tripping its stall bit early
     here, tilting the slider - see string_down_confirm_* above. */
  (void)Pair_TensionHome(string_node_up,  string_node_down,  joint_home_timeout_ms, 0, 1);
  (void)Pair_TensionHome(string2_node_up, string2_node_down, joint_home_timeout_ms, 0, 0);

  string_paid_mm    = 0.0f;     /* taut = the payout reference is zero */
  string2_paid_mm   = 0.0f;
  string_init_done  = 0;        /* force a fresh config on first move  */
  string2_init_done = 0;
  Base_CompReset();             /* all four tendons taut at this base angle */
}

/* TASK 2 - arm_state 10. Initialise nodes 3/4, pull them taut, then reset
   every piece of position bookkeeping so the next round starts here.  */
void Inite_joint_forming(void)
{
  String_PmcInit();
  (void)Pair_TensionHome(string_node_up, string_node_down, joint_home_timeout_ms, 0, 0);

  string_paid_mm = 0.0f;
  joint_phi_deg  = joint_phi_home_deg;
  joint_X_mm     = Joint_TendonLen(joint_phi_deg);
  Base_CompReset();             /* base compensation starts fresh too */
}

/* TASK 3 - end-of-round small-slider init. Tension-home nodes 5/6 ONLY:
   pull back until each side stalls (a stall on one does not stop the
   other - it keeps pulling until it stalls too), then clear stall/error
   status on both. Call this AFTER Inite_joint_forming() - the small slider
   pulls back until it physically locks against the (already taut)
   joint-forming slider, so 3/4 must be tensioned first. */
void init_small_slider(void)
{
  (void)Pair_TensionHome(string2_node_up, string2_node_down, joint_home_timeout_ms, 1, 0);

  string2_paid_mm   = 0.0f;     /* taut = the payout reference is zero */
  string2_init_done = 0;        /* force a fresh config on first move  */
  Base_CompSyncS2();            /* 5/6 are now taut at the current base angle */
}

static void Joint_Differential(float motor_deg, int8_t pull);

/* Differential move: one tendon pulls in while the other releases, which
   bends the joint instead of paying out slack. dir 0 = UP, 1 = DOWN.
   Travel comes from the pose-dependent tendon length, so joint_phi_deg
   must track - the same angle costs different tendon at different poses.

   joint_phi_sense maps dir onto the sign of dphi. Bench-set to -1: with +1
   the arm moved DOWN on dir 0. Flip it in Ozone if the arm ever bends the
   wrong way again - no rebuild needed. Note this is NOT a pure sign flip of
   the motor command: the tendon arc is asymmetric, so the OTHER direction
   needs a different amount of travel for the same joint angle.          */
void Joint_Move(float joint_deg, int dir)
{
  float jd = (joint_deg < 0.0f) ? -joint_deg : joint_deg;
  if (jd < 0.01f) { joint_skip = 1; return; }        /* nothing commanded */

  joint_cmd_deg = jd * joint_gain + joint_slack_deg;
  float dphi    = ((dir == 0) ? joint_cmd_deg : -joint_cmd_deg)
                  * (float)joint_phi_sense;

  float phi_b = joint_phi_deg + dphi;

  /* SOFTWARE LIMITS. dL/dphi is proportional to sin(phi), so the slope
     changes sign at 0 and 180 deg. Past a limit, the same dir produces the
     OPPOSITE pull and the joint reverses. Clamp the pose, and refuse a
     command that only pushes further into a limit already reached.      */
  if (phi_b > joint_phi_max_deg) phi_b = joint_phi_max_deg;
  if (phi_b < joint_phi_min_deg) phi_b = joint_phi_min_deg;

  float travel = phi_b - joint_phi_deg;
  if (travel < 0.001f && travel > -0.001f) {  /* already at that limit */
    joint_limit_hit = 1;
    joint_skip      = 4;
    return;
  }
  joint_limit_hit = (phi_b >= joint_phi_max_deg - 0.001f ||
                     phi_b <= joint_phi_min_deg + 0.001f) ? 1u : 0u;

  joint_dX_mm = Joint_TendonLen(phi_b) - Joint_TendonLen(joint_phi_deg);
  joint_X_mm  = Joint_TendonLen(phi_b);

  float adx = (joint_dX_mm < 0.0f) ? -joint_dX_mm : joint_dX_mm;
  joint_motor_deg = (adx / r_motor) * 180.0f / 3.1415926f;
  int8_t pull = (joint_dX_mm < 0.0f) ? 1 : -1;       /* shorter tendon = pull in */
  joint_phi_deg = phi_b;                             /* track the new pose */

  joint_skip = 0;
  joint_moves++;
  Joint_Differential(joint_motor_deg, pull);
}

/* Issue the differential pair move: one spool pulls in by motor_deg while the
   other releases the same amount. pull = +1 means the UP spool takes the
   pull-in sense. Shared by Joint_Move and the base-rotation compensation.  */
static void Joint_Differential(float motor_deg, int8_t pull)
{
  uint32_t steps = (uint32_t)((motor_deg / 360.0f) * (float)string_steps_per_rev + 0.5f);
  if (steps == 0u) { joint_skip = 2; return; }

  if (!string_init_done) String_PmcInit();

  /* Both dirs default CW (bench-confirmed). The spools are mounted mirrored,
     so equal commanded signs already give opposite physical rotation.     */
  int8_t su = (((float)pull * (float)joint_dir_up)   >= 0.0f) ? 1 : -1;
  int8_t sd = (((float)pull * (float)joint_dir_down) >= 0.0f) ? 1 : -1;

  joint_spd_up   = (int32_t)joint_speed_pps * su;
  joint_spd_down = (int32_t)joint_speed_pps * sd;

  PUSI_ClearStall(string_node_up);
  PUSI_ClearStall(string_node_down);

  joint_err_up   = (uint8_t)PUSI_MoveRel(string_node_up, steps,
                                         (su > 0) ? PUSI_DIR_FORWARD : PUSI_DIR_REVERSE,
                                         joint_spd_up, joint_accel, joint_decel);
  joint_err_down = (uint8_t)PUSI_MoveRel(string_node_down, steps,
                                         (sd > 0) ? PUSI_DIR_FORWARD : PUSI_DIR_REVERSE,
                                         joint_spd_down, joint_accel, joint_decel);
}

/* ---- base-rotation tendon compensation -------------------------------
   Rotating the base swings each tendon anchor P2 around a circle of radius
   base_R about the rotation centre, while the far anchor P1 stays fixed.
   The tendon is the straight 3D line between them, so its length changes
   without the joint being commanded: one side goes taut, the other slack.

   P1 = (offx, y1, z1) and P2(th) = (0, R cos th, R sin th). Expanding the
   distance and collapsing y1*cos + z1*sin into a single cosine gives

     L(th)^2 = offx^2 + rho^2 + R^2 - 2*rho*R*cos(th - psi)

   i.e. the law of cosines plus a constant for the out-of-plane offset,
   where rho = hypot(y1,z1) is P1 projected into P2's plane and psi is its
   angle there. Constants from the CAD:
     R = 65.46, P1 = (62, 72.5, 63) -> rho = 96.05, psi = 40.98 deg.

   The two spools sit at DIFFERENT th0 on the same circle, so their length
   changes are not equal and opposite - each gets its own correction.
   Note dL is NOT monotonic in base angle: it peaks where th - psi = 180
   (about 52.7 deg of base rotation for th0 = 168.3) and falls after, which
   is why a linear coupling factor cannot work.

   joint_phi_deg is NOT advanced - the joint itself has not rotated.     */
volatile uint8_t  joint_base_comp    = 1;        /* 0 = disable            */
volatile float    base_R             = 65.46f;   /* P2 circle radius [mm]  */
volatile float    base_rho           = 96.05f;   /* P1 in P2's plane [mm]  */
volatile float    base_psi_deg       = 40.98f;   /* angle of that projection */
volatile float    base_offx          = 62.0f;    /* out-of-plane offset [mm] */
/* Both anchors are 158.07 mm from P1 at base 0, so both are roots of
   L(th) = 158.07 - they are not competing answers for one anchor.
     168.3 deg -> path GROWS with base rotation -> node 4 goes taut
     -86.4 deg -> path SHRINKS                  -> node 3 goes slack
   which is exactly the bench observation. Verify by measuring node 3's
   tendon at base 0: if it is not also ~158 mm, its th0 is a separate
   unknown and -86.4 is only a mathematical artefact.                   */
volatile float    base_th0_up        = -86.4f;   /* node 3 anchor at base 0 */
volatile float    base_th0_down      = 168.3f;   /* node 4 anchor at base 0 */
volatile int8_t   joint_base_sense   = +1;       /* flip if it corrects backwards */

/* Nodes 5/6 (section-forming slider tendons) cross the same base joint. No
   CAD anchor angles for them yet, so they default to the 3/4 values (5
   routed like 3, 6 like 4) - replace with the measured ones. */
volatile float    base_th0_s2_up     = -86.4f;   /* node 5 anchor at base 0 */
volatile float    base_th0_s2_down   = 168.3f;   /* node 6 anchor at base 0 */

/* Base angle the tendons were last tensioned at. Length changes are
   measured from here, not from base 0, so a home done with the base parked
   a few degrees off 0 does not leave a built-in error. */
volatile float    base_ref_deg       = 0.0f;
volatile float    joint_base_deg     = 0.0f;     /* base angle applied so far */
volatile float    base_need_up       = 0.0f;     /* length actually moved so far [mm] */
volatile float    base_need_down     = 0.0f;
volatile float    base_need_s2_up    = 0.0f;
volatile float    base_need_s2_down  = 0.0f;
volatile float    base_do_up         = 0.0f;     /* owed this pass [mm] */
volatile float    base_do_down       = 0.0f;
volatile float    base_do_s2_up      = 0.0f;
volatile float    base_do_s2_down    = 0.0f;
volatile uint32_t joint_base_moves   = 0;

/* Straight-line tendon length for an anchor at th [deg] on the circle. */
float Base_TendonLen(float th_deg)
{
  float c = cosf((th_deg - base_psi_deg) * 3.1415926f / 180.0f);
  float v = base_offx * base_offx + base_rho * base_rho + base_R * base_R
            - 2.0f * base_rho * base_R * c;
  return (v > 0.0f) ? sqrtf(v) : 0.0f;
}

/* Total length change a tendon needs at base_deg, relative to base_ref_deg. */
static float Base_Owed(float th0, float base_deg)
{
  float s = (float)joint_base_sense;
  return Base_TendonLen(th0 + base_deg * s) - Base_TendonLen(th0 + base_ref_deg * s);
}

/* Current base angle, or 0 if the servo feedback is not arriving - without
   it sdm_pos_user_deg is just the zero offset (~92 deg), not an angle. */
static float Base_NowDeg(void)
{
  SDM_Motor_t *sm = SDM_GetMotor(sdm_id);
  if (sm != NULL && (HAL_GetTick() - sm->last_rx_tick) < 200u) return sdm_pos_user_deg;
  return 0.0f;
}

/* All four tendons have just been tensioned at the current base angle. */
void Base_CompReset(void)
{
  float now = Base_NowDeg();
  base_ref_deg      = now;
  joint_base_deg    = now;
  base_need_up      = 0.0f;
  base_need_down    = 0.0f;
  base_need_s2_up   = 0.0f;
  base_need_s2_down = 0.0f;
}

/* Only nodes 5/6 were just tensioned: count them as already compensated for
   the current angle, so the next move does not re-apply that change. */
void Base_CompSyncS2(void)
{
  float now = Base_NowDeg();
  base_need_s2_up   = Base_Owed(base_th0_s2_up,   now);
  base_need_s2_down = Base_Owed(base_th0_s2_down, now);
}

/* Move ONE spool. + = pay out, - = reel in. Returns the length the drive
   actually accepted (step-quantised), 0 if nothing was sent or it refused. */
static float Spool_MoveOne(uint8_t node, int8_t dir, float length_mm,
                           volatile uint8_t *err, volatile int32_t *spd)
{
  float d   = (length_mm < 0.0f) ? -length_mm : length_mm;
  float deg = (d / r_motor) * 180.0f / 3.1415926f;
  if (deg < 0.05f) return 0.0f;

  uint32_t steps = (uint32_t)((deg / 360.0f) * (float)string_steps_per_rev + 0.5f);
  if (steps == 0u) return 0.0f;

  float  sgn = (length_mm >= 0.0f) ? 1.0f : -1.0f;
  int8_t s   = ((sgn * (float)dir) >= 0.0f) ? 1 : -1;
  *spd = (int32_t)string_speed_pps * s;

  PUSI_ClearStall(node);
  *err = (uint8_t)PUSI_MoveRel(node, steps,
                               (s > 0) ? PUSI_DIR_FORWARD : PUSI_DIR_REVERSE,
                               *spd, string_accel, string_decel);
  if (*err != (uint8_t)HAL_OK) return 0.0f;

  float moved = ((float)steps / (float)string_steps_per_rev) * 6.2831853f * r_motor;
  return sgn * moved;
}

/* One tendon: move whatever is still owed at base_deg. need tracks what the
   drive actually accepted, so a refused move (still busy) is retried on the
   next call instead of being lost. Returns 1 if nothing is left owed. */
static uint8_t Base_CompOne(uint8_t node, int8_t dir, float th0, float base_deg,
                            volatile float *need, volatile float *owed,
                            volatile uint8_t *err, volatile int32_t *spd)
{
  *owed  = Base_Owed(th0, base_deg) - *need;
  float sent = Spool_MoveOne(node, dir, *owed, err, spd);
  *need += sent;
  float left = *owed - sent;
  return (left < 0.05f && left > -0.05f);
}

/* Path grew -> pay that spool out; shrank -> reel it in. Payout senses are
   the same bench-checked string_dir_* / string2_dir_* used by the fill, so a
   positive length always RELEASES tendon. */
void Joint_CompensateBase(float base_deg)
{
  if (!joint_base_comp) { joint_base_deg = base_deg; return; }

  float d = base_deg - joint_base_deg;
  if (d < 0.01f && d > -0.01f) return;             /* already compensated */

  if (!string_init_done) String_PmcInit();
  uint8_t ok = 1;
  ok &= Base_CompOne(string_node_up,   string_dir_up,   base_th0_up,   base_deg,
                     &base_need_up,   &base_do_up,   &string_err_up,   &string_spd_up);
  ok &= Base_CompOne(string_node_down, string_dir_down, base_th0_down, base_deg,
                     &base_need_down, &base_do_down, &string_err_down, &string_spd_down);

  if (string2_enable) {
    if (!string2_init_done) String2_Init();
    ok &= Base_CompOne(string2_node_up,   string2_dir_up,   base_th0_s2_up,   base_deg,
                       &base_need_s2_up,   &base_do_s2_up,   &string2_err_up,   &string2_spd_up);
    ok &= Base_CompOne(string2_node_down, string2_dir_down, base_th0_s2_down, base_deg,
                       &base_need_s2_down, &base_do_s2_down, &string2_err_down, &string2_spd_down);
  }

  if (ok) joint_base_deg = base_deg;               /* else retry next call */
  joint_base_moves++;
}

/* ==================================================================== */
/*  GRASPING                                                            */
/* ==================================================================== */
volatile uint8_t  grasp_node_up       = 7;
volatile uint8_t  grasp_node_down     = 8;
volatile int8_t   grasp_dir_up        = PUSI_CCW;  /* +1: the sense that CLOSES */
volatile int8_t   grasp_dir_down      = PUSI_CCW;  /* +1 */
volatile int32_t  grasp_pps           = 600;
volatile uint16_t grasp_current_ma    = 800;     // grip force - tune this first
volatile uint16_t grasp_thresh        = 12;
volatile uint16_t grasp_steps_per_rev = 4000;
volatile uint16_t grasp_accel         = 5;
volatile uint16_t grasp_decel         = 5;
volatile uint8_t  grasp_stop_apply    = 0;   // Ozone one-shot: stop now
volatile uint8_t  grasp_state         = 0;   // 0 idle/held, 3 closing

/* Close and return immediately. Nothing ends this but Grasp_StopNow() - the
   stall IS the grip, and the drive holds torque once it stops turning, so
   there is no position target and nothing to wait for. Stall detection is
   left OFF (0x601B = 0) so the drive cannot halt itself part-way.        */
void Grasp_Start(void)
{
  const uint8_t n[2] = { grasp_node_up, grasp_node_down };
  const int8_t  s[2] = { grasp_dir_up,  grasp_dir_down  };
  for (int i = 0; i < 2; i++) {
    PUSI_ClearStall(n[i]);
    PUSI_SetMicrostep(n[i], spool_microstep);
    PUSI_SetRunCurrent(n[i], grasp_current_ma);   /* grip force lives here */
    PUSI_ConfigStall(n[i], grasp_thresh, 0);
    PUSI_EnableMotor(n[i], 1);
  }
  for (int i = 0; i < 2; i++)
    (void)PUSI_SetSpeed(n[i], (int32_t)grasp_pps * ((s[i] >= 0) ? 1 : -1));
  grasp_state = 3;
}

void Grasp_StopNow(void)
{
  const uint8_t n[2] = { grasp_node_up, grasp_node_down };
  for (int i = 0; i < 2; i++) {
    PUSI_Stop(n[i]);                                            /* 0x6020 = 0 */
    (void)PUSI_SdoWrite(n[i], PUSI_OBJ_SPEED,     0, 0u, 4);
    (void)PUSI_SdoWrite(n[i], PUSI_OBJ_WORK_MODE, 0, PUSI_MODE_POSITION, 1);
    PUSI_EnableMotor(n[i], 1);                                  /* hold the grip */
  }
  grasp_state = 0;
}

/* Move both gripper spools by a fixed angle. POSITIVE closes, NEGATIVE opens.
   Finite move, so it stops itself at the commanded angle - unlike
   Grasp_Start(), which runs until something stops it.

   The drive is forced back to POSITION mode first: 0x6004 relative moves are
   ignored while it is in speed mode, which is exactly the state a previous
   Grasp_Start() leaves it in.                                            */
void Grasp_MoveBy(float degrees)
{
  const uint8_t n[2] = { grasp_node_up, grasp_node_down };
  const int8_t  s[2] = { grasp_dir_up,  grasp_dir_down  };
  float d = (degrees < 0.0f) ? -degrees : degrees;
  uint32_t steps = (uint32_t)((d / 360.0f) * (float)grasp_steps_per_rev + 0.5f);
  if (steps == 0u) return;

  int8_t sense = (degrees >= 0.0f) ? 1 : -1;   /* +1 closes, -1 opens */

  for (int i = 0; i < 2; i++) {
    PUSI_Stop(n[i]);                                            /* leave speed mode */
    (void)PUSI_SdoWrite(n[i], PUSI_OBJ_SPEED,     0, 0u, 4);
    (void)PUSI_SdoWrite(n[i], PUSI_OBJ_WORK_MODE, 0, PUSI_MODE_POSITION, 1);
    PUSI_ClearStall(n[i]);
    PUSI_SetMicrostep(n[i], spool_microstep);
    PUSI_SetRunCurrent(n[i], grasp_current_ma);
    PUSI_ConfigStall(n[i], grasp_thresh, 0);    /* ignore stall: finite move */
    PUSI_EnableMotor(n[i], 1);
  }
  for (int i = 0; i < 2; i++) {
    int8_t op = ((s[i] >= 0) ? 1 : -1) * sense;
    (void)PUSI_MoveRel(n[i], steps,
                       (op > 0) ? PUSI_DIR_FORWARD : PUSI_DIR_REVERSE,
                       (int32_t)grasp_pps * op, grasp_accel, grasp_decel);
  }
  grasp_state = 0;
}

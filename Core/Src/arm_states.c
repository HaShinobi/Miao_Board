// file: arm_states.c
// One function per arm_state, so the dispatch switch reads as a list of what
// the arm can do. Each runs once per main-loop pass while its state is
// selected; pullback_started distinguishes "first pass here" from "still here".

#include "main.h"
#include "arm_states.h"
#include "arm_control.h"
#include "pump_control.h"
#include "pump_application_function.h"
#include "pressure_sensor.h"
#include "position_control.h"
#include "sdm8070.h"

/* hfdcan1, program_state, pump_state, volume_pumped_mL, r_motor: see main.h */

/* ==================================================================== */
/*  COMMANDS                                                            */
/* ==================================================================== */
volatile uint8_t arm_state      = 0;
volatile uint8_t last_arm_state = 0;

volatile float   joint_degree      = 0.0f;
volatile uint8_t dir               = 0;      // 0 = up, 1 = down
volatile float   grasping_degree   = 0.0f;

volatile float   arm7_target_deg = 0.0f;
volatile int16_t arm7_speed_rpm  = 2;
volatile float   arm7_tol_deg    = 1.5f;
/* Base-compensation trigger: the servo counts as arrived once its measured
   angle changes by less than arm7_settle_deg for arm7_settle_n consecutive
   passes. Independent of WHERE it parked - see ArmState_ServoAngle. */
volatile float   arm7_settle_deg = 0.2f;
volatile uint8_t arm7_settle_n   = 3;
/* Release the slider tendons every time the base has turned this far,
   while it is still turning - not all at once after it stops. */
volatile float   arm7_sync_deg   = 1.0f;

/* Post-home move to the 0 deg ready position lands short, so aim this many
   degrees further in the direction of travel (away from the hard stop).
   Negative pulls it back the other way. */
volatile float   home_ready_extra_deg = 4.0f;

/* state 14: section-forming slider init under pressure control. Each cycle
   holds the chamber at sf_target_kpa, drains sf_q_ml, then reels nodes 5/6
   in by Q / PC_A_EFF, until sf_total_mm has been pulled back. */
volatile float    sf_target_kpa   = 30.0f;   // chamber pressure held throughout [kPa]
volatile float    sf_band_kpa     = 2.0f;    // +/- band counted as "at target"
volatile uint8_t  sf_settle_n     = 3;       // in-band passes before each drain
volatile float    sf_min_kpa      = 20.0f;   // end a drain early below this
volatile float    sf_q_ml         = 10.0f;   // Q: water drained per cycle [mL]
volatile float    sf_total_mm     = 30.0f;   // total pull-back; 30 mm = the first test
volatile uint16_t sf_fill_duty    = 150;     // PumpIn duty when below the band
volatile uint16_t sf_drain_duty   = 300;     // PumpOut duty, regulating and draining
volatile uint32_t sf_drain_max_ms = 10000;   // give up on a drain that never sees Q
volatile uint8_t  sf_phase        = 0;       // readback: 0 regulate, 1 drain, 2 reel, 3/4 done
volatile uint8_t  sf_status       = 0;       // readback: 0 running, 1 done, 2 reel jammed, 3 no drain flow
volatile uint16_t sf_cycle        = 0;       // readback: cycles completed
volatile float    sf_done_mm      = 0.0f;    // readback: pulled back so far [mm]
volatile float    sf_drained_ml   = 0.0f;    // readback: drained this cycle [mL]

volatile float   x_target_mm    = 0.0f;
volatile float   x_cmd_scale    = 0.002492f;
volatile float   x_cmd_offset_m = 0.1323f;

volatile uint8_t pc_str_enable   = 1;
volatile float   pc_str_gain     = 1.0f;
volatile float   pc_str_offset_m = -0.030f;
volatile float   pos_true_raw    = 0.0f;

volatile uint8_t grasping_state      = 0;   // 0 stop, 1 close, 2 open
volatile uint8_t last_grasping_state = 0;

/* One-shot latch: 0 = this state has not started yet. States 3/6 park it at
   1/2 so a re-entry does not re-fire the move. */
static uint8_t pullback_started = 0;
static uint8_t coast_loops      = 0;   // coast-down loops left after the pump stops
#define COAST_WINDOW  11               // ~2.2 s; lower if overshooting, raise if under

/* ---- pre-payout pressure drop (arm_state 6/7) --------------------------
   Bleed the chamber down to a low band right before a tendon payout move
   on nodes 3/4, so that move isn't fighting full working pressure. State 6
   calls this before its rotation (which IS the payout move); state 7 calls
   it after the base servo has settled, right before the tendon correction
   that follows the swing.
   Drains for up to pm_predrop_ms total: pumps out while above
   pm_predrop_hi_kpa, holds (pump off) once inside the band or below it.
   Always returns once the window elapses and lets the move proceed
   either way - this is a settling window, not a hard gate, so a stuck or
   slow-draining system can't strand the sequence here. Disabling it
   (pm_predrop_enable = 0) falls back to the normal pressure hold. */
volatile uint8_t  pm_predrop_enable = 1;
volatile float    pm_predrop_lo_kpa = 20.0f;
volatile float    pm_predrop_hi_kpa = 25.0f;
volatile uint16_t pm_predrop_duty   = 300;     // PumpOut duty while draining
volatile uint32_t pm_predrop_ms     = 2000;    // total window before the move fires

static void PreMove_PressureDrop(void)
{
  if (!pm_predrop_enable) { Sensor_PressureControl(); return; }

  uint32_t t0 = HAL_GetTick();
  while ((HAL_GetTick() - t0) < pm_predrop_ms) {
    Sensor_ReadOnce();
    if (pressure_value > pm_predrop_hi_kpa) PumpOut(pm_predrop_duty);
    else                                    PumpStopAll();
    HAL_Delay(50);
  }
  PumpStopAll();
}

/* Non-blocking, one-pass version of the same drop logic, for continuous
   per-pass use while the servo is still moving (arm_state 7 - see
   ArmState_ServoAngle): pump out while above pm_predrop_hi_kpa, hold once
   at/below it. Same threshold/duty knobs as PreMove_PressureDrop, just
   applied one pass at a time instead of as a blocking wait, since this
   runs every loop for as long as the base keeps moving. */
static void PressureDrop_Step(void)
{
  if (!pm_predrop_enable) { Sensor_PressureControl(); return; }

  Sensor_ReadOnce();
  if (pressure_value > pm_predrop_hi_kpa) PumpOut(pm_predrop_duty);
  else                                    PumpStopAll();
}

/* ==================================================================== */
/*  ARM STATES                                                          */
/* ==================================================================== */

/* 1 - Big servo home. The SDM8070 home is fired by Arm_StartupHoming()
       from main.c and run by SDM_ServoTask(), so nothing is needed here -
       this state exists only so the sequence can select "home the big
       servo" on its own, separate from the tendon pairs (state 2). */
void ArmState_SDMHome(void)
{
}

/* 2 - Tendon init. Tension-homes BOTH tendon pairs - nodes 3/4 and 5/6 -
       so every tendon is in a known physical state before reaching
       begins. Blocks while it seeks. */
void ArmState_Home(void)
{
  if (!pullback_started) {
    String_InitBothPairs();
    pullback_started = 1;
  }
}

/* 2 - Fill: pump water in, track slider position from the flow integral,
       and pay string out so a slack margin always leads the slider. */
void ArmState_Fill(void)
{
  if (!pullback_started) {
    PositionControl_Init();
    string_paid_mm         = 0.0f;   /* string reference zero = start of fill */
    string2_paid_mm        = 0.0f;
    string_init_done       = 0;      /* re-configure + enable the spool nodes */
    string2_init_done      = 0;
    string_prewind_pending = 1;      /* owe the initial one-revolution unwind  */
    pullback_started       = 1;
    coast_loops            = 0;
  }

  PumpIn(400);
  volume_pumped_mL += PositionControl_TrackFlowOnly();
  pos_true_raw = PositionControl_TruePosFromVolume(volume_pumped_mL);
  pos_true     = pos_true_raw;

  /* String attached -> chamber runs loaded -> the raw model over-reads. */
  if (pc_str_enable && string_enable) {
    pos_true = pos_true_raw * pc_str_gain + pc_str_offset_m;
    if (pos_true < 0.0f) pos_true = 0.0f;
  }

  if (!string_enable) return;
  string_pos_mm = pos_true * 1000.0f;                 /* m -> mm */

  /* Both spool pairs pay out the same length: nodes 3/4 and nodes 5/6.
     Pair 1 only ever pays out - the reel-in in states 5 and 10 is pair 0. */
  if (string_prewind_pending) {
    /* free slack dump: one full spool revolution (78.54 mm at r = 12.5) */
    string_prewind_mm = string_prewind_rev * 6.2831853f * r_motor;
    if (string_prewind_mm > 0.05f) {
      String_Move (string_prewind_mm);
      String2_Move(string_prewind_mm);
      string_paid_mm  += string_prewind_mm;
      string2_paid_mm += string_prewind_mm;
    }
    string_prewind_pending = 0;
  } else {
    /* hold string_excess_mm of slack ahead of the slider */
    float delta_mm = (string_pos_mm + string_excess_mm) - string_paid_mm;
    if (delta_mm >= string_step_mm) {
      String_Move (delta_mm);
      String2_Move(delta_mm);
      string_paid_mm  += delta_mm;
      string2_paid_mm += delta_mm;
    }
  }
}

/* 3 - Grasp on the PUSIROBOT spools. grasping_degree >= 0 closes (the
       stall IS the grip), < 0 opens by that angle. Hands off to hold. */
void ArmState_Grasp(void)
{
  Sensor_PressureControl();
  if (!pullback_started) {
    /* Non-zero = finite move to that angle (+ closes, - opens).
       Zero = close continuously until grasp_stop_apply. */
    if (grasping_degree != 0.0f) Grasp_MoveBy(grasping_degree);
    else                         Grasp_Start();
    pullback_started = 1;
  }
  arm_state = 9;
}

/* 4 - Release the slider assembly. DO NOT USE.
       The 180-degree release move ran on the old CanMessage_t drives, which
       are gone. There is no PUSIROBOT equivalent defined yet, so this state
       now only holds pressure and hands off. */
void ArmState_ReleaseAssembly(void)
{
  Sensor_PressureControl();
  pullback_started = 1;
  arm_state = 9;
}

/* 5 - Separate the big slider from the small one by reeling the string in.
       The string is in TENSION here, hence the chunked stall-proof pull.

       Pull-back length is derived from the P1(tendon)/P2(joint-forming)/
       O(base) geometry rather than the old flat "half the travel" guess -
       see the comment above sep_delta_L_mm in arm_control.c for the full
       derivation. string_reel_extra_mm stays on top as a small bench
       margin; set it to 0 if none is wanted. */
void ArmState_SeparateSliders(void)
{
  Sensor_PressureControl();
  if (pullback_started == 1) return;

  if (string_enable) {
    /* pos_true is the value latched by arm_state 2 - not recomputed here */
    string_sep_pos_mm = pos_true * 1000.0f;                          // X

    sep_geometric_pull_mm = (string_sep_pos_mm + sep_L0_mm) * 0.5f
                           - sep_delta_L_mm;                          // (X+L0)/2 - DeltaL
    sep_slack_mm           = string_paid_mm - string_sep_pos_mm;      // L_re - X

    string_reel_mm = sep_geometric_pull_mm + sep_slack_mm + string_reel_extra_mm;
    HAL_Delay((uint32_t)string_sep_delay_ms);
    String_ReelIn(string_reel_mm);
  }
  pullback_started = 1;
}

/* 6 - Joint rotation, differential on the same spools. Set
       joint_home_apply = 1 first to tension-home and set the pose
       reference; otherwise it rotates by joint_degree in dir.

       Before the actual rotation, PreMove_PressureDrop() bleeds the
       chamber down to pm_predrop_lo_kpa..pm_predrop_hi_kpa and holds it
       there for pm_predrop_ms, so the joint isn't rotating against full
       working pressure. */
void ArmState_JointRotate(void)
{
  if (pullback_started != 2) {
    if (joint_home_apply) {
      Sensor_PressureControl();
      Joint_HomeSpools();
      joint_home_apply = 0;
    } else {
      PreMove_PressureDrop();                    /* bleed to the low band, hold */
      Joint_Move(joint_degree, dir);             /* dir 0 = up, 1 = down */
      HAL_Delay(2000);
    }
  }
  pullback_started = 2;
  arm_state = 9;
}

/* 7 - Big servo to an absolute angle. Auto-moves whenever you change
       arm7_target_deg in Ozone; no apply flag.

       The slider tendons (nodes 3/4 AND 5/6) are compensated IN STEP with
       the rotation: every time the measured base angle has moved
       arm7_sync_deg past the last compensated angle, each tendon is paid
       out or reeled in by what the base geometry says it needs at that
       angle. Pressure is held LOW while the base moves (PressureDrop_Step)
       and back at the normal hold once it stops, with one final top-up for
       wherever it actually parked.

       Uses the MEASURED angle, never the target, and only while the 50 Hz
       feedback is arriving - with no feedback sdm_pos_user_deg is just the
       zero offset, and compensating for that would wind the tendons by
       ~90 deg worth of geometry. */
void ArmState_ServoAngle(void)
{
  static float   last_pos = 0.0f;
  static uint8_t still    = 0;

  SDM_MoveOnChange(arm7_target_deg, arm7_speed_rpm, arm7_tol_deg);

  float here  = sdm_pos_user_deg;
  float moved = here - last_pos;
  if (moved < 0.0f) moved = -moved;
  last_pos = here;

  if (moved < arm7_settle_deg) { if (still < 255u) still++; }
  else                         { still = 0u; }

  SDM_Motor_t *sm = SDM_GetMotor(sdm_id);
  uint8_t live = (sm != NULL) && ((HAL_GetTick() - sm->last_rx_tick) < 200u);
  float   lag  = here - joint_base_deg;
  if (lag < 0.0f) lag = -lag;

  if (still >= arm7_settle_n) {
    Sensor_PressureControl();                               /* arrived: normal hold  */
    if (live && lag >= arm7_settle_deg) Joint_CompensateBase(here);  /* final top-up */
  } else {
    PressureDrop_Step();                                    /* moving: hold low      */
    if (live && lag >= arm7_sync_deg)   Joint_CompensateBase(here);  /* release in step */
  }
}

/* 8 - Drain. Watch pos_phys here, not pos_true: draining does not add to
       volume_pumped_mL. */
void ArmState_Drain(void)
{
  PumpOut(400);
  PositionControl_TrackFlowOnly();
  pullback_started = 0;
}

/* 9 - Hold position and maintain pressure. States 3/4/6 fall through here. */
void ArmState_Hold(void)
{
  Sensor_PressureControl();
  pullback_started = 0;
}

/* 10 - Joint forming init: configure nodes 3/4, pull them taut, then reset
        every piece of position bookkeeping so the next round starts here. */
void ArmState_InitSlider(void)
{
  Sensor_PressureControl();
  if (!pullback_started) {
    Inite_joint_forming();
    pullback_started = 1;
  }
}

/* 12 - Closed-loop move to a commanded slider displacement. Set
        x_target_mm to the REAL displacement wanted [mm]; the calibration
        maps it to the control setpoint and the PI loop pumps to it.
        OWNS the pump, so no pressure control here. */
void ArmState_MoveToPosition(void)
{
  x_target = x_cmd_scale * x_target_mm + x_cmd_offset_m;
  PositionControl_Update();
}

/* 0 / default - everything off, with a coast window so the flow that keeps
                 moving after the pump stops is still counted. */
void ArmState_Idle(void)
{
  if (pullback_started != 0) {
    String_StopAll();          /* all four spools: stop and hold */
    pullback_started = 0;
    coast_loops = COAST_WINDOW;
  }
  PumpStopAll();
  if (coast_loops > 0) {
    volume_pumped_mL += PositionControl_TrackFlowOnly();   /* real coast-down flow */
    pos_true = PositionControl_TruePosFromVolume(volume_pumped_mL);
    coast_loops--;
  } else {
    PositionControl_TrackFlowOnly();   /* drain buffer only; sensor floor, not real flow */
  }
}

/* 13 - End-of-round small-slider init. Tension-homes nodes 5/6 ("small
        slider") ONLY, then zeroes every piece of position/volume
        bookkeeping - tendon payout, the PI controller state, and the
        volume/position integrals - so the next round starts exactly like a
        fresh trial, not carrying over anything from the one just finished.
        Meant to run AFTER the joint-forming slider (nodes 3/4) has already
        been tension-homed - e.g. by arm_state 10 - since the small slider
        pulls back until it physically locks against the already-taut
        joint-forming slider. Servo homing is its own state now (1), so it
        is not repeated here - sequence 1 -> 10 -> 13 for a full re-init.
        Same one-shot + pressure-hold shape as arm_state 10. */
void ArmState_InitSmallSlider(void)
{
  Sensor_PressureControl();
  if (!pullback_started) {
    init_small_slider();     // nodes 5/6: tension-home to lock against it

    /* Fresh-trial reset: every position/volume integral back to zero.
       string2_paid_mm is already zeroed inside init_small_slider(). */
    string_paid_mm   = 0.0f;
    PositionControl_Init();  // pos_estimate + PI integral/direction -> 0
    volume_pumped_mL = 0.0f;
    net_volume_mL    = 0.0f;
    pos_phys         = 0.0f;
    pos_true         = 0.0f;
    pos_true_raw     = 0.0f;

    pullback_started = 1;
  }
}

/* Drive the chamber toward sf_target_kpa from either side. 1 = in band.
   PumpStopAll (not PumpDisAll) so the valves close and the pressure holds. */
static uint8_t SF_RegulatePressure(void)
{
  if (pressure_value < sf_target_kpa - sf_band_kpa) { PumpIn(sf_fill_duty);   return 0; }
  if (pressure_value > sf_target_kpa + sf_band_kpa) { PumpOut(sf_drain_duty); return 0; }
  PumpStopAll();
  return 1;
}

/* 14 - Section-forming slider init under pressure control. Run with the
        joint-forming slider already docked (state 10 first).

        The chamber has to stay stiff the whole way, so instead of hauling
        the slider back against the water, each cycle makes room first:
          1. regulate to sf_target_kpa (pumps in or out) and let it settle;
          2. drain Q = sf_q_ml, measured by the flow sensor;
          3. reel nodes 5/6 in by Q / PC_A_EFF - the slider moving back
             takes up exactly the volume that was removed;
        repeated until sf_total_mm is pulled back. The last cycle drains
        only what the remaining distance needs. If pressure drops below
        sf_min_kpa mid-drain, the drain ends early and the reel uses the
        volume actually removed.

        sf_status: 1 done, 2 a reel did not finish (jammed or already
        locked), 3 a drain never saw flow. After it ends the chamber is
        kept at sf_target_kpa. Not stall-based: nodes 5/6 ignore stall
        while reeling here, so it stops on distance only - run state 13
        afterwards to pull the last bit taut and lock. */
void ArmState_SectionInit(void)
{
  static uint8_t  in_band = 0;
  static float    v_start = 0.0f;
  static float    q_cycle = 0.0f;
  static uint32_t t_drain = 0;

  (void)PositionControl_TrackFlowOnly();       /* keeps net_volume_mL current */

  if (!pullback_started) {
    sf_phase = 0;  sf_status = 0;  sf_cycle = 0;
    sf_done_mm = 0.0f;  sf_drained_ml = 0.0f;
    in_band = 0;
    if (!string2_enable) { sf_status = 2; sf_phase = 3; }
    else                 String2_ReelMode(1);
    pullback_started = 1;
  }

  switch (sf_phase) {
    case 0:                                    /* regulate, then start a drain */
      if (SF_RegulatePressure()) { if (in_band < 255u) in_band++; }
      else                       in_band = 0;
      if (in_band >= sf_settle_n) {
        float left_ml = (sf_total_mm - sf_done_mm) * PC_A_EFF * 1.0e3f;  /* mm -> mL */
        q_cycle       = (left_ml < sf_q_ml) ? left_ml : sf_q_ml;
        v_start       = net_volume_mL;
        t_drain       = HAL_GetTick();
        sf_drained_ml = 0.0f;
        in_band       = 0;
        sf_phase      = 1;
      }
      break;

    case 1:                                    /* drain Q */
      sf_drained_ml = v_start - net_volume_mL;
      if (sf_drained_ml >= q_cycle || pressure_value < sf_min_kpa) {
        PumpStopAll();
        if (sf_drained_ml < 0.5f) { sf_status = 3; sf_phase = 3; }
        else                      sf_phase = 2;
      } else if ((HAL_GetTick() - t_drain) > sf_drain_max_ms) {
        PumpStopAll();
        sf_status = 3;
        sf_phase  = 3;
      } else {
        PumpOut(sf_drain_duty);
      }
      break;

    case 2: {                                  /* reel in Q / A */
      float mm   = sf_drained_ml * 1.0e-3f / PC_A_EFF;                   /* mL -> mm */
      float left = sf_total_mm - sf_done_mm;
      if (mm > left) mm = left;
      if (!String2_ReelIn(mm)) { sf_status = 2; sf_phase = 3; break; }
      sf_done_mm += mm;
      sf_cycle++;
      if (sf_done_mm >= sf_total_mm - 0.05f) { sf_status = 1; sf_phase = 3; }
      else                                   sf_phase = 0;
      break;
    }

    default:                                   /* finished: stay stiff */
      if (sf_phase == 3) {
        if (string2_enable) String2_ReelMode(0);
        sf_phase = 4;
      }
      (void)SF_RegulatePressure();
      break;
  }
}

/* ==================================================================== */
/*  ENTRY POINTS                                                        */
/* ==================================================================== */

/* One-time SDM8070 homing. Fires when the arm sequence first reaches
   arm_state 1, waits ~1 s for the 50 Hz feedback to establish, homes into
   the hard stop, then moves to 0 deg as the ready position. Called every
   loop from main() in every program_state, so a mode switch mid-home does
   not strand the sequence. */
void Arm_StartupHoming(void)
{
  static uint8_t  startup_state = 0;   // 0 wait, 1 homing, 2 done
  static uint16_t startup_wait  = 0;

  if (startup_state == 0 && program_state == 1 && arm_state == 1) {
    if (++startup_wait >= 5) {         // ~1 s for feedback to establish
      sdm_home_skip_backoff = 1;       // native back-off unused; move to 0 handles it
      sdm_home_apply = 1;
      startup_state = 1;
    }
  } else if (startup_state == 1 && sdm_home_state == 4) {
    sdm_speed_rpm    = (uint16_t)arm7_speed_rpm;
    sdm_move_tol_deg = arm7_tol_deg;
    sdm_target_deg   = 0.0f - home_ready_extra_deg;   // ready position, plus overshoot
    sdm_move_hold    = 1;
    sdm_move_apply   = 1;
    startup_state = 2;                 // done: SDM_ServoTask holds it there
  }
}

/* Re-arm every latch after a program_state switch, so the next state runs
   its one-shot from a known starting point. */
void Arm_OnProgramStateChange(void)
{
  pullback_started    = 0;
  last_pump_state     = 0xFF;
  last_arm_state      = 0xFF;
}

/* Housekeeping run before the dispatch switch in main.c, every pass.

   On an arm_state CHANGE: start the pump coast window if dropping to idle,
   then clear pullback_started so the new state fires its one-shot once
   whichever state we came from. That is what makes 2->5 behave like 2->9->5.
   Nothing needs an explicit stop - every spool move is a finite position
   command, and the gripper is stopped through grasp_stop_apply.

   Every pass: service the Ozone gripper-stop one-shot. */
void Arm_BeginPass(void)
{
  if (arm_state != last_arm_state) {
    /* Nodes 5/6 pay out during state 2 and are used nowhere else, so they
       are stopped on EVERY state change - they must never be left turning
       once the fill ends. State 2 re-commands them on its next pass. */
    String2_Stop();

    if (pullback_started != 0 && arm_state == 0) {
      coast_loops = COAST_WINDOW;      // pump coast-down after leaving state 2
    }
    pullback_started = 0;
    last_arm_state = arm_state;
  }

  if (grasp_stop_apply) { Grasp_StopNow(); grasp_stop_apply = 0; }
}

/* Manual bench mode: pump and spools driven straight from Ozone, with no
   sequencing. Used for filling, draining and polarity checks. */
void Bench_Task(void)
{
  if (pump_state != last_pump_state) {
    if      (pump_state == 0) PumpStopAll();
    else if (pump_state == 1) PumpIn(200);
    else if (pump_state == 2) PositionControl_ResetPI();  // reset PI, keep pos_estimate
    last_pump_state = pump_state;
  }

  /* pump_state 2: pump in + accumulate volume -> slider position, open loop */
  if (pump_state == 2) {
    PumpIn(400);
    volume_pumped_mL += PositionControl_TrackFlowOnly();
    pos_true = PositionControl_TruePosFromVolume(volume_pumped_mL);
  }

  /* Gripper on nodes 7/8: 0 = stop and hold, 1 = move by grasping_degree
     (+ closes, - opens), 2 = close continuously until stopped. */
  if (last_grasping_state != grasping_state) {
    if      (grasping_state == 0) Grasp_StopNow();
    else if (grasping_state == 2) Grasp_Start();
    else                          Grasp_MoveBy(grasping_degree);
    last_grasping_state = grasping_state;
  }
}

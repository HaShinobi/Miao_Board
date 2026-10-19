// file: arm_states.h
//
// The arm state machine (program_state 1) and the manual bench mode
// (program_state 0). main.c owns the loop; this owns what happens inside it.
//
// arm_state:
//   0  idle, everything off      8  drain
//   1  servo homing              9  hold + pressure
//   2  tendon init (nodes 3/4)  10  joint forming init + fresh-trial reset
//   3  fill + pay out slack     11  (unused - falls through to idle)
//   4  grasp / release          12  closed-loop move to x_target_mm
//   5  separate + reel back     13  base pump out + end-tip pump in
//   6  joint rotation
//   7  big servo to arm7_target_deg, tendons released in step
//
// 3, 4, 5, 6, 7, 9 and 10 also keep the end-tip section flat (tip_flat_*).

#ifndef INC_ARM_STATES_H_
#define INC_ARM_STATES_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ---- commands (set these in Ozone) ---- */
extern volatile uint8_t arm_state;
extern volatile uint8_t last_arm_state;

extern volatile float   joint_degree;       // state 6 target [joint deg]
extern volatile uint8_t dir;                // state 6 sense: 0 = up, 1 = down
extern volatile float   grasping_degree;    // state 3: >= 0 close, < 0 open by |deg|

/* state 7: absolute servo angle, held. Homing runs once at startup.
   No apply flag - SDM_MoveOnChange fires whenever arm7_target_deg changes. */
extern volatile float   arm7_target_deg;
extern volatile int16_t arm7_speed_rpm;
extern volatile float   arm7_tol_deg;       // hold deadband; wider than one loop's
                                            // travel at low speed so it settles
extern volatile float   arm7_settle_deg;    // deg/pass below which it counts as stopped
extern volatile uint8_t arm7_settle_n;      // consecutive still passes before compensating
extern volatile float   arm7_sync_deg;      // release tendons every this many deg while turning

/* Startup home: extra degrees past 0 on the post-home move to the ready
   position, in the direction of travel (away from the hard stop). */
extern volatile float   home_ready_extra_deg;

/* End-tip keep-flat, states 3/4/5/6/7/9/10: end-tip pressure below
   tip_flat_kpa -> end-tip pump out at tip_flat_duty, else off. */
extern volatile uint8_t  tip_flat_enable;
extern volatile float    tip_flat_kpa;
extern volatile uint16_t tip_flat_duty;

/* state 13: base pump out + end-tip pump in, tip capped at xfer_tip_max_kpa. */
extern volatile uint16_t xfer_base_out_duty;
extern volatile uint16_t xfer_tip_in_duty;
extern volatile float    xfer_tip_max_kpa;

/* Pre-move pressure drop, states 6 and 7: bleed to lo..hi kPa and hold for
   pm_predrop_ms before the rotation/servo move fires. 0 = skip (normal
   pressure hold instead). See PreMove_PressureDrop() in arm_states.c. */
extern volatile uint8_t  pm_predrop_enable;
extern volatile float    pm_predrop_lo_kpa;
extern volatile float    pm_predrop_hi_kpa;
extern volatile uint16_t pm_predrop_duty;
extern volatile uint32_t pm_predrop_ms;

/* state 12: desired REAL slider travel -> PI setpoint, via the bench fit
   V[mL] = 3.893*L[mm] + 206.7 and x = V/(A_eff*1e6):
     x_target = 0.002492 * L[mm] + 0.1323
   The offset is the ~207 mL dead volume; zero it if you zero the volume
   after filling. */
extern volatile float   x_target_mm;
extern volatile float   x_cmd_scale;        // control-x [m] per mm of travel
extern volatile float   x_cmd_offset_m;     // dead-volume term [m]

/* String-loaded position correction. The volume->position fit was taken with
   no tendon load; loaded, the chamber bulges radially instead of extending, so
   the raw model over-reads by ~30 mm.
     corrected = raw * pc_str_gain + pc_str_offset_m
   Constant undershoot -> trim the offset. Grows with stroke -> cut the gain. */
extern volatile uint8_t pc_str_enable;
extern volatile float   pc_str_gain;        // 1.0 = no slope correction
extern volatile float   pc_str_offset_m;    // constant correction [m]
extern volatile float   pos_true_raw;       // uncorrected model output [m]

/* ---- manual bench mode (program_state 0) ---- */
extern volatile uint8_t grasping_state;      // 0 stop, 1 close, 2 open
extern volatile uint8_t last_grasping_state;

/* ---- entry points ---- */
void Arm_StartupHoming(void);        // one-shot servo home, fires at arm_state 1
void Arm_OnProgramStateChange(void); // re-arm the latches after a mode switch
void Arm_BeginPass(void);            // housekeeping before the dispatch switch
void Bench_Task(void);               // program_state 0: manual pump / motors / sensor ID

/* ---- one function per arm_state; main.c dispatches these ---- */
void ArmState_Idle(void);            //  0  everything off (+ coast window)
void ArmState_SDMHome(void);         //  1  servo homing
void ArmState_Home(void);            //  2  tendon homing (nodes 3/4)
void ArmState_Fill(void);            //  3  fill + pay out slack
void ArmState_Grasp(void);           //  4  grasp / release
void ArmState_SeparateSliders(void); //  5  separate + reel back
void ArmState_JointRotate(void);     //  6  joint rotation
void ArmState_ServoAngle(void);      //  7  big servo to arm7_target_deg
void ArmState_Drain(void);           //  8  pump out
void ArmState_Hold(void);            //  9  hold + pressure
void ArmState_InitSlider(void);      // 10  joint forming init + fresh-trial reset
void ArmState_MoveToPosition(void);  // 12  closed-loop move to x_target_mm
void ArmState_BaseOutTipIn(void);    // 13  base pump out + end-tip pump in

#ifdef __cplusplus
}
#endif

#endif /* INC_ARM_STATES_H_ */

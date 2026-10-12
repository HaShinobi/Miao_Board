// file: arm_control.h
//
// The four PUSIROBOT spool motors and what the arm does with them.
//   nodes 3 / 4  = slider string  -> payout (common mode) + joint rotation (differential)
//   nodes 7 / 8  = gripper tendons
// main.c owns the state machine; this owns the mechanism.
//
// Naming: UP / DOWN, not A / B.

#ifndef INC_ARM_CONTROL_H_
#define INC_ARM_CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdint.h>

/* Drive settings shared by all four spools (0x600A / 0x6017). */
extern volatile uint16_t spool_microstep;    // steps/rev = 200 * microstep
extern volatile uint16_t spool_stall_thresh; // open-loop stall threshold

/* 0x6005 read back after the most recent Pair_Config. 0 = position mode
   (wanted), 1 = speed mode (motor runs continuously), 0xFF = read failed.
   Nodes 5/6 configure after 3/4, so during arm_state 2 these show pair 1. */
extern volatile uint8_t  pair_mode_up;
extern volatile uint8_t  pair_mode_down;

/* ==================================================================== */
/*  STRING PAYOUT  (nodes 3 / 4, common mode)                           */
/*  arm_state 2 keeps string_excess_mm of slack ahead of the slider so   */
/*  the tendon never goes taut and jams it. arm_state 5 reels back in.   */
/* ==================================================================== */
extern volatile uint8_t  string_enable;        // 0 = disable all payout
extern volatile float    string_excess_mm;     // slack held during the fill [mm]
extern volatile float    string_step_mm;       // min increment before commanding [mm]
extern volatile float    string_paid_mm;       // net payout so far [mm] (readback)
extern volatile float    string_pos_mm;        // slider position feeding the target [mm]
extern volatile float    string_sep_pos_mm;    // position latched by state 5 [mm]
extern volatile float    string_reel_mm;       // last reel-in commanded [mm]
extern volatile float    string_sep_delay_ms;  // wait after separation before reeling

/* State-5 reel-back geometry (P1 tendon-connect / P2 joint-forming / O base
   reference) - see the comment above sep_delta_L_mm in arm_control.c. */
extern volatile float    sep_delta_L_mm;         // DeltaL = |P1P2| [mm]
extern volatile float    sep_L0_mm;              // L0 = |P1O| at the reference pose [mm]
extern volatile float    sep_geometric_pull_mm;  // readback: (X+L0)/2 - DeltaL
extern volatile float    sep_slack_mm;           // readback: L_re - X

extern volatile uint8_t  string_node_up;       // UP spool node id
extern volatile uint8_t  string_node_down;     // DOWN spool node id
extern volatile int8_t   string_dir_up;        // payout sense, UP   (+1 / -1)
extern volatile int8_t   string_dir_down;      // payout sense, DOWN (same as up)

extern volatile float    string_prewind_rev;   // revolutions unwound at state-2 entry
extern volatile float    string_prewind_mm;    // resulting length [mm] (readback)
extern volatile uint8_t  string_prewind_pending;
extern volatile int32_t  string_speed_pps;
extern volatile uint16_t string_accel;
extern volatile uint16_t string_decel;
extern volatile uint16_t string_steps_per_rev;
extern volatile uint16_t string_current_ma;
extern volatile uint8_t  string_stall_stop;    // 0x601B: 0 = ignore stall (tension!)

/* Loaded reel-in: chunked so a fault costs one chunk, not the whole move. */
extern volatile uint16_t string_reel_current_ma;
extern volatile float    string_chunk_mm;
extern volatile uint32_t string_chunk_timeout_ms;
extern volatile float    string_reel_extra_mm; // constant extra pull-back [mm]
extern volatile float    string_reel_done_mm;  // progress (readback)
extern volatile uint8_t  string_reel_abort;    // 1 = a chunk timed out (jam)

extern volatile uint8_t  string_init_done;
extern volatile uint8_t  string_err_up;        // last PUSI_MoveRel status
extern volatile uint8_t  string_err_down;
extern volatile float    string_last_deg;      // last commanded spool angle [deg]
extern volatile int32_t  string_spd_up;        // signed speed sent (sign = direction)
extern volatile int32_t  string_spd_down;

void String_PmcInit(void);
void String_InitBothPairs(void);   // arm_state 1: tension-home both pairs
void Inite_joint_forming(void);    // arm_state 10: home 3/4 + reset position
void String_Move  (float length_mm);   // pair 0: + = pay out, - = reel in
void String_ReelIn(float total_mm);    // pair 0: loaded pull, chunked + stall-proof
void String_StopAll(void);             // hard stop all four spools (3/4 and 5/6)

/* ---- second payout pair, nodes 5 / 6 --------------------------------
   Mirrors pair 0 during the fill and does nothing else: no reel-in, no
   joint rotation, no tensioning home. Shares the slack policy, the spool
   radius and every drive setting, so it pays out the same length.     */
extern volatile uint8_t  string2_enable;      // 0 = disable this pair
extern volatile uint8_t  string2_node_up;     // 5
extern volatile uint8_t  string2_node_down;   // 6
extern volatile int8_t   string2_dir_up;      // +1, node 5
extern volatile int8_t   string2_dir_down;    // -1, node 6
extern volatile float    string2_scale;       // 1.0 = identical to pair 0
extern volatile float    string2_paid_mm;     // net payout so far [mm]
extern volatile uint8_t  string2_init_done;
extern volatile uint8_t  string2_err_up;
extern volatile uint8_t  string2_err_down;
extern volatile int32_t  string2_spd_up;      // signed speed sent
extern volatile int32_t  string2_spd_down;

void String2_Init(void);
void String2_Move(float length_mm);    // pair 1: + = pay out, - = reel in
void String2_Stop(void);               // hard stop nodes 5/6
void    String2_ReelMode(uint8_t on);  // 1 = reel current + ignore stall, 0 = normal
uint8_t String2_ReelIn(float length_mm); // reel 5/6 in and wait: 1 = done, 0 = jammed

/* RT_SPEED stall fallback, used ONLY inside init_small_slider() below - see
   the comment above string2_stall_speed_pps in arm_control.c for why. Not
   applied to nodes 3/4 or the arm_state 1/2 startup home. */
extern volatile int32_t  string2_stall_speed_pps; // |RT_SPEED| below this = "not moving" [pps]
extern volatile uint8_t  string2_stall_stuck_n;   // consecutive not-moving polls = stalled
extern volatile uint16_t string2_stall_grace_n;   // ignore the detector this many polls after start

/* RT_SPEED-confirmed stall for node 4 (string_node_down) only, used inside
   String_InitBothPairs() - node 4 was observed stalling early there and
   tilting the slider. See the comment above string_down_confirm_speed_pps
   in arm_control.c for how this differs from the string2_stall_* fallback
   above (confirming a stall instead of declaring one the drive missed). */
extern volatile int32_t  string_down_confirm_speed_pps;   // |RT_SPEED| below this = "not moving" [pps]
extern volatile uint8_t  string_down_confirm_stuck_n;     // consecutive not-moving polls = confirmed stall
extern volatile uint16_t string_down_confirm_grace_n;     // ignore the confirm check this many polls per retry
extern volatile uint8_t  string_down_confirm_max_retries; // accept the drive's stall bit after this many retries

/* Pair_TensionHome readbacks - [0] up node, [1] down node of the pair being
   homed. See the comment above th_active in arm_control.c. */
extern volatile uint8_t  th_active;      // 1 while the seek loop is running
extern volatile uint8_t  th_node[2];
extern volatile uint8_t  th_done[2];
extern volatile uint8_t  th_status[2];   // last 0x6001 read (bit2 = stalled)
extern volatile int32_t  th_speed[2];    // last RT_SPEED read (small slider only)
extern volatile int32_t  th_pos[2];      // last motor position read (small slider only)
extern volatile uint32_t th_loops;

/* Tension-home nodes 5/6 only ("small slider"): pull back until each side
   stalls (independently - a stall on one does not stop the other), then
   clear stall/error status on both. Meant to run AFTER the joint-forming
   slider (nodes 3/4, see Inite_joint_forming) has already been tension-homed,
   since the small slider pulls back until it locks against it. Also uses
   the RT_SPEED stall fallback above, since the drive's own stall bit was
   observed not tripping reliably for this pair. */
void init_small_slider(void);

/* ==================================================================== */
/*  JOINT ROTATION  (same spools, differential mode)                    */
/*  Tendon length is the third side of a triangle, NOT proportional to   */
/*  angle:  X(phi) = sqrt(Ra^2 + Rb^2 - 2*Ra*Rb*cos phi).                */
/*  A constant gear ratio under-rotates by ~2x, so phi must be tracked.  */
/* ==================================================================== */
extern volatile int8_t   joint_dir_up;         // UP spool sign for "rotate up"
extern volatile int8_t   joint_dir_down;       // DOWN spool sign
extern volatile int8_t   joint_phi_sense;      // which way phi moves for dir 0
                                               // (-1 = bench-corrected: 0 = UP)
extern volatile int32_t  joint_speed_pps;
extern volatile uint16_t joint_accel;
extern volatile uint16_t joint_decel;
extern volatile uint8_t  joint_err_up;
extern volatile uint8_t  joint_err_down;
extern volatile int32_t  joint_spd_up;
extern volatile int32_t  joint_spd_down;
extern volatile float    joint_motor_deg;      // commanded spool angle [deg]
extern volatile uint8_t  joint_skip;           // 0 moved, 1 angle ~0, 2 steps 0,
                                               // 4 = at a software limit
/* Travel limits on the tracked pose. dL/dphi goes as sin(phi), so the pull
   direction for a given dir reverses at 0 and 180 deg - clamping phi is
   what stops the joint reversing once it passes a limit. Narrow these to
   the real mechanical range if it is tighter.                          */
extern volatile float    joint_phi_min_deg;
extern volatile float    joint_phi_max_deg;
extern volatile uint8_t  joint_limit_hit;      // 1 = a limit was reached
extern volatile uint32_t joint_moves;          // moves actually issued

extern volatile float    joint_Ra;             // radius to moving anchor [mm]
extern volatile float    joint_Bx;             // fixed anchor x from centre [mm]
extern volatile float    joint_By;             // fixed anchor y from centre [mm]
extern volatile float    joint_phi_home_deg;   // phi at the homed reference [deg]
extern volatile float    joint_phi_deg;        // current phi [deg] (tracked)
extern volatile float    joint_dX_mm;          // tendon length change this move [mm]
extern volatile float    joint_X_mm;           // current tendon length [mm]
extern volatile float    joint_gain;           // residual trim (1.0 = none)
extern volatile float    joint_slack_deg;      // constant take-up [joint deg]
extern volatile float    joint_cmd_deg;        // corrected joint angle [deg]

/* Tensioning home: reel both in until each stalls on a taut tendon, zero
   there. Sets the phi reference every later rotation is measured from. */
extern volatile uint8_t  joint_home_apply;     // 1 = home on next arm_state 6
extern volatile int32_t  joint_home_pps;       // slow seek speed [pps]
extern volatile uint16_t joint_home_current_ma;// reduced so it stalls gently
extern volatile uint16_t joint_home_thresh;
extern volatile uint32_t joint_home_timeout_ms;
extern volatile uint8_t  joint_home_state;     // 0 not run, 1 homed, 2 timeout
extern volatile uint8_t  joint_homed;
extern volatile uint32_t joint_home_ms;

void  Joint_HomeSpools(void);
void  Joint_Move(float joint_deg, int dir);    // dir 0 = up, 1 = down
float Joint_TendonLen(float phi_deg);

/* ---- base-rotation tendon compensation -------------------------------
   Each tendon anchor P2 rides a circle of radius base_R about the rotation
   centre; the far anchor P1 is fixed. The tendon is the straight 3D line
   between them, so base rotation changes its length with no joint command:

     L(th)^2 = offx^2 + rho^2 + R^2 - 2*rho*R*cos(th - psi)

   Each spool sits at its own th0 on that circle, so their corrections are
   NOT equal and opposite - each is computed and moved separately, for all
   four slider tendons (nodes 3/4 and 5/6). dL is also not monotonic in base
   angle; it peaks near th - psi = 180.
   Called by arm_state 7 every arm7_sync_deg of rotation WHILE the base
   moves, with the live measured base angle, and once more where it parks.
   Lengths are measured from base_ref_deg, the angle the tendons were last
   tensioned at (set by the tension-homes via Base_CompReset).            */
extern volatile uint8_t  joint_base_comp;    // 0 = disable compensation
extern volatile float    base_R;             // P2 circle radius [mm]
extern volatile float    base_rho;           // P1 projected into P2's plane [mm]
extern volatile float    base_psi_deg;       // angle of that projection
extern volatile float    base_offx;          // out-of-plane offset [mm]
extern volatile float    base_th0_up;        // node 3 anchor at base 0 [deg]
extern volatile float    base_th0_down;      // node 4 anchor at base 0 [deg]
extern volatile float    base_th0_s2_up;     // node 5 anchor at base 0 [deg] (defaults = node 3)
extern volatile float    base_th0_s2_down;   // node 6 anchor at base 0 [deg] (defaults = node 4)
extern volatile int8_t   joint_base_sense;   // flip if it corrects backwards

extern volatile float    base_ref_deg;       // base angle the tendons were tensioned at
extern volatile float    joint_base_deg;     // base angle applied so far
extern volatile float    base_need_up;       // length moved so far [mm], per tendon
extern volatile float    base_need_down;
extern volatile float    base_need_s2_up;
extern volatile float    base_need_s2_down;
extern volatile float    base_do_up;         // owed this pass [mm], per tendon
extern volatile float    base_do_down;
extern volatile float    base_do_s2_up;
extern volatile float    base_do_s2_down;
extern volatile uint32_t joint_base_moves;

float Base_TendonLen(float th_deg);
void  Base_CompReset(void);    // all four tendons just tensioned at the current base angle
void  Base_CompSyncS2(void);   // only nodes 5/6 just tensioned
void  Joint_CompensateBase(float base_deg);

/* ==================================================================== */
/*  GRASPING  (nodes 7 / 8)                                             */
/*  Closes until each side stalls on the object - the stall IS the grip, */
/*  so there is no position target.                                      */
/* ==================================================================== */
extern volatile uint8_t  grasp_node_up;
extern volatile uint8_t  grasp_node_down;
extern volatile int8_t   grasp_dir_up;         // sign that CLOSES
extern volatile int8_t   grasp_dir_down;
extern volatile int32_t  grasp_pps;            // closing speed [pps]
extern volatile uint16_t grasp_current_ma;     // grip force [mA]
extern volatile uint16_t grasp_thresh;         // 0x6017 stall threshold
extern volatile uint16_t grasp_steps_per_rev;
extern volatile uint16_t grasp_accel;
extern volatile uint16_t grasp_decel;
extern volatile uint8_t  grasp_stop_apply;     // 1 = stop both spools NOW
extern volatile uint8_t  grasp_state;          // 0 idle / holding, 3 closing

void Grasp_Start(void);            // close continuously; runs until stopped
void Grasp_StopNow(void);          // hard stop, keeps holding torque
void Grasp_MoveBy(float degrees);  // finite move: + closes, - opens

#ifdef __cplusplus
}
#endif

#endif /* INC_ARM_CONTROL_H_ */

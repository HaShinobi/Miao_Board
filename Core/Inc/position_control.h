// Core/Inc/position_control.h
//
// Bidirectional saturated PI position controller for one hydraulic slider.
//
//   direction   = sign(x_target - pos_estimate)   (+1 fill, -1 drain)
//   eff_target  = x_target - direction * PC_OVERSHOOT_COMP
//   error       = direction * (eff_target - pos_estimate)   (>= 0 while moving toward target)
//   P_term      = Kp * error
//   I_term      = integral(Ki * error) dt                   (clamped to [0, PC_I_LIMIT])
//   flow_cmd    = saturate(P_term + I_term, 0, Q_max)
//   flow_cmd > 0 + direction= 1 -> PumpIn(duty)
//   flow_cmd > 0 + direction=-1 -> PumpOut(duty)
//   flow_cmd = 0                -> PumpStopAll()
//
#ifndef INC_POSITION_CONTROL_H_
#define INC_POSITION_CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define PC_A_EFF             1.5625e-3f   // effective cylinder area [m^2] — calibrated from physical measurement
#define PC_Q_MAX             1.667e-5f   // max pump flow [m^3/s]
#define PC_KP                3.22e-3f
#define PC_KI                6.44e-5f
#define PC_L_ARM             1.0f        // max arm extension [m]
#define PC_I_LIMIT           (PC_Q_MAX / 10.0f)
#define PC_E_SAT             (PC_Q_MAX / PC_KP)
#define PC_X_TARGET_DEFAULT  0.1f

// Fill: stop this many metres early, hydraulic inertia carries the rest.
// Calibrated: x_target=0.32m (500mL) and x_target=0.512m (800mL) -> avg coasting ~0.042m
#define PC_OVERSHOOT_COMP    0.042f

// Drain: stop this many metres early. Much smaller than fill because water
// does not keep flowing after the pump stops (no hydraulic inertia on drain).
#define PC_DRAIN_COMP        0.001f

// Drain flow scale: the flow sensor over-reports outflow relative to fill
// (~1.65x observed). Applied to negative measured flow only, so fill/state-8
// fill calibration is unaffected. Calibrated from one point (k = 120/198);
// verify with a second drain size and refine if needed.
#define PC_DRAIN_FLOW_SCALE  0.60f

// True-position calibration for pos_true / pos_phys (the readout).
// Regression: y[m] = PC_ML_TO_M_SLOPE * V[mL] + PC_ML_TO_M_OFFSET
// 25-trial least-squares refit (extends the range to 1134 mL):
//   k = 0.2757 mm/mL, b = -53.63 mm  ->  y[m] = 2.757e-4*V - 0.05363
//   R^2 = 0.9896, MAE 5.50 mm, RMSE 6.89 mm, ~195 mL dead volume.
//   Implied A_eff 3.63e-3 m^2 -> effective radius ~34.0 mm vs nominal 32 mm
//   (6% excess - the closest the fit has come to the true bore; the 20-trial
//   fit was 12% out, so the wider range is recovering real geometry).
//   Largest residuals: trial 8 (-17.6 mm) and trial 23 (+13.6 mm). Trials 8
//   and 16 report the SAME displacement 34 mL apart, so some readings are
//   graduation-limited; excluding those would tighten the fit.
// NOTE: valid only if volume_pumped_mL is zeroed at the same physical
// reference condition the regression data was taken from (managed manually).
#define PC_ML_TO_M_SLOPE     2.757e-4f
#define PC_ML_TO_M_OFFSET    (-0.05363f)

extern volatile float x_target;
extern volatile float pos_estimate;
extern volatile float pump_flow_cmd;
extern volatile float slider_velocity;  // m/s — visible in Ozone
extern volatile float pos_true;         // physical slider position [m] from volume regression — visible in Ozone
extern volatile uint32_t flow_sample_ms; // HAL_GetTick() of the last flow-sensor sample (staleness guard for tendon_sync)

// Signed net volume [mL] since manual zero: fill adds, drain subtracts (with
// PC_DRAIN_FLOW_SCALE applied). Never auto-reset — zero it manually (Ozone) at
// the same physical reference the regression was calibrated from. Feeds
// pos_phys and, later, the Phase-2 tendon length loop.
extern volatile float net_volume_mL;
extern volatile float pos_phys;         // physical slider position [m] = regression(net_volume_mL)

void PositionControl_Init(void);
void PositionControl_ResetPI(void);
void PositionControl_Update(void);
float PositionControl_TrackFlowOnly(void);  // returns mL added this call (0 if no inflow)
float PositionControl_TruePosFromVolume(float volume_mL);  // stateless mapping, clamped to [0, PC_L_ARM]

#ifdef __cplusplus
}
#endif

#endif /* INC_POSITION_CONTROL_H_ */

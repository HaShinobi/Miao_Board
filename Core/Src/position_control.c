// Core/Src/position_control.c
//
// Bidirectional saturated PI position controller.
//
// Direction is determined from the sign of (x_target - pos_estimate):
//   positive -> fill only  (PumpIn)
//   negative -> drain only (PumpOut)
//
// A one-sided PI (error magnitude, flow_cmd always >= 0) is used in both
// cases. The effective target is shifted inward by PC_OVERSHOOT_COMP so the
// controller stops early and lets hydraulic inertia carry the arm the
// remaining distance. When the arm overshoots the effective target, the
// negative p_term drives flow_cmd to zero and the pump stops.
//
// The integral is reset whenever direction changes so a large fill integral
// does not immediately saturate the drain command (and vice versa).

#include "position_control.h"
#include "pump_application_function.h"
#include "pump_control.h"
#include "flow_sensor.h"

/* ---- Exported controller state ---- */
volatile float x_target        = PC_X_TARGET_DEFAULT;
volatile float pos_estimate    = 0.0f;
volatile float pump_flow_cmd   = 0.0f;
volatile float slider_velocity = 0.0f;  // m/s — visible in Ozone
volatile float pos_true        = 0.0f;  // physical slider position [m] — visible in Ozone
volatile uint32_t flow_sample_ms = 0;   // tick of last flow sample (tendon_sync staleness guard)
volatile float net_volume_mL   = 0.0f;  // signed net volume since manual zero (fill +, drain −)
volatile float pos_phys        = 0.0f;  // physical slider position [m] from net_volume_mL

/* ---- Internal PI state ---- */
static float    integral_state     = 0.0f;
static uint32_t last_tick_ms       = 0;
static int8_t   current_dir        = 0;    // 1 = filling, -1 = draining
static float    drain_start_pos    = 0.0f; // pos_estimate when drain began

void PositionControl_Init(void)
{
    pos_estimate   = 0.0f;
    integral_state = 0.0f;
    pump_flow_cmd  = 0.0f;
    current_dir    = 0;
    last_tick_ms   = HAL_GetTick();
}

/* Map cumulative pumped volume [mL] to true slider position [m] using the
   bench-calibrated regression. Stateless: derived from the single shared
   volume integral (volume_pumped_mL), so it cannot drift independently of it. */
float PositionControl_TruePosFromVolume(float volume_mL)
{
    float y = PC_ML_TO_M_SLOPE * volume_mL + PC_ML_TO_M_OFFSET;
    if (y < 0.0f)      y = 0.0f;   // inside dead volume, slider not moving yet
    if (y > PC_L_ARM)  y = PC_L_ARM;
    return y;
}

/* Reset PI state only — pos_estimate is intentionally preserved so that
   residual water already in the arm is not lost from the position estimate. */
void PositionControl_ResetPI(void)
{
    integral_state = 0.0f;
    pump_flow_cmd  = 0.0f;
    current_dir    = 0;
    last_tick_ms   = HAL_GetTick();
}

/* One flow sample, shared by TrackFlowOnly and Update so the two can never
   drift apart. Advances the timestep, reads the sensor, applies the deadband
   and drain correction, and maintains slider_velocity / net_volume_mL /
   pos_phys. Returns the corrected flow [m^3/s]; *dt_out gets the timestep.

   Deadband is symmetric so the sensor floor and decay tail are ignored;
   negative (drain) flow passes through scaled, keeping slider_velocity and
   net_volume_mL signed-correct in every state. The drain scale exists because
   the sensor over-reports outflow ~1.65x (bench: a 500->300 mL command
   stopped at ~380 mL real => k = 120/198). */
static float PC_SampleFlow(float *dt_out)
{
    uint32_t now_ms = HAL_GetTick();
    float dt = (now_ms - last_tick_ms) / 1000.0f;
    last_tick_ms = now_ms;
    if (dt <= 0.0f || dt > 1.0f) dt = 0.2f;

    float flow = FlowSensor_ReadFlowRate_m3s(dt);
    flow_sample_ms = HAL_GetTick();
    if (flow > -5.0e-7f && flow < 5.0e-7f) flow = 0.0f;
    if (flow < 0.0f) flow *= PC_DRAIN_FLOW_SCALE;

    slider_velocity = flow / PC_A_EFF;                 // m/s - logged in Ozone
    net_volume_mL  += flow * dt * 1.0e6f;              // signed, since manual zero
    pos_phys        = PositionControl_TruePosFromVolume(net_volume_mL);

    *dt_out = dt;
    return flow;
}

/* Integrate pos_estimate from the flow sensor without touching the pump.
   Returns the mL that flowed IN this call (0 while draining or idle).
   Caller accumulates: volume_pumped_mL += PositionControl_TrackFlowOnly(); */
float PositionControl_TrackFlowOnly(void)
{
    float dt;
    float measured_flow = PC_SampleFlow(&dt);

    if (measured_flow <= 0.0f) return 0.0f;

    pos_estimate += slider_velocity * dt;
    if (pos_estimate < 0.0f)     pos_estimate = 0.0f;
    if (pos_estimate > PC_L_ARM) pos_estimate = PC_L_ARM;

    return measured_flow * dt * 1.0e6f;
}

void PositionControl_Update(void)
{
    /* 1. Direction from raw error (evaluated before the position update) */
    float raw_error = x_target - pos_estimate;
    int8_t dir = (raw_error >= 0.0f) ? 1 : -1;

    /* Reset integral on direction change to avoid fighting the new direction */
    if (dir != current_dir)
    {
        integral_state  = 0.0f;
        current_dir     = dir;
        if (dir == -1)
            drain_start_pos = pos_estimate; // snapshot start position for drain comp
    }

    /* 2. Flow sample, then integrate to position. Only integrate while the
       pump is actually commanded, so the sensor floor cannot drift
       pos_estimate while idle. */
    float dt;
    (void)PC_SampleFlow(&dt);

    if (pump_flow_cmd != 0.0f)
    {
        pos_estimate += slider_velocity * dt;
        if (pos_estimate < 0.0f)     pos_estimate = 0.0f;
        if (pos_estimate > PC_L_ARM) pos_estimate = PC_L_ARM;
    }

    /* 4+5+6. Direction-specific actuation */
    if (dir == 1)
    {
        /* FILL: PI with fixed overshoot compensation. */
        float effective_target = x_target - PC_OVERSHOOT_COMP;
        float error = effective_target - pos_estimate;

        float p_term = PC_KP * error;

        integral_state += PC_KI * error * dt;
        if (integral_state < 0.0f)       integral_state = 0.0f;
        if (integral_state > PC_I_LIMIT) integral_state = PC_I_LIMIT;

        float flow_cmd = p_term + integral_state;
        if (flow_cmd < 0.0f)     flow_cmd = 0.0f;
        if (flow_cmd > PC_Q_MAX) flow_cmd = PC_Q_MAX;
        pump_flow_cmd = flow_cmd;

        if (flow_cmd > 0.0f)
        {
            uint16_t duty = (uint16_t)((flow_cmd / PC_Q_MAX) * 1000.0f);
            if (duty > 1000) duty = 1000;
            PumpIn(duty);
        }
        else
        {
            PumpStopAll();
        }
    }
    else
    {
        /* DRAIN: run pump at full speed, stop early by PC_DRAIN_COMP so
           inertia carries pos_estimate the rest of the way to x_target. */
        pump_flow_cmd = -PC_Q_MAX;

        float drain_stop_pos = x_target + PC_DRAIN_COMP;

        if (pos_estimate > drain_stop_pos)
        {
            PumpOut(1000);
        }
        else
        {
            PumpStopAll();
        }
    }
}

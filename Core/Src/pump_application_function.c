// Core/Src/pump_application_function.c
#include "pump_application_function.h"
#include "pump_control.h"
#include "pressure_sensor.h"
#include "main.h"

extern Pump_t pump_In;
extern Pump_t pump_Out;
extern Pump_t pump_TipIn;
extern Pump_t pump_TipOut;
extern volatile float pressure_value;

// Last commanded pump duties (0..1000) — tendon_sync feed-forward.
volatile uint16_t pump_in_duty_cmd  = 0;
volatile uint16_t pump_out_duty_cmd = 0;

// ---- Closed-loop pressure control (proportional, pump-in only) ----
// Pump water IN to reach and hold pc_target. The pump duty is scaled by how far
// the pressure is below target, so the pump eases off as it nears pc_target
// (lower duty close to the setpoint) to avoid overshoot. At/above target - or
// within pc_deadband of it - the pump stops. All fields live-tunable in Ozone.
volatile float    pc_target   = 60.0f;   // target pressure
volatile float    pc_deadband = 1.0f;    // stop when within this of target (anti-chatter)
volatile float    pc_kp       = 20.0f;   // pump duty per unit of pressure error
volatile uint16_t pc_duty_min = 40;      // floor duty while pumping (overcome stiction)
volatile uint16_t pc_duty_max = 250;     // ceiling duty (far below target)


//Pump control functions
void PumpIn(uint16_t speed)
{
  pump_in_duty_cmd  = (speed > 1000) ? 1000 : speed;
  pump_out_duty_cmd = 0;
  HAL_GPIO_WritePin(ValveBaseOut_GPIO_Port, ValveBaseOut_Pin, GPIO_PIN_RESET);
  // Disable PE9
  Pump_Stop(&pump_Out);
  HAL_GPIO_WritePin(ValveBaseIn_GPIO_Port, ValveBaseIn_Pin, GPIO_PIN_SET);
  HAL_Delay(5); // Ensure valve has time to switch before starting the pump
  // Enable PE13
  Pump_SetSpeed(&pump_In, speed);
}

void PumpOut(uint16_t speed)
{
  pump_in_duty_cmd  = 0;
  pump_out_duty_cmd = (speed > 1000) ? 1000 : speed;
  HAL_GPIO_WritePin(ValveBaseIn_GPIO_Port, ValveBaseIn_Pin, GPIO_PIN_RESET);
  // Disable PE13
  Pump_Stop(&pump_In);
  HAL_GPIO_WritePin(ValveBaseOut_GPIO_Port, ValveBaseOut_Pin, GPIO_PIN_SET);
  HAL_Delay(5); // Ensure valve has time to switch before starting the pump
  // Enable PE9
  Pump_SetSpeed(&pump_Out, speed);
}

void PumpStopAll(void)
{
    pump_in_duty_cmd  = 0;
    pump_out_duty_cmd = 0;
    Pump_Stop(&pump_In);
    Pump_Stop(&pump_Out);
    HAL_Delay(5); // Ensure pumps have time to stop before switching valves
    HAL_GPIO_WritePin(ValveBaseIn_GPIO_Port, ValveBaseIn_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ValveBaseOut_GPIO_Port, ValveBaseOut_Pin, GPIO_PIN_RESET);
    HAL_Delay(5); // Ensure pumps have time to stop before switching valves
}

void PumpDisAll(void)
{
    pump_in_duty_cmd  = 0;
    pump_out_duty_cmd = 0;
    HAL_Delay(5); // Ensure pumps have time to stop before switching valves
    Pump_Stop(&pump_In);
    Pump_Stop(&pump_Out);
}

// ---- End-tip pumps (TIM2: PA2 in, PA0 out; valves PB11 in, PD10 out) ----
void TipPumpIn(uint16_t speed)
{
  HAL_GPIO_WritePin(ValveTipOut_GPIO_Port, ValveTipOut_Pin, GPIO_PIN_RESET);
  Pump_Stop(&pump_TipOut);
  HAL_GPIO_WritePin(ValveTipIn_GPIO_Port, ValveTipIn_Pin, GPIO_PIN_SET);
  HAL_Delay(5); // Ensure valve has time to switch before starting the pump
  Pump_SetSpeed(&pump_TipIn, speed);
}

void TipPumpOut(uint16_t speed)
{
  HAL_GPIO_WritePin(ValveTipIn_GPIO_Port, ValveTipIn_Pin, GPIO_PIN_RESET);
  Pump_Stop(&pump_TipIn);
  HAL_GPIO_WritePin(ValveTipOut_GPIO_Port, ValveTipOut_Pin, GPIO_PIN_SET);
  HAL_Delay(5); // Ensure valve has time to switch before starting the pump
  Pump_SetSpeed(&pump_TipOut, speed);
}

void TipPumpStopAll(void)
{
    Pump_Stop(&pump_TipIn);
    Pump_Stop(&pump_TipOut);
    HAL_Delay(5); // Ensure pumps have time to stop before switching valves
    HAL_GPIO_WritePin(ValveTipIn_GPIO_Port, ValveTipIn_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ValveTipOut_GPIO_Port, ValveTipOut_Pin, GPIO_PIN_RESET);
}

void Sensor_PressureControl(void)
{
    Sensor_ReadOnce();                          // refresh pressure_value

    float error = pc_target - pressure_value;   // > 0 => below target => pump in

    /* At/above target (or within the deadband): hold. This is a pump-IN-only
       regulator, so if the pressure overshoots it is left to bleed down on its
       own rather than actively drained. */
    if (error <= pc_deadband)
    {
        PumpDisAll();
        return;
    }

    /* Proportional control: duty is large when far below target and shrinks
       toward pc_duty_min as the pressure approaches pc_target, so the pump eases
       off near the setpoint instead of slamming past it. */
    float duty_f = pc_kp * error;
    uint16_t duty = (duty_f > (float)pc_duty_max) ? pc_duty_max
                  : (duty_f < (float)pc_duty_min) ? pc_duty_min
                  : (uint16_t)duty_f;

    PumpIn(duty);
}


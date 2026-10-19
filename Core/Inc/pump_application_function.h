// Core/Inc/pump_application_function.h

#ifndef INC_PUMP_APPLICATION_FUNCTION_H_
#define INC_PUMP_APPLICATION_FUNCTION_H_

#include "main.h" // 包含这行可以很方便地获取到 uint16_t 等类型定义

// Last commanded pump duties (0..1000). Feed-forward sources for tendon_sync:
// exactly one is nonzero while a pump runs, both 0 when stopped.
extern volatile uint16_t pump_in_duty_cmd;
extern volatile uint16_t pump_out_duty_cmd;
extern volatile uint16_t tip_in_duty_cmd;     // end-tip pumps, same idea
extern volatile uint16_t tip_out_duty_cmd;

// 函数原型声明 (Function Prototypes)
void PumpIn(uint16_t speed);
void PumpOut(uint16_t speed);
void PumpStopAll(void);
void TipPumpIn(uint16_t speed);   // end-tip pump, PA2 + valve PB11
void TipPumpOut(uint16_t speed);  // end-tip pump, PA0 + valve PD10
void TipPumpStopAll(void);
void Sensor_PressureControl(void);
void Sensor_PID_Reset(void);

#endif /* INC_PUMP_APPLICATION_FUNCTION_H_ */
// Core/Inc/pump_control.h

#ifndef INC_PUMP_CONTROL_H_
#define INC_PUMP_CONTROL_H_

#include "main.h" // 包含主头文件以获取HAL库定义

// 定义水泵控制结构体
typedef struct {
    TIM_HandleTypeDef* htim;        // 指向定时器句柄的指针
    uint32_t           Channel;     // 定时器通道 (e.g., TIM_CHANNEL_1)
    uint32_t           MaxCompareValue; // PWM最大比较值 (等于ARR)
} Pump_t;

// 函数原型声明
void Pump_Init(Pump_t* pump);
void Pump_SetSpeed(Pump_t* pump, uint16_t speed);
void Pump_Stop(Pump_t* pump);

#endif /* INC_PUMP_CONTROL_H_ */
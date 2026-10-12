// Core/Src/pump_control.c

#include "pump_control.h"

void Pump_Init(Pump_t* pump)
{
    __HAL_TIM_SetCompare(pump->htim, pump->Channel, pump->MaxCompareValue);
    HAL_TIM_PWM_Start(pump->htim, pump->Channel);
}


void Pump_SetSpeed(Pump_t* pump, uint16_t speed)
{
    if (speed > 1000)
        speed = 1000;

    if (speed == 0)
    {
        __HAL_TIM_SetCompare(pump->htim, pump->Channel, pump->MaxCompareValue); // 0%占空比
        return;
    }

    uint32_t compare_value = pump->MaxCompareValue - ((uint32_t)speed * pump->MaxCompareValue / 1000);
    __HAL_TIM_SetCompare(pump->htim, pump->Channel, compare_value);
    //HAL_TIM_PWM_Start(pump->htim, pump->Channel);      // 确保PWM在运行
}

void Pump_Stop(Pump_t* pump)
{
    __HAL_TIM_SetCompare(pump->htim, pump->Channel, pump->MaxCompareValue); // 0%占空比
    //HAL_TIM_PWM_Stop(pump->htim, pump->Channel);
}
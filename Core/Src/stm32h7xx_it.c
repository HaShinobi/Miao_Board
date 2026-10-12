#include "main.h"
#include "stm32h7xx_it.h"
#include "pressure_sensor.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;   // SDM8070JA1 servo bus
extern DMA_HandleTypeDef  hdma_usart3_rx;
extern UART_HandleTypeDef huart2;   // flow sensor
extern UART_HandleTypeDef huart3;   // pressure sensor

void NMI_Handler(void)        { while (1) {} }
void HardFault_Handler(void)  { while (1) {} }
void MemManage_Handler(void)  { while (1) {} }
void BusFault_Handler(void)   { while (1) {} }
void UsageFault_Handler(void) { while (1) {} }
void SVC_Handler(void)        {}
void DebugMon_Handler(void)   {}
void PendSV_Handler(void)     {}
void SysTick_Handler(void)    { HAL_IncTick(); }

void DMA1_Stream0_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_usart3_rx); }
void FDCAN1_IT0_IRQHandler(void)   { HAL_FDCAN_IRQHandler(&hfdcan1); }
void FDCAN1_IT1_IRQHandler(void)   { HAL_FDCAN_IRQHandler(&hfdcan1); }
void FDCAN2_IT0_IRQHandler(void)   { HAL_FDCAN_IRQHandler(&hfdcan2); }
void FDCAN2_IT1_IRQHandler(void)   { HAL_FDCAN_IRQHandler(&hfdcan2); }
void USART2_IRQHandler(void)       { HAL_UART_IRQHandler(&huart2); }
void USART3_IRQHandler(void)       { HAL_UART_IRQHandler(&huart3); }

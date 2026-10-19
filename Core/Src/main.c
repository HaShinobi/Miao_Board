/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "pump_control.h"
#include "pump_application_function.h"
#include "Pressure_sensor.h"
#include "position_control.h"
#include "flow_sensor.h"
#include <math.h>
#include "sdm8070.h"
#include "pusirobot.h"
#include "arm_control.h"
#include "arm_states.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;   /* SDM8070JA1 servo bus: 1 Mbps, standard IDs */

TIM_HandleTypeDef htim1;   // base pumps     (PE9 CH1 out, PE13 CH3 in)
TIM_HandleTypeDef htim2;   // end-tip pumps  (PA0 CH1 out, PA2 CH3 in)

UART_HandleTypeDef huart2;  // Flow sensor     (USART2: PD5/PD6, DE=PD4)
UART_HandleTypeDef huart3;  // Pressure sensor (USART3: PD8/PD9, DE=PB14)
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */

/* Bump on every change and check it in Ozone - a stale flash is the
   commonest false test result. */
volatile uint32_t g_build_tag = 0x261019AB;

volatile uint8_t program_state = 0;
volatile uint8_t last_program_state = 0;

//Sensor reading variables
extern volatile int16_t  pressure_raw;
extern volatile float    pressure_value;
extern volatile HAL_StatusTypeDef debug_tx_status;
extern volatile HAL_StatusTypeDef debug_rx_status;
extern volatile uint16_t debug_len;
extern volatile uint8_t debug_raw[10];

/* Pump */
volatile uint8_t pump_state       = 0;
volatile uint8_t last_pump_state  = 0;
volatile float   volume_pumped_mL = 0.0f;
Pump_t pump_In;
Pump_t pump_Out;
Pump_t pump_TipIn;
Pump_t pump_TipOut;

/* Mechanical */
const float r_motor = 12.5f;   // spool radius [mm]

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// FDCAN RX FIFO0 callback: hand every UIM242 reply/RTCN frame to the driver.
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  FDCAN_RxHeaderTypeDef rxHeader;
  uint8_t rxData[8];

  if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
  {
    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0)
    {
      if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK)
        break;

      /* True payload length. In THIS HAL version FDCAN_DLC_BYTES_n is defined
         as the plain byte count (FDCAN_DLC_BYTES_8 == 0x08), NOT the older
         "n << 16" encoding - so DataLength is used directly. Getting this
         wrong made len read 0 for every frame, which silently blocked the
         SDM8070 position decode (its gates require len >= 8 / >= 3). */
      uint8_t len = (uint8_t)rxHeader.DataLength;
      if (len > 8) len = 8;

      /* Dispatch by peripheral: the two buses speak DIFFERENT protocols.
         FDCAN1 = PMC006C3 CANopen SDO replies (11-bit standard, 0x580+node).
         FDCAN2 = SDM8070JA1 servo (11-bit standard, its own protocol). */
      if (hfdcan->Instance == FDCAN2) {
        SDM_HandleRxFrame(rxHeader.Identifier, rxData, len);
      } else if (rxHeader.IdType == FDCAN_STANDARD_ID) {
        PUSI_HandleSdoResp(rxHeader.Identifier, rxData, len);
      }
    }
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();       // SDM8070JA1 servo bus (1 Mbps, standard ids)
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();  // Flow sensor
  MX_USART3_UART_Init();  // Pressure sensor

  /* USER CODE BEGIN 2 */

  // ---- Pump In ----
  pump_In.htim = &htim1;
  pump_In.Channel = TIM_CHANNEL_3;
  pump_In.MaxCompareValue = __HAL_TIM_GET_AUTORELOAD(&htim1);
  Pump_Init(&pump_In);

  // ---- Pump Out ----
  pump_Out.htim = &htim1;
  pump_Out.Channel = TIM_CHANNEL_1;
  pump_Out.MaxCompareValue = __HAL_TIM_GET_AUTORELOAD(&htim1);
  Pump_Init(&pump_Out);

  // ---- End-tip Pump In ----
  pump_TipIn.htim = &htim2;
  pump_TipIn.Channel = TIM_CHANNEL_3;
  pump_TipIn.MaxCompareValue = __HAL_TIM_GET_AUTORELOAD(&htim2);
  Pump_Init(&pump_TipIn);

  // ---- End-tip Pump Out ----
  pump_TipOut.htim = &htim2;
  pump_TipOut.Channel = TIM_CHANNEL_1;
  pump_TipOut.MaxCompareValue = __HAL_TIM_GET_AUTORELOAD(&htim2);
  Pump_Init(&pump_TipOut);

  Pump_Stop(&pump_In);
  Pump_Stop(&pump_Out);
  Pump_Stop(&pump_TipIn);
  Pump_Stop(&pump_TipOut);

  // FDCAN filter configuration
  // UIM242 replies use 29-bit extended ids, so accept the full extended range.
  FDCAN_FilterTypeDef sFilterConfig;

  sFilterConfig.IdType = FDCAN_EXTENDED_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x00000000;
  sFilterConfig.FilterID2 = 0x1FFFFFFF;      // 29-bit max

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  // ACCEPT standard-id frames into FIFO0 too: the PMC006C3 CANopen SDO replies
  // (0x580+node) are standard-id and share this bus with the extended UIM242
  // frames. The RX callback splits them by id type (extended->UIM, standard->PMC).
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT,
                                   FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* ---- FDCAN2: SDM8070JA1 servo bus (1 Mbps, STANDARD 11-bit ids) ----
     Mirror of the above but for standard ids: accept the whole 11-bit range
     so the motor's frames (0x202 group, 0x208/0x209 per-motor) all arrive,
     and reject extended frames here (they belong to the other bus). */
  FDCAN_FilterTypeDef sFilterConfig2;
  sFilterConfig2.IdType       = FDCAN_STANDARD_ID;
  sFilterConfig2.FilterIndex  = 0;
  sFilterConfig2.FilterType   = FDCAN_FILTER_RANGE;
  sFilterConfig2.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig2.FilterID1    = 0x000;
  sFilterConfig2.FilterID2    = 0x7FF;       // 11-bit max
  if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT,
                                   FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }

  Sensor_Init();          // Pressure sensor init (keeps RS485-U3 DE low)
  FlowSensor_Init();      // Flow sensor init
  PositionControl_Init();
  PUSI_Init(&hfdcan1);         // PMC006C3 CANopen: shares FDCAN1 (125k, standard ids)
  (void)g_build_tag;          // volatile read: stops --gc-sections stripping the
                              // freshness marker, so it stays visible in Ozone

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    Sensor_ReadOnce();    // Base pressure (RS485-U3 addr 1), every loop (~200 ms period)
    Sensor_ReadTip();     // End-tip pressure (RS485-U3 addr 2)

    Arm_StartupHoming();     // one-shot servo home when arm_state 1 is reached

    if (program_state != last_program_state) {
      PumpStopAll();
      String_StopAll();                 /* all four spools, nodes 3/4 + 5/6 */
      Grasp_StopNow();                  /* nodes 7/8 run continuously       */
      Arm_OnProgramStateChange();
      last_program_state = program_state;
    }

    if (program_state == 0) {
      Bench_Task();                 // manual pump / motor controls
    }
    else if (program_state == 1) {
      /* Arm state machine. Each case is one call; the bodies live in
         arm_states.c. Arm_BeginPass() does the per-pass housekeeping:
         stop whatever the previous state left running and re-arm the
         one-shot latch on every arm_state change. */
      Arm_BeginPass();

      switch (arm_state)
      {
        case 1:   ArmState_SDMHome();             break;   /* servo homing         */
        case 2:   ArmState_Home();                break;   /* pull back UNTIL STALLING*/
        case 3:   ArmState_Fill();             break;   /* fill + pay out slack */
        case 4:   ArmState_Grasp();            break;   /* grasp / release      */
        case 5:   ArmState_SeparateSliders();  break;   /* separate + reel back */
        case 6:   ArmState_JointRotate();      break;   /* joint rotation       */
        case 7:   ArmState_ServoAngle();       break;   /* big servo to angle   */
        case 8:   ArmState_Drain();            break;   /* pump out             */
        case 9:   ArmState_Hold();             break;   /* hold + pressure      */
        case 10:  ArmState_InitSlider();       break;   /* pull back UNTIL STALLING*/
        case 12:  ArmState_MoveToPosition();   break;   /* closed-loop position */
        case 13:  ArmState_InitSmallSlider(); break;   /* Init section-forming slider */
        case 14:  ArmState_SectionInit();     break;   /* section-forming slider init, pressure-held */
        case 0:
        default:  ArmState_Idle();             break;   /* everything off       */
      }
    }

    /* SDM8070 servo (FDCAN2) MUST run every loop, in every program_state:
       it drives homing, feedback mirror, jog, moves and holds. */
    SDM_ServoTask();

    HAL_Delay(200);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                               | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
                               | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  */
static void MX_FDCAN1_Init(void)
{
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  /* 125 kbps tendon bus (0xC1-0xC4 CanMessage_t drives run at 125K).
   * FDCAN kernel clock = 96 MHz (PLL). 96e6 / (48 * (1 + 13 + 2)) = 125 kbps.
   * Sample point = (1 + 13) / 16 = 87.5% (CAN-standard).
   * NOTE: every node on this wire must be 125K. If any UIM242 drive is still
   * set to 800K it will go silent here - reconfigure it in StepEva to 125K. */
  hfdcan1.Init.NominalPrescaler = 48;
  hfdcan1.Init.NominalSyncJumpWidth = 1;   // matches the old 125k config exactly
  hfdcan1.Init.NominalTimeSeg1 = 13;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 1;
  hfdcan1.Init.RxFifo0ElmtsNbr = 4;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 4;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN2 Initialization Function — SDM8070JA1 servo bus
  *
  * Separate port from the UIM242 bus so the two bit rates can coexist:
  * FDCAN1 = 800 kbps (UIM242, extended IDs), FDCAN2 = 1 Mbps (SDM8070,
  * standard 11-bit IDs). Keeping them apart means this motor cannot disturb
  * the nodes 7-10 bus.
  *
  * Timing: FDCAN kernel clock = 96 MHz (shared PLL source with FDCAN1).
  *   96e6 / (6 * (1 + 12 + 3)) = 96e6 / 96 = 1 Mbps
  *   Sample point = (1 + 12) / 16 = 81.25%
  *
  * MessageRAMOffset: the FDCAN message RAM is SHARED between peripherals, so
  * FDCAN2 must not overlap FDCAN1's region. FDCAN1 uses 34 words
  * (1 ext filter x2 + RxFIFO0 4x4 + TxFIFO 4x4), so start FDCAN2 at 64.
  */
static void MX_FDCAN2_Init(void)
{
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = ENABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 6;
  hfdcan2.Init.NominalSyncJumpWidth = 3;
  hfdcan2.Init.NominalTimeSeg1 = 12;
  hfdcan2.Init.NominalTimeSeg2 = 3;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 1;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.MessageRAMOffset = 64;      /* must clear FDCAN1's 34 words */
  hfdcan2.Init.StdFiltersNbr = 1;          /* this motor uses STANDARD ids */
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.RxFifo0ElmtsNbr = 8;        /* deeper than FDCAN1's 4 */
  hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxFifo1ElmtsNbr = 0;
  hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxBuffersNbr = 0;
  hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.TxEventsNbr = 0;
  hfdcan2.Init.TxBuffersNbr = 0;
  hfdcan2.Init.TxFifoQueueElmtsNbr = 8;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  */
static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 199;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim1);
}

/**
  * @brief TIM2 Initialization Function - end-tip pumps, same ~960 Hz PWM as TIM1
  */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim2);
}

/**
  * @brief USART2 Initialization Function — flow sensor (PD5 TX, PD6 RX, PD4 DE)
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART3 Initialization Function — pressure sensor (PD8 TX, PD9 RX, PB14 DE)
  */
static void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* Set outputs low before configuring */
  HAL_GPIO_WritePin(GPIOD, ValveBaseIn_Pin | ValveTipOut_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, ValveBaseOut_Pin | ValveTipIn_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RS485_DE_RE_3_GPIO_Port, RS485_DE_RE_3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RS485_DE_RE_2_GPIO_Port, RS485_DE_RE_2_Pin, GPIO_PIN_RESET);

  /* ValveBaseIn (PD7), ValveTipOut (PD10) */
  GPIO_InitStruct.Pin = ValveBaseIn_Pin | ValveTipOut_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* ValveBaseOut (PB10), ValveTipIn (PB11) */
  GPIO_InitStruct.Pin = ValveBaseOut_Pin | ValveTipIn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* RS485 DE/RE for USART3 (pressure sensor) — PB14 */
  GPIO_InitStruct.Pin = RS485_DE_RE_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RS485_DE_RE_3_GPIO_Port, &GPIO_InitStruct);

  /* RS485 DE/RE for USART2 (flow sensor) — PD4 */
  GPIO_InitStruct.Pin = RS485_DE_RE_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RS485_DE_RE_2_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */

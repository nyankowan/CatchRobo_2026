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
#include "can_protocol.h"
#include "coordinate.h"
#include "stm_can.h"
#include <math.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SERVO_0   500
#define SERVO_270 2500

// Status_LEDでLeft->Middle->Right->Expandの順に各chの状態を点滅回数で表示する
// 長いマーカー点灯(周期の開始) -> 各chごとに短い点滅(ON:2回 OFF:1回) -> 一定時間消灯 の繰り返し
#define STATUS_LED_MARKER_MS     1000 //周期の始まりを示す長い点灯
#define STATUS_LED_BLINK_MS      100  //1回分の点滅の点灯時間
#define STATUS_LED_INTRA_GAP_MS  100  //ONの2回点滅の間の消灯時間
#define STATUS_LED_CHANNEL_GAP_MS 700 //ch同士の間，およびマーカー直後の消灯時間(繋がって見えないように)
#define STATUS_LED_END_PAUSE_MS  1200 //4ch分表示し終えてから次のマーカーまでの消灯時間
#define LOOP_MS 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
CAN_RxHeaderTypeDef rx_header;
can_data_t rx_data = {0};

// 現在のLeft/Middle/Right/Expandの状態(true=ON)．Status_LEDの点滅表示に使う
bool lower_arm_left, lower_arm_middle, lower_arm_right, lower_arm_expand;

typedef enum{
  STATUS_LED_STATE_MARKER,          //周期開始の長い点灯
  STATUS_LED_STATE_CHANNEL_GAP,     //ch移行時(マーカー直後含む)の消灯
  STATUS_LED_STATE_BLINK_ON,        //点滅の点灯中
  STATUS_LED_STATE_BLINK_GAP,       //点滅の点灯の間，または最後の点滅後の消灯
  STATUS_LED_STATE_END_PAUSE,       //4ch分表示し終えた後の消灯
}status_led_state_t;

status_led_state_t status_led_state;
uint32_t status_led_phase_start;
uint8_t status_led_channel_index;     //現在表示中のch(0:Left 1:Middle 2:Right 3:Expand)
uint8_t status_led_blinks_remaining;  //現在のchで残っている点滅回数

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
  if(hcan->Instance != CAN)return;
  if(HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_header,rx_data.raw) != HAL_OK)return;

  switch (rx_header.StdId) {
  case CAN_ID_LOWER_ARM_COMMAND:
    lower_arm_left   = rx_data.lower_arm.left;
    lower_arm_middle = rx_data.lower_arm.middle;
    lower_arm_right  = rx_data.lower_arm.right;
    lower_arm_expand = rx_data.lower_arm.expand;

    if(rx_data.lower_arm.left)  {__HAL_TIM_SET_COMPARE(&Left_htim,Left_TIM_CHANNEL,SERVO_270);}else{__HAL_TIM_SET_COMPARE(&Left_htim,Left_TIM_CHANNEL,SERVO_0);}
    if(rx_data.lower_arm.middle){__HAL_TIM_SET_COMPARE(&Middle_htim,Middle_TIM_CHANNEL,SERVO_270);}else{__HAL_TIM_SET_COMPARE(&Middle_htim,Middle_TIM_CHANNEL,SERVO_0);}
    if(rx_data.lower_arm.right) {__HAL_TIM_SET_COMPARE(&Right_htim,Right_TIM_CHANNEL,SERVO_270);}else{__HAL_TIM_SET_COMPARE(&Right_htim,Right_TIM_CHANNEL,SERVO_0);}
    if(rx_data.lower_arm.expand){__HAL_TIM_SET_COMPARE(&Expand_htim,Expand_TIM_CHANNEL,SERVO_270);}else{__HAL_TIM_SET_COMPARE(&Expand_htim,Expand_TIM_CHANNEL,SERVO_0);}
    direct_t direct = {.x = rx_data.lower_arm.x, .y = rx_data.lower_arm.y};
    __HAL_TIM_SET_COMPARE(&Shaft_htim,Shaft_TIM_CHANNEL,to_polar(direct).theta * (SERVO_270 - SERVO_0) / (3 * M_PI_2) + SERVO_0);
    break;

  case CAN_ID_LOWER_HOMING:
    lower_arm_left = false;
    lower_arm_middle = false;
    lower_arm_right = false;
    lower_arm_expand = false;

    __HAL_TIM_SET_COMPARE(&Left_htim,Left_TIM_CHANNEL,SERVO_0);
    __HAL_TIM_SET_COMPARE(&Middle_htim,Middle_TIM_CHANNEL,SERVO_0);
    __HAL_TIM_SET_COMPARE(&Right_htim,Right_TIM_CHANNEL,SERVO_0);
    __HAL_TIM_SET_COMPARE(&Expand_htim,Expand_TIM_CHANNEL,SERVO_0);
    __HAL_TIM_SET_COMPARE(&Shaft_htim,Shaft_TIM_CHANNEL,SERVO_0);
    break;
  

  default:
    break;
  }
}

/**
* @brief chのindex(0:Left 1:Middle 2:Right 3:Expand)から現在の状態(true=ON)を返す．
*/
static bool status_led_channel_value(uint8_t channel_index){
  switch(channel_index){
    case 0: return lower_arm_left;
    case 1: return lower_arm_middle;
    case 2: return lower_arm_right;
    default: return lower_arm_expand;
  }
}

/**
* @brief Status_LEDに，先頭の長いマーカー点灯に続けてLeft->Middle->Right->Expandの
*        順で各chの状態を点滅回数で表示する．ONは2回，OFFは1回の短い点滅．
*        ch同士の間には必ず消灯を挟むので，何回点滅したかを数えればON/OFFが分かる．
*        4ch分表示し終えたら一定時間消灯してから，再びマーカーに戻る．
*        周期的(mainループ毎)に呼び出すこと．
*/
void status_led_update(){
  uint32_t now = HAL_GetTick();
  uint32_t elapsed = now - status_led_phase_start;

  switch(status_led_state){
  case STATUS_LED_STATE_MARKER:
    if(elapsed >= STATUS_LED_MARKER_MS){
      status_led_state = STATUS_LED_STATE_CHANNEL_GAP;
      status_led_channel_index = 0;
      status_led_phase_start = now;
    }
    break;

  case STATUS_LED_STATE_CHANNEL_GAP:
    if(elapsed >= STATUS_LED_CHANNEL_GAP_MS){
      status_led_blinks_remaining = status_led_channel_value(status_led_channel_index) ? 2 : 1;
      status_led_state = STATUS_LED_STATE_BLINK_ON;
      status_led_phase_start = now;
    }
    break;

  case STATUS_LED_STATE_BLINK_ON:
    if(elapsed >= STATUS_LED_BLINK_MS){
      status_led_blinks_remaining--;
      status_led_state = STATUS_LED_STATE_BLINK_GAP;
      status_led_phase_start = now;
    }
    break;

  case STATUS_LED_STATE_BLINK_GAP:
    if(status_led_blinks_remaining > 0){
      // 同じchの2回目の点滅がまだ残っている
      if(elapsed >= STATUS_LED_INTRA_GAP_MS){
        status_led_state = STATUS_LED_STATE_BLINK_ON;
        status_led_phase_start = now;
      }
    }else{
      // このchの表示は終わり
      if(elapsed >= STATUS_LED_CHANNEL_GAP_MS){
        status_led_channel_index++;
        status_led_phase_start = now;
        if(status_led_channel_index >= 4){
          status_led_state = STATUS_LED_STATE_END_PAUSE;
        }else{
          status_led_blinks_remaining = status_led_channel_value(status_led_channel_index) ? 2 : 1;
          status_led_state = STATUS_LED_STATE_BLINK_ON;
        }
      }
    }
    break;

  case STATUS_LED_STATE_END_PAUSE:
    if(elapsed >= STATUS_LED_END_PAUSE_MS){
      status_led_state = STATUS_LED_STATE_MARKER;
      status_led_phase_start = now;
    }
    break;
  }

  bool on = (status_led_state == STATUS_LED_STATE_MARKER) ||
            (status_led_state == STATUS_LED_STATE_BLINK_ON);
  HAL_GPIO_WritePin(
    STATUS_LED_GPIO_Port,
    STATUS_LED_Pin,
    on ? GPIO_PIN_SET : GPIO_PIN_RESET
  );
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

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_CAN_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  
  HAL_TIM_PWM_Start(&htim3, Left_TIM_CHANNEL);
  HAL_TIM_PWM_Start(&htim3, Middle_TIM_CHANNEL);
  HAL_TIM_PWM_Start(&htim3, Right_TIM_CHANNEL);
  HAL_TIM_PWM_Start(&htim3, Expand_TIM_CHANNEL);

  HAL_TIM_PWM_Start(&htim1, Shaft_TIM_CHANNEL);

  HAL_CAN_Start(&hcan);
  if (HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
    Error_Handler();
  }
  HAL_GPIO_WritePin(
    STATUS_LED_GPIO_Port,
    STATUS_LED_Pin,
    GPIO_PIN_SET
  );
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_heartbeat = HAL_GetTick();
  while (1)
  {
    status_led_update();

    if(HAL_GetTick() - last_heartbeat > HEARTBEAT_MS){
      last_heartbeat = HAL_GetTick();
      stm_can_send(&hcan, &(can_command_data_t){.id = CAN_ID_LOWER_ARM_HEARTBEAT});
    }
    /* USER CODE END WHILE */
    HAL_Delay(LOOP_MS);
    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 1;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  /* can_rx setting */
  CAN_FilterTypeDef filter;

  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;

  /* 受信IDフィルター　全受信 */
  filter.FilterIdHigh = 0x0000;
  filter.FilterIdLow = 0x0000;
  filter.FilterMaskIdHigh = 0x0000;
  filter.FilterMaskIdLow = 0x0000;

  filter.FilterFIFOAssignment = CAN_RX_FIFO0; //fifo(first-in first-out) = Queue
  filter.FilterActivation = ENABLE;

  if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK) {
      Error_Handler();
  }
  /* can_rx setting */
  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 8-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000-1;
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
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : STATUS_LED_Pin */
  GPIO_InitStruct.Pin = STATUS_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STATUS_LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
    HAL_GPIO_TogglePin(
    STATUS_LED_GPIO_Port,
    STATUS_LED_Pin
    );
    HAL_Delay(1000);
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
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
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"
#include "stm_can.h"
#include <math.h>
#include <stdint.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
    uint16_t angle_raw;

    int32_t total_angle;

    int16_t rpm;
    int16_t current;
    uint8_t temperature;

    uint16_t last_angle; //to calc total_angle

    uint32_t last_update; // 最後に受信した時間 HAL_GetTick()

} robomas_feedback_t;

typedef struct{
  double p;   //比例ゲイン
  double i;   //積分ゲイン
  double d;   //微分ゲイン
  double mv;  //manipulated value 操作量
  double sv;  //set value 目標値
  double it;  //integral term 積分項 偏差の総和
  double pe;  //previous error 直前の偏差
  double pv;  //process value 制御量 フィードバック
  uint32_t last_update;  // 最後に更新した時間 HAL_GetTick()
}pid_t;

typedef enum{
  ROBOMAS_HOMING, // 速度制御によって，リミットスイッチ位置まで回転する
  ROBOMAS_IDLE,   // ホーミング終了による他のホーミングを待機
  ROBOMAS_READY,  // ユーザが制御可能な状態
}robomas_state_t;

typedef struct{
  robomas_feedback_t feedback;
  pid_t pid;
  robomas_state_t state;

}robomas_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define R_ROBOMAS_DIAMETER 30 //アーム長ロボマスにつくギアの直径(mm)
#define LOWER_R_MIN 200 //ToDo: アーム長を一番短くしたときのR(mm)を測る
#define POLAR_RATIO 2.6666666 //= 8/3 アーム角度ロボマスの直径比

#define ARM_HOME_COORDINATE {\
  .x = LOWER_R_MIN,\
  .y = 0,\
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
direct_t coordinate[2] = {ARM_HOME_COORDINATE, ARM_HOME_COORDINATE};
#define coordinate_lower coordinate[0]
#define coordinate_upper coordinate[1]

//Sending robomas value setting
int16_t robomas_tx_torque[4]={0,0,0,0}; // {ID.1,ID.2,ID.3,ID.4}
#define robomas_lower_r_tx_torque robomas_tx_torque[0]
#define robomas_lower_deg_tx_torque robomas_tx_torque[1]
#define robomas_upper_r_tx_torque robomas_tx_torque[2]
#define robomas_upper_deg_tx_torque robomas_tx_torque[3]

robomas_t robomas[4] = {0};
#define robomas_lower_r robomas[0]
#define robomas_lower_deg robomas[1]
#define robomas_upper_r robomas[2]
#define robomas_upper_deg robomas[3]
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void command_receive(can_command_data_t *com);
void robomas_receive(uint8_t robomas_id, uint8_t *data);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];
  if(HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_header,rx_data) != HAL_OK)return;
  if (hcan->Instance == COMMAND_CAN) {
    can_command_data_t com = {
      .id = rx_header.StdId,
    };
    memcpy(com.data.raw, rx_data, rx_header.DLC);
    command_receive(&com);
  }else if (hcan->Instance == ROBOMAS_CAN) {
    if (rx_header.IDE != CAN_ID_STD || (rx_header.StdId < 0x201 || rx_header.StdId > 0x204))return;
    uint8_t robomas_id = rx_header.StdId - 0x201; // 201→0, 202→1, 203→2
    robomas_receive(robomas_id, rx_data);
  }
}

void command_receive(can_command_data_t *com){
  switch (com->id) {
  case CAN_ID_LOWER_ARM_COMMAND:
    //com->data.lower_arm

  break;  
  case CAN_ID_UPPER_ARM_COMMAND:
    //com->data.upper_arm

  break;
  case CAN_ID_UPPER_HOMING:
    
  break;
  case CAN_ID_LOWER_HOMING:

  break;

  default:
  break;
  }
}

void robomas_receive(uint8_t robomas_id, uint8_t *data){
// Robomasterからのフィードバック
  robomas[robomas_id].feedback.angle_raw =   ((uint16_t)data[0] << 8) | data[1];
  robomas[robomas_id].feedback.rpm =         ((int16_t)data[2] << 8) | data[3];
  robomas[robomas_id].feedback.current =     ((int16_t)data[4] << 8) | data[5];
  robomas[robomas_id].feedback.temperature = data[6]; 
  
  int16_t delta = robomas[robomas_id].feedback.angle_raw - robomas[robomas_id].feedback.last_angle;

  if (delta > 4096){
      delta -= 8192;
  }else if (delta < -4096){
      delta += 8192;
  }

  robomas[robomas_id].feedback.total_angle += delta;
  robomas[robomas_id].feedback.last_angle = robomas[robomas_id].feedback.angle_raw;
  robomas[robomas_id].feedback.last_update = HAL_GetTick();// temp
}

void robomas_send_torque(int16_t *torque){
  if(0 < HAL_CAN_GetTxMailboxesFreeLevel(&ROBOMAS_HCAN)){
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t TxData[8];
    TxHeader.StdId = 0x200;                 // CAN ID
    TxHeader.RTR = CAN_RTR_DATA;            // フレームタイプはデータフレーム
    TxHeader.IDE = CAN_ID_STD;              // 標準ID(11ﾋﾞｯﾄ)
    TxHeader.DLC = 8;                       // データ長は8バイトに
    TxHeader.TransmitGlobalTime = DISABLE;  // ???
    TxData[0] = torque[0] >> 8 & 0x00FF;
    TxData[1] = torque[0] & 0x00FF;
    TxData[2] = torque[1] >> 8 & 0x00FF;
    TxData[3] = torque[1] & 0x00FF;
    TxData[4] = torque[2] >> 8 & 0x00FF;
    TxData[5] = torque[2] & 0x00FF;
    TxData[6] = torque[3] >> 8 & 0x00FF;
    TxData[7] = torque[3] & 0x00FF;   
    HAL_CAN_AddTxMessage(&ROBOMAS_HCAN, &TxHeader, TxData, &TxMailbox);
  }
}
/**
* @brief 目標値spに基づいてpid式で操作量mvを計算する．
* @return 操作量を返す
  double p;   //比例ゲイン
  double i;   //積分ゲイン
  double d;   //微分ゲイン
  double mv;  //manipulated value 操作量
  double sv;  //set value 目標値
  double it;  //integral term 積分項 偏差の総和
  double pe;  //previous error 直前の偏差
  double pv;  //process value 制御量 フィードバック
*/
double calc_pid(pid_t *pid){
  uint32_t now = HAL_GetTick();
  if(now == pid->last_update)return pid->mv;
  double delta_sec = (now - pid->last_update)/1000.0;
  double e =  pid->sv - pid->pv; //erro 偏差
  double dt = (e - pid->pe)/delta_sec; //derivative term 微分項
  pid->it += e * delta_sec;
  pid->mv = pid->p * e + pid->i * pid->it + pid->d * dt;
  pid->pe = e;
  pid->last_update = now;
  return pid->mv;
}

/**
* @brief 積分項，直前の偏差，操作量をリセット．last_updateも更新する．
*/
void pid_reset(pid_t *pid){
    pid->it = 0.0;
    pid->pe = 0.0;
    pid->mv = 0.0;
    pid->last_update = HAL_GetTick();
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
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_CAN_Start(&hcan1);
  HAL_CAN_Start(&hcan2);

  HAL_CAN_ActivateNotification(
      &hcan1,
      CAN_IT_RX_FIFO0_MSG_PENDING
  );

  HAL_CAN_ActivateNotification(
      &hcan2,
      CAN_IT_RX_FIFO0_MSG_PENDING
  );
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    stm_can_send(&COMMAND_HCAN, &(can_command_data_t){.id = CAN_ID_ROBOMAS_CONTROLLER_HEARTBEAT});
    HAL_Delay(HEARTBEAT_MS);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 2;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  CAN_FilterTypeDef filter = {0};

  filter.FilterBank = 0;
  filter.SlaveStartFilterBank = 14;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0;
  filter.FilterIdLow = 0;
  filter.FilterMaskIdHigh = 0;
  filter.FilterMaskIdLow = 0;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;

  HAL_CAN_ConfigFilter(&hcan1, &filter);
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 2;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  CAN_FilterTypeDef filter = {0};
  filter.FilterBank = 14;
  filter.SlaveStartFilterBank = 14;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0;
  filter.FilterIdLow = 0;
  filter.FilterMaskIdHigh = 0;
  filter.FilterMaskIdLow = 0;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;

  HAL_CAN_ConfigFilter(&hcan2, &filter);
  /* USER CODE END CAN2_Init 2 */

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Status_LED_GPIO_Port, Status_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : UPPER_ARM_R_LIMIT_Pin UPPER_ARM_DEG_UNDER_LIMIT_Pin UPPER_ARM_DEG_OVER_LIMIT_Pin LOWER_ARM_R_LIMIT_Pin
                           LOWER_ARM_DEG_UNDER_LIMIT_Pin LOWER_ARM_DEG_OVER_LIMIT_Pin */
  GPIO_InitStruct.Pin = UPPER_ARM_R_LIMIT_Pin|UPPER_ARM_DEG_UNDER_LIMIT_Pin|UPPER_ARM_DEG_OVER_LIMIT_Pin|LOWER_ARM_R_LIMIT_Pin
                          |LOWER_ARM_DEG_UNDER_LIMIT_Pin|LOWER_ARM_DEG_OVER_LIMIT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Status_LED_Pin */
  GPIO_InitStruct.Pin = Status_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Status_LED_GPIO_Port, &GPIO_InitStruct);

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

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
#include "arm.h"
#include "can_protocol.h"
#include "coordinate.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm_can.h"
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
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

    bool initialized; //初回ロボマスフィードバックを受けたか
} robomas_feedback_t;

/**
* @param double p;   比例ゲイン 
* @param double i;   積分ゲイン
* @param double d;   微分ゲイン
* @param double mv;  manipulated value 操作量
* @param double sv;  set value 目標値
* @param double it;  integral term 積分項 偏差の総和
* @param double pe;  previous error 直前の偏差
* @param double pv;  process value 制御量 フィードバック
* @param uint32_t last_update;   最後に更新した時間 HAL_GetTick()
**/
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

/**
* @param ROBOMAS_INITAL, // 何もトルクを送らない状態
* @param ROBOMAS_HOMING, // 速度制御によって，リミットスイッチ位置まで回転する
* @param ROBOMAS_IDLE,   // ホーミング終了による他のホーミングを待機
* @param ROBOMAS_READY,  // ユーザが制御可能な状態
* @param ROBOMAS_ERROR,  // フィードバックを受け取れず，制御不能な状態
 */
typedef enum{
  ROBOMAS_INITIAL,  // 初回起動時状態
  ROBOMAS_HOMING,   // 速度制御によって，リミットスイッチ位置まで回転する
  ROBOMAS_IDLE,     // ホーミング終了による他のホーミングを待機
  ROBOMAS_READY,    // ユーザが制御可能な状態
  ROBOMAS_ERROR,    // 制御不能状態
}robomas_state_t;

/**
* @param robomas_feedback_t feedback
* @param pid_t rpm_pid   モーターrpmの制御 
* @param pid_t ang_pid   モーター角(resolution 8192)の制御 
* @param robomas_state_t state     ロボマス制御状態
* @param in32_t total_angle_home 極座標の原点に対応するモーター角(一周あたりの解像度 8192)
* @param int16_t tx_torque     ロボマスへ送るトルク値
* @param double gear_ratio    遊星軸/モーター軸 ギア比
*/
typedef struct{
  robomas_feedback_t feedback;
  pid_t rpm_pid;  //モーターrpmの制御
  pid_t ang_pid;  //モーター角(resolution 8192)の制御
  robomas_state_t state;
  int32_t total_angle_home;//極座標の原点に対応するモーター角(一周あたりの解像度 8192)
  int16_t tx_torque;
  double gear_ratio;  // 遊星軸/モーター軸 ギア比

}robomas_t;

uint32_t upper_homing_start_time;
uint32_t lower_homing_start_time;

//(HOMING -> HOMING_ACK -> ... -> HOMING_DONE -> HOMING_DONE_ACK)
can_sequence_t upper_homing_sequence;
can_sequence_t lower_homing_sequence;

bool upper_homing_done_ack_pending; // HOMING_DONEを送信し，HOMING_DONE_ACK待ちかどうか
bool lower_homing_done_ack_pending;
uint32_t upper_homing_done_sent_time;
uint32_t lower_homing_done_sent_time;
uint8_t upper_homing_done_retry_count;
uint8_t lower_homing_done_retry_count;

// 一度のホーミング試行につき，タイムアウトのCAN_ID_ERROR_CODEは1回しか送らない
bool upper_homing_timeout_notified;
bool lower_homing_timeout_notified;

#define HOMING_DONE_ACK_RETRY_MS 200   //この間隔でHOMING_DONEを再送する
#define HOMING_DONE_ACK_MAX_RETRY 5    //再送を諦めるまでの回数

// Status_LED点滅パターン用
// lowerがREADYでない:+1 upperがREADYでない:+2 (両方でないなら3回点滅) 両方READYなら常時点灯
#define STATUS_LED_BLINK_ON_MS   150
#define STATUS_LED_BLINK_OFF_MS  150
#define STATUS_LED_BLINK_PAUSE_MS 700
uint8_t status_led_blink_index; //現在の点滅サイクルで何回点滅したか
bool status_led_on;
uint32_t status_led_phase_start;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LOOP_MS 2
#define ROBOMAS_FEEDBACK_TIMEOUT_MS 100

#define ROBOMAS_NUM 4

#define ROBOMAS_MAX_TORQUE 16384 //ロボマスに送れる最大電流値
#define ROBOMAS_M3_GEAR_RATIO (3591.0 / 187.0)//M3508
#define ROBOMAS_M2_GEAR_RATIO 36              //M2006
#define ROBOMAS_ANGLE_RESOLUTION 8192 //0〜8191

#define ARM_DEG_ROBOMAS_DIRECTION 1 //上から見て半時計回りが正でモーターは右ねじを正とするとき
#define ARM_R_ROBOMAS_DIRECTION -1 //アームが伸びる方向が正でモーター右ねじ正


//ToDo: pullupしているので，導通したらRESET，していないならSETになる．リミットスイッチの接続によって変える．
//      リミットスイッチ：COM NO NC の三つのコネクタがあり，COMは常に接続されている．NOはスイッチを押すと導通，NCはスイッチを離すと導通．
#define LOWER_ARM_DEG_UNDER_LIMIT_ON (HAL_GPIO_ReadPin(LOWER_ARM_DEG_UNDER_LIMIT_GPIO_Port, LOWER_ARM_DEG_UNDER_LIMIT_Pin) == GPIO_PIN_RESET)
#define LOWER_ARM_DEG_UNDER_LIMIT_OFF !LOWER_ARM_DEG_UNDER_LIMIT_ON

#define LOWER_ARM_R_LIMIT_ON (HAL_GPIO_ReadPin(LOWER_ARM_R_LIMIT_GPIO_Port, LOWER_ARM_R_LIMIT_Pin) == GPIO_PIN_RESET)
#define LOWER_ARM_R_LIMIT_OFF !LOWER_ARM_R_LIMIT_ON

#define UPPER_ARM_DEG_UNDER_LIMIT_ON (HAL_GPIO_ReadPin(UPPER_ARM_DEG_UNDER_LIMIT_GPIO_Port, UPPER_ARM_DEG_UNDER_LIMIT_Pin) == GPIO_PIN_RESET)
#define UPPER_ARM_DEG_UNDER_LIMIT_OFF !UPPER_ARM_DEG_UNDER_LIMIT_ON

#define UPPER_ARM_R_LIMIT_ON (HAL_GPIO_ReadPin(UPPER_ARM_R_LIMIT_GPIO_Port, UPPER_ARM_R_LIMIT_Pin) == GPIO_PIN_RESET)
#define UPPER_ARM_R_LIMIT_OFF !UPPER_ARM_R_LIMIT_ON

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
direct_t coordinate[ARM_NUM] = {LOWER_ARM_HOME_COORDINATE, UPPER_ARM_HOME_COORDINATE};
#define coordinate_lower coordinate[0]
#define coordinate_upper coordinate[1]

robomas_t robomas[ROBOMAS_NUM] = {
  { .state = ROBOMAS_INITIAL,
    .gear_ratio = ROBOMAS_M3_GEAR_RATIO,
    .rpm_pid = {
    .p = 10, .i = 0, .d = 0,
  }, .ang_pid = {
    .p = 0.1, .i = 0, .d = 0,
  }},
  { .state = ROBOMAS_INITIAL,
    .gear_ratio = ROBOMAS_M3_GEAR_RATIO,
    .rpm_pid = {
    .p = 10, .i = 0, .d = 0,
  }, .ang_pid = {
    .p = 0.1, .i = 0, .d = 0,
  }},
  { .state = ROBOMAS_INITIAL,
    .gear_ratio = ROBOMAS_M3_GEAR_RATIO,
    .rpm_pid = {
    .p = 10, .i = 0, .d = 0,
  }, .ang_pid = {
    .p = 0.1, .i = 0, .d = 0,
  }},
  { .state = ROBOMAS_INITIAL,
    .gear_ratio = ROBOMAS_M3_GEAR_RATIO,
    .rpm_pid = {
    .p = 10, .i = 0, .d = 0,
  }, .ang_pid = {
    .p = 0.1, .i = 0, .d = 0,
  }},
};
#define robomas_lower_deg robomas[0]
#define robomas_lower_r robomas[1]
#define robomas_upper_deg robomas[2]
#define robomas_upper_r robomas[3]
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
void pid_reset(pid_t *pid);
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
    if(robomas_lower_r.state != ROBOMAS_READY || robomas_lower_deg.state != ROBOMAS_READY)return;
    coordinate_lower.x = com->data.lower_arm.x;
    coordinate_lower.y = com->data.lower_arm.y;
    break;  
  case CAN_ID_UPPER_ARM_COMMAND:
    if(robomas_upper_r.state != ROBOMAS_READY || robomas_upper_deg.state != ROBOMAS_READY)return;
    coordinate_upper.x = com->data.upper_arm.x;
    coordinate_upper.y = com->data.upper_arm.y;
    break;
  case CAN_ID_UPPER_HOMING:
    if(!robomas_upper_deg.feedback.initialized || !robomas_upper_r.feedback.initialized)return;
    if(robomas_upper_r.state == ROBOMAS_IDLE || robomas_upper_deg.state == ROBOMAS_IDLE){
      // 既に受理済みの要求の再送(ACK消失)ならACKを返し直す．新規の要求は無視する．
      if(com->data.homing_sequence == upper_homing_sequence){
        stm_can_send(&COMMAND_HCAN, &(can_command_data_t){
          .id = CAN_ID_UPPER_HOMING_ACK,
          .data.homing_sequence = upper_homing_sequence,
        });
      }
      return;
    }
    if(robomas_upper_r.state == ROBOMAS_ERROR || robomas_upper_deg.state == ROBOMAS_ERROR){
      stm_can_send(&COMMAND_HCAN, &(can_command_data_t){
        .id = CAN_ID_ERROR_CODE,
        .data.error_code = CAN_ERROR_UPPER_HOMING_REJECTED,
      });
      return;
    }
    //INITAIL -> HOMING or READY -> HOMING
    upper_homing_sequence = com->data.homing_sequence;
    upper_homing_done_ack_pending = false; // 前回分の再送待ちが残っていたら破棄
    upper_homing_timeout_notified = false;
    pid_reset(&robomas_upper_r.rpm_pid);
    pid_reset(&robomas_upper_deg.rpm_pid);
    robomas_upper_r.state = ROBOMAS_HOMING;
    robomas_upper_deg.state = ROBOMAS_HOMING;
    upper_homing_start_time = HAL_GetTick();

    stm_can_send(&COMMAND_HCAN, &(can_command_data_t){
      .id = CAN_ID_UPPER_HOMING_ACK,
      .data.homing_sequence = upper_homing_sequence,
    });
    break;
  case CAN_ID_LOWER_HOMING:
    if(!robomas_lower_deg.feedback.initialized || !robomas_lower_r.feedback.initialized)return;
    if(robomas_lower_r.state == ROBOMAS_IDLE || robomas_lower_deg.state == ROBOMAS_IDLE){
      // 既に受理済みの要求の再送(ACK消失)ならACKを返し直す．新規の要求は無視する．
      if(com->data.homing_sequence == lower_homing_sequence){
        stm_can_send(&COMMAND_HCAN, &(can_command_data_t){
          .id = CAN_ID_LOWER_HOMING_ACK,
          .data.homing_sequence = lower_homing_sequence,
        });
      }
      return;
    }
    if(robomas_lower_r.state == ROBOMAS_ERROR || robomas_lower_deg.state == ROBOMAS_ERROR){
      stm_can_send(&COMMAND_HCAN, &(can_command_data_t){
        .id = CAN_ID_ERROR_CODE,
        .data.error_code = CAN_ERROR_LOWER_HOMING_REJECTED,
      });
      return;
    }
    //INITAIL -> HOMING or READY -> HOMING
    lower_homing_sequence = com->data.homing_sequence;
    lower_homing_done_ack_pending = false; // 前回分の再送待ちが残っていたら破棄
    lower_homing_timeout_notified = false;
    pid_reset(&robomas_lower_r.rpm_pid);
    pid_reset(&robomas_lower_deg.rpm_pid);
    robomas_lower_r.state = ROBOMAS_HOMING;
    robomas_lower_deg.state = ROBOMAS_HOMING;
    lower_homing_start_time = HAL_GetTick();

    stm_can_send(&COMMAND_HCAN, &(can_command_data_t){
      .id = CAN_ID_LOWER_HOMING_ACK,
      .data.homing_sequence = lower_homing_sequence,
    });
    break;
  case CAN_ID_UPPER_HOMING_DONE_ACK:
    if(com->data.homing_sequence == upper_homing_sequence){
      upper_homing_done_ack_pending = false;
    }
    break;
  case CAN_ID_LOWER_HOMING_DONE_ACK:
    if(com->data.homing_sequence == lower_homing_sequence){
      lower_homing_done_ack_pending = false;
    }
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

  if(!robomas[robomas_id].feedback.initialized){
    robomas[robomas_id].feedback.total_angle = robomas[robomas_id].feedback.angle_raw;
    robomas[robomas_id].feedback.last_angle = robomas[robomas_id].feedback.angle_raw;
    robomas[robomas_id].feedback.initialized = true;
    robomas[robomas_id].feedback.last_update = HAL_GetTick();
    return;
  }

  int16_t delta = robomas[robomas_id].feedback.angle_raw - robomas[robomas_id].feedback.last_angle;

  if (delta > ROBOMAS_ANGLE_RESOLUTION / 2){
      delta -= ROBOMAS_ANGLE_RESOLUTION;
  }else if (delta < -ROBOMAS_ANGLE_RESOLUTION / 2){
      delta += ROBOMAS_ANGLE_RESOLUTION;
  }

  robomas[robomas_id].feedback.total_angle += delta;
  robomas[robomas_id].feedback.last_angle = robomas[robomas_id].feedback.angle_raw;
  robomas[robomas_id].feedback.last_update = HAL_GetTick();
}

void robomas_send_torque(robomas_t rb[]){
  for(int i = 0; i < ROBOMAS_NUM; i++){
    if(rb[i].tx_torque > ROBOMAS_MAX_TORQUE)rb[i].tx_torque = ROBOMAS_MAX_TORQUE;
    if(rb[i].tx_torque < -ROBOMAS_MAX_TORQUE)rb[i].tx_torque = -ROBOMAS_MAX_TORQUE;
  }
  if(0 < HAL_CAN_GetTxMailboxesFreeLevel(&ROBOMAS_HCAN)){
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t TxData[8];
    TxHeader.StdId = 0x200;                 // CAN ID
    TxHeader.RTR = CAN_RTR_DATA;            // フレームタイプはデータフレーム
    TxHeader.IDE = CAN_ID_STD;              // 標準ID(11ﾋﾞｯﾄ)
    TxHeader.DLC = 8;                       // データ長は8バイトに
    TxHeader.TransmitGlobalTime = DISABLE;  // ???
    TxData[0] = rb[0].tx_torque >> 8 & 0x00FF;
    TxData[1] = rb[0].tx_torque & 0x00FF;
    TxData[2] = rb[1].tx_torque >> 8 & 0x00FF;
    TxData[3] = rb[1].tx_torque & 0x00FF;
    TxData[4] = rb[2].tx_torque >> 8 & 0x00FF;
    TxData[5] = rb[2].tx_torque & 0x00FF;
    TxData[6] = rb[3].tx_torque >> 8 & 0x00FF;
    TxData[7] = rb[3].tx_torque & 0x00FF;   
    HAL_CAN_AddTxMessage(&ROBOMAS_HCAN, &TxHeader, TxData, &TxMailbox);
  }
}
/**
* @brief 目標値sp,制御量pvに基づいてpid式で操作量mvを計算し，結果をmvに格納する
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

/*in main roop begin*/
void upper_homing(){
  // HOMING_DONEを送信済みでHOMING_DONE_ACK待ちの場合，一定時間ごとに再送する
  if(upper_homing_done_ack_pending &&
     HAL_GetTick() - upper_homing_done_sent_time > HOMING_DONE_ACK_RETRY_MS){
    if(upper_homing_done_retry_count < HOMING_DONE_ACK_MAX_RETRY){
      stm_can_send(&COMMAND_HCAN, &(can_command_data_t){
        .id = CAN_ID_UPPER_HOMING_DONE,
        .data.homing_sequence = upper_homing_sequence,
      });
      upper_homing_done_sent_time = HAL_GetTick();
      upper_homing_done_retry_count++;
    }else{
      upper_homing_done_ack_pending = false; //諦める
    }
  }

  if(robomas_upper_r.state == ROBOMAS_READY && robomas_upper_deg.state == ROBOMAS_READY)return;

  if(!upper_homing_timeout_notified &&
     (robomas_upper_deg.state == ROBOMAS_HOMING || robomas_upper_deg.state == ROBOMAS_IDLE ||
      robomas_upper_r.state   == ROBOMAS_HOMING || robomas_upper_r.state   == ROBOMAS_IDLE) &&
     HAL_GetTick() - upper_homing_start_time > HOMING_UPPER_ARM_TIMEOUT_MS){
    // ROBOMAS_ERROR中の軸はrobomas_update()が管理するのでここでは触らない
    if(robomas_upper_deg.state != ROBOMAS_ERROR)robomas_upper_deg.state = ROBOMAS_INITIAL;
    if(robomas_upper_r.state   != ROBOMAS_ERROR)robomas_upper_r.state   = ROBOMAS_INITIAL;

    upper_homing_timeout_notified = true;

    stm_can_send(&COMMAND_HCAN, &(can_command_data_t){
      .id = CAN_ID_ERROR_CODE,
      .data.error_code = CAN_ERROR_UPPER_HOMING_TIMEOUT,
    });
  }

  if(robomas_upper_deg.state == ROBOMAS_HOMING && UPPER_ARM_DEG_UNDER_LIMIT_ON){
    robomas_upper_deg.state = ROBOMAS_IDLE;
    robomas_upper_deg.total_angle_home = robomas_upper_deg.feedback.total_angle;
    pid_reset(&robomas_upper_deg.rpm_pid);
  }
    
  if(robomas_upper_r.state == ROBOMAS_HOMING && UPPER_ARM_R_LIMIT_ON){
    robomas_upper_r.state = ROBOMAS_IDLE;
    robomas_upper_r.total_angle_home = robomas_upper_r.feedback.total_angle - ARM_R_ROBOMAS_DIRECTION * UPPER_ARM_R_MIN / (R_ROBOMAS_DIAMETER * M_PI) * ROBOMAS_ANGLE_RESOLUTION;
    pid_reset(&robomas_upper_r.rpm_pid);
  }

  if(robomas_upper_r.state == ROBOMAS_IDLE && robomas_upper_deg.state == ROBOMAS_IDLE){
    pid_reset(&robomas_upper_r.ang_pid);
    pid_reset(&robomas_upper_deg.ang_pid);
    
    robomas_upper_r.state = ROBOMAS_READY;
    robomas_upper_deg.state = ROBOMAS_READY;

    stm_can_send(&COMMAND_HCAN, &(can_command_data_t){
      .id = CAN_ID_UPPER_HOMING_DONE,
      .data.homing_sequence = upper_homing_sequence,
    });
    upper_homing_done_ack_pending = true;
    upper_homing_done_sent_time = HAL_GetTick();
    upper_homing_done_retry_count = 0;
    return;
  }
}

void lower_homing(){
  // HOMING_DONEを送信済みでHOMING_DONE_ACK待ちの場合，一定時間ごとに再送する
  if(lower_homing_done_ack_pending &&
     HAL_GetTick() - lower_homing_done_sent_time > HOMING_DONE_ACK_RETRY_MS){
    if(lower_homing_done_retry_count < HOMING_DONE_ACK_MAX_RETRY){
      stm_can_send(&COMMAND_HCAN, &(can_command_data_t){
        .id = CAN_ID_LOWER_HOMING_DONE,
        .data.homing_sequence = lower_homing_sequence,
      });
      lower_homing_done_sent_time = HAL_GetTick();
      lower_homing_done_retry_count++;
    }else{
      lower_homing_done_ack_pending = false; //諦める
    }
  }

  if(robomas_lower_r.state == ROBOMAS_READY && robomas_lower_deg.state == ROBOMAS_READY)return;
  if(!lower_homing_timeout_notified &&
     (robomas_lower_deg.state == ROBOMAS_HOMING || robomas_lower_deg.state == ROBOMAS_IDLE ||
      robomas_lower_r.state   == ROBOMAS_HOMING || robomas_lower_r.state   == ROBOMAS_IDLE) &&
     HAL_GetTick() - lower_homing_start_time > HOMING_LOWER_ARM_TIMEOUT_MS){
    // ROBOMAS_ERROR中の軸はrobomas_update()が管理するのでここでは触らない
    if(robomas_lower_deg.state != ROBOMAS_ERROR)robomas_lower_deg.state = ROBOMAS_INITIAL;
    if(robomas_lower_r.state   != ROBOMAS_ERROR)robomas_lower_r.state   = ROBOMAS_INITIAL;

    lower_homing_timeout_notified = true;

    stm_can_send(&COMMAND_HCAN, &(can_command_data_t){
      .id = CAN_ID_ERROR_CODE,
      .data.error_code = CAN_ERROR_LOWER_HOMING_TIMEOUT,
    });
  }

  if(robomas_lower_deg.state == ROBOMAS_HOMING && LOWER_ARM_DEG_UNDER_LIMIT_ON){
    robomas_lower_deg.state = ROBOMAS_IDLE;
    robomas_lower_deg.total_angle_home = robomas_lower_deg.feedback.total_angle;
    pid_reset(&robomas_lower_deg.rpm_pid);
  }
    
  if(robomas_lower_r.state == ROBOMAS_HOMING && LOWER_ARM_R_LIMIT_ON){
    robomas_lower_r.state = ROBOMAS_IDLE;
    robomas_lower_r.total_angle_home = robomas_lower_r.feedback.total_angle  - ARM_R_ROBOMAS_DIRECTION * LOWER_ARM_R_MIN / (R_ROBOMAS_DIAMETER * M_PI) * ROBOMAS_ANGLE_RESOLUTION;
    pid_reset(&robomas_lower_r.rpm_pid);
  }

  if(robomas_lower_r.state == ROBOMAS_IDLE && robomas_lower_deg.state == ROBOMAS_IDLE){
    pid_reset(&robomas_lower_r.ang_pid);
    pid_reset(&robomas_lower_deg.ang_pid);
    
    robomas_lower_r.state = ROBOMAS_READY;
    robomas_lower_deg.state = ROBOMAS_READY;

    stm_can_send(&COMMAND_HCAN, &(can_command_data_t){
      .id = CAN_ID_LOWER_HOMING_DONE,
      .data.homing_sequence = lower_homing_sequence,
    });
    lower_homing_done_ack_pending = true;
    lower_homing_done_sent_time = HAL_GetTick();
    lower_homing_done_retry_count = 0;
    return;
  }
}

double get_r(robomas_t *rb)
{
    double motor_angle =
        rb->feedback.total_angle - rb->total_angle_home;

    return ARM_R_ROBOMAS_DIRECTION
     * motor_angle
     / ROBOMAS_ANGLE_RESOLUTION
     * (R_ROBOMAS_DIAMETER * M_PI);
}

/**
* @brief ホーミングrpmを目標値svにセットする．
* @return 正常なら引数を返す．異常ならNULL． 
*/
robomas_t *set_robomas_homing_rpm(robomas_t *rb){
  if(rb == &robomas_lower_deg){
    rb->rpm_pid.sv = -ARM_DEG_ROBOMAS_DIRECTION * HOMING_LOWER_DEG_RPM * rb->gear_ratio;
  }else if(rb == &robomas_upper_deg){
    rb->rpm_pid.sv = -ARM_DEG_ROBOMAS_DIRECTION * HOMING_UPPER_DEG_RPM * rb->gear_ratio;
  }else if(rb == &robomas_lower_r){
    rb->rpm_pid.sv = -ARM_R_ROBOMAS_DIRECTION * HOMING_LOWER_R_RPM * rb->gear_ratio;
  }else if(rb == &robomas_upper_r){
    rb->rpm_pid.sv = -ARM_R_ROBOMAS_DIRECTION * HOMING_UPPER_R_RPM * rb->gear_ratio;
  }else{return NULL;}
  return rb;
}
/**
* @brief 座標に対応するロボマスの角度(一周あたり8192)を座標から計算し，角度目標値svにセットする．
* @return 正常なら引数を返す．異常ならNULL． 
**/
robomas_t *set_robomas_deg_from_coordinate(robomas_t *rb){
  if(rb == &robomas_lower_deg){
    rb->ang_pid.sv = ARM_DEG_ROBOMAS_DIRECTION * to_polar(coordinate_lower).theta / (2 * M_PI) * POLAR_RATIO * ROBOMAS_ANGLE_RESOLUTION * rb->gear_ratio;
  }else if(rb == &robomas_lower_r){
    rb->ang_pid.sv = ARM_R_ROBOMAS_DIRECTION * to_polar(coordinate_lower).r /(R_ROBOMAS_DIAMETER * M_PI) * ROBOMAS_ANGLE_RESOLUTION * rb->gear_ratio;
  }else if(rb == &robomas_upper_deg){
    rb->ang_pid.sv = ARM_DEG_ROBOMAS_DIRECTION * to_polar(coordinate_upper).theta / (2 * M_PI) * POLAR_RATIO * ROBOMAS_ANGLE_RESOLUTION * rb->gear_ratio;
  }else if(rb == &robomas_upper_r){
    rb->ang_pid.sv = ARM_R_ROBOMAS_DIRECTION * to_polar(coordinate_upper).r /(R_ROBOMAS_DIAMETER * M_PI) * ROBOMAS_ANGLE_RESOLUTION * rb->gear_ratio;
  }else{return NULL;}
  return rb;
}

/**
* @brief フィードバック途絶によりROBOMAS_ERRORへ遷移した軸を特定するエラーコードを返す．
*/
can_error_t get_lost_control_error_code(robomas_t *rb){
  if(rb == &robomas_lower_r)        return CAN_ERROR_LOWER_R_LOST_CONTROL;
  else if(rb == &robomas_lower_deg) return CAN_ERROR_LOWER_DEG_LOST_CONTROL;
  else if(rb == &robomas_upper_r)   return CAN_ERROR_UPPER_R_LOST_CONTROL;
  else                              return CAN_ERROR_UPPER_DEG_LOST_CONTROL;
}

void robomas_update(robomas_t *rb){
  robomas_state_t prev_state = rb->state;

  if(HAL_GetTick() - rb->feedback.last_update > ROBOMAS_FEEDBACK_TIMEOUT_MS){
    rb->state = ROBOMAS_ERROR;
    if(prev_state != ROBOMAS_ERROR){
      stm_can_send(&COMMAND_HCAN, &(can_command_data_t){
        .id = CAN_ID_ERROR_CODE,
        .data.error_code = get_lost_control_error_code(rb),
      });
    }
  }else if(rb->state == ROBOMAS_ERROR){
    rb->state = ROBOMAS_INITIAL;
  }

  switch(rb->state){
    case ROBOMAS_HOMING:
      set_robomas_homing_rpm(rb);
      break;

    case ROBOMAS_IDLE:
      rb->rpm_pid.sv = 0;
      break;

    case ROBOMAS_READY:
      set_robomas_deg_from_coordinate(rb);
      rb->ang_pid.pv = rb->feedback.total_angle- rb->total_angle_home;
      rb->rpm_pid.sv = calc_pid(&rb->ang_pid);
      break;

    case ROBOMAS_ERROR:
    case ROBOMAS_INITIAL:
    default:
      rb->tx_torque = 0;
      return;

  }
  rb->rpm_pid.pv = rb->feedback.rpm;
  rb->tx_torque = calc_pid(&rb->rpm_pid);
}

/**
* @brief Status_LEDの点滅を管理する．
*        lower/upperそれぞれがREADY(r軸・deg軸とも)でなければ「未ホーミング」とみなす．
*        lowerのみ未ホーミング:1回点滅 upperのみ:2回点滅 両方:3回点滅 両方READY:常時点灯
*        周期的(mainループ毎)に呼び出すこと．
*/
void status_led_update(){
  bool lower_ready = (robomas_lower_r.state == ROBOMAS_READY && robomas_lower_deg.state == ROBOMAS_READY);
  bool upper_ready = (robomas_upper_r.state == ROBOMAS_READY && robomas_upper_deg.state == ROBOMAS_READY);

  if(lower_ready && upper_ready){
    HAL_GPIO_WritePin(Status_LED_GPIO_Port, Status_LED_Pin, GPIO_PIN_SET);
    status_led_blink_index = 0;
    status_led_on = false;
    status_led_phase_start = HAL_GetTick();
    return;
  }

  uint8_t blink_count = (lower_ready ? 0 : 1) + (upper_ready ? 0 : 2);

  uint32_t now = HAL_GetTick();
  uint32_t elapsed = now - status_led_phase_start;

  if(status_led_on){
    if(elapsed >= STATUS_LED_BLINK_ON_MS){
      HAL_GPIO_WritePin(Status_LED_GPIO_Port, Status_LED_Pin, GPIO_PIN_RESET);
      status_led_on = false;
      status_led_phase_start = now;
      status_led_blink_index++;
    }
  }else{
    bool cycle_done = (status_led_blink_index >= blink_count);
    uint32_t off_duration = cycle_done ? STATUS_LED_BLINK_PAUSE_MS : STATUS_LED_BLINK_OFF_MS;

    if(elapsed >= off_duration){
      if(cycle_done)status_led_blink_index = 0; //一巡したので最初から

      HAL_GPIO_WritePin(Status_LED_GPIO_Port, Status_LED_Pin, GPIO_PIN_SET);
      status_led_on = true;
      status_led_phase_start = now;
    }
  }
}

/*in main roop end*/



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  uint32_t last_heartbeat = HAL_GetTick();
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
  // ここで全ROBOMAS_NUM分のフィードバックを待つと，一部のロボマスターしか
  // 接続していない状態(例: 動作確認で下アームの2台のみ接続)ではメインループに
  // 一切到達できず，接続済みのロボマスターについても
  // 制御・Heartbeat送信・Status_LED表示等が何も行われない(＝外から見て
  // 「ロボマスがつながっているのにフィードバックを受け取れない」ように見える)．
  // 未接続の軸はrobomas_update()のROBOMAS_FEEDBACK_TIMEOUT_MSタイムアウトにより
  // 自動的にROBOMAS_ERRORへ遷移しCAN_ID_ERROR_CODEで通知されるので，
  // ここで全台の初期化を待つ必要はない．

  for(int i = 0; i < ROBOMAS_NUM; i++){
    pid_reset(&robomas[i].rpm_pid);
    pid_reset(&robomas[i].ang_pid);
  }

  while (1)
  {
    upper_homing();
    lower_homing();

    for(int i = 0; i < ROBOMAS_NUM; i++){
      robomas_update(&robomas[i]);
    }

    robomas_send_torque(robomas);
    status_led_update();
    
    if(HAL_GetTick() - last_heartbeat > HEARTBEAT_MS){
      last_heartbeat = HAL_GetTick();
      stm_can_send(&COMMAND_HCAN, &(can_command_data_t){.id = CAN_ID_ROBOMAS_CONTROLLER_HEARTBEAT});
    }
    HAL_Delay(LOOP_MS);
    /* USER CODE END WHILE */

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
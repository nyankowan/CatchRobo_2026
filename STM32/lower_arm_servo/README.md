# STM32/lower_arm_servo
```text
                 CAN
ESP32 ──────── PA11/PA12
                   │
              STM32F303K8
                   │
       ┌───────────┴───────────┐
       │                       │
     TIM3                    TIM1
  CH1 CH2 CH3 CH4             CH1
  │    │   │   │               │
Lhand  M   R   EXP            shaft
```
## settings
### Pin

| 用途      | STM32ピン | CubeMX設定 | Nucleo端子 |
| ------- | ------- | -------- | -------- |
| Left    | PB4     | TIM3_CH1 | D12      |
| Middle  | PB5     | TIM3_CH2 | D11      |
| Right   | PB0     | TIM3_CH3 | D3       |
| Expand  | PB1     | TIM3_CH4 | D6       |
| Shaft   | PA8     | TIM1_CH1 | D9       |
| CAN RX  | PA11    | CAN_RX   | D10      |
| CAN TX  | PA12    | CAN_TX   | D2       |
| STATUS_LED|PB3    |GPIO_Output| D13     |


### SYS
```text
System Core
└── SYS
Debug = Serial Wire
```
### Clock
```text
RCC
  HSI = ON

Clock Configuration

HSI = 8 MHz
   ↓
SYSCLK = 8 MHz

AHB  /1 → 8 MHz
APB1 /1 → 8 MHz
APB2 /1 → 8 MHz
```

### TIM
#### TIM1
```text
Prescaler      = 7
Counter Period = 19999
Pulse          = 1500
```
#### TIM3
```text
Prescaler      = 7
Counter Period = 19999
Counter Mode   = Up

CH1 Pulse = 1500
CH2 Pulse = 1500
CH3 Pulse = 1500
CH4 Pulse = 1500
```
### CAN
```text
Prescaler                        1
Time Quanta in Bit Segment 1     6
Time Quanta in Bit Segment 2     1
ReSynchronization Jump Width     1
```
```text
CAN
└── NVIC Settings
CAN RX0 interrupt -> Enable
```

## Code
```C
/* USER CODE BEGIN Private defines */
#define Left_htim htim3
#define Middle_htim htim3
#define Right_htim htim3
#define Expand_htim htim3
#define Shaft_htim htim1

#define Left_TIM_CHANNEL TIM_CHANNEL_1
#define Middle_TIM_CHANNEL TIM_CHANNEL_2
#define Right_TIM_CHANNEL TIM_CHANNEL_3
#define Expand_TIM_CHANNEL TIM_CHANNEL_4
#define Shaft_TIM_CHANNEL TIM_CHANNEL_1
```

```C
/* USER CODE BEGIN PD */
#define SERVO_0   500
#define SERVO_270 2500
```

```C
/* USER CODE BEGIN Includes */
#include "can_protocol.h"
#include "coordinate.h"
#include "stm_can.h"
#include <math.h>
```

Heartbeat送信の実装は非ブロッキング(`HAL_Delay(HEARTBEAT_MS)`のような待機はしない)．詳細は後述の[Heartbeat](#heartbeat)章を参照．

### PWM Start
```C
/* USER CODE BEGIN 2 */
HAL_TIM_PWM_Start(&htim3, Left_TIM_CHANNEL);
HAL_TIM_PWM_Start(&htim3, Middle_TIM_CHANNEL);
HAL_TIM_PWM_Start(&htim3, Right_TIM_CHANNEL);
HAL_TIM_PWM_Start(&htim3, Expand_TIM_CHANNEL);

HAL_TIM_PWM_Start(&htim1, Shaft_TIM_CHANNEL);
```
### PWM Set
以下はServo1に1.5msのPWMを設定する方法．
```C
__HAL_TIM_SET_COMPARE(
    &htim3,
    TIM_CHANNEL_1,
    1500
);
```

### CAN
```C
/* USER CODE BEGIN PV */
CAN_RxHeaderTypeDef rx_header;
can_data_t rx_data = {0};
```

`CAN_ID_LOWER_ARM_COMMAND`(Left/Middle/Right/Expand/shaft_rotate)と`CAN_ID_LOWER_HOMING`を受信し，ハンドとShaftサーボのPWMを更新する．

```C
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
    // shaft_rotate=1のとき，アーム軸の回転によらずハンドの向きを90度回転させる
    // (基本は0~180度動くシャフトを，90~270度で動くようにする)
    double shaft_theta = to_polar(direct).theta;
    if(rx_data.lower_arm.shaft_rotate){shaft_theta += M_PI_2;}
    __HAL_TIM_SET_COMPARE(&Shaft_htim,Shaft_TIM_CHANNEL,shaft_theta * (SERVO_270 - SERVO_0) / (3 * M_PI_2) + SERVO_0);
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
```
```C
/* USER CODE BEGIN 2 */
HAL_CAN_Start(&hcan);
if (HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
  Error_Handler();
}
```

### Heartbeat

mainループ内で`HEARTBEAT_MS`(300ms，`common/can_protocol`で定義)周期ごとに`CAN_ID_LOWER_ARM_HEARTBEAT`を送信する．ESP32側はこれの受信有無でLower Arm Controllerとの通信生存を判定する(詳細は[common/can_protocol/README.md](../../common/can_protocol/README.md)のHeartbeat章参照)．

```C
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
```

```C
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
```

### LED

Status_LEDは，Left/Middle/Right/Expandそれぞれの現在の状態(true=ON)を点滅回数で表示する．CAN受信ハンドラでON/OFFを保持しておき，`status_led_update()`を周期的(mainループ毎)に呼ぶことで非ブロッキングに表示する．

表示順は Left → Middle → Right → Expand で，各chはONなら2回，OFFなら1回の短い点滅で表す(点滅回数を数えればON/OFFが分かる)．周期の始まりは長い点灯(マーカー)で示す．

```text
[長い点灯(マーカー)] [消灯] [Left:1or2回点滅] [消灯] [Middle:1or2回点滅] [消灯] [Right:...] [消灯] [Expand:...] [消灯(長め)] → 最初に戻る
```

| 定数 | 値 | 説明 |
| :--- | ---: | :--- |
| `STATUS_LED_MARKER_MS` | 1000 ms | 周期の始まりを示す長い点灯 |
| `STATUS_LED_BLINK_MS` | 100 ms | 1回分の点滅の点灯時間 |
| `STATUS_LED_INTRA_GAP_MS` | 100 ms | ONの2回点滅の間の消灯時間 |
| `STATUS_LED_CHANNEL_GAP_MS` | 700 ms | ch同士の間，およびマーカー直後の消灯時間 |
| `STATUS_LED_END_PAUSE_MS` | 1200 ms | 4ch分表示し終えてから次のマーカーまでの消灯時間 |

実装は`status_led_state_t`によるステートマシン(`STATUS_LED_STATE_MARKER` → `..._CHANNEL_GAP` → `..._BLINK_ON`/`..._BLINK_GAP`(chごとに1〜2回) → `..._END_PAUSE` → 最初に戻る)．ONかOFFかで点滅回数が変わるため，固定長のフェーズ表ではなくこの状態機械で管理している．

`CAN_ID_LOWER_HOMING`受信時は4chとも状態をOFFにリセットする(実際にサーボもSERVO_0へ戻す)．Shaftサーボも同時にSERVO_0へ戻す(Status_LEDの点滅表示には含まれない)．

```C
/* USER CODE BEGIN 2 */
HAL_GPIO_WritePin(
  STATUS_LED_GPIO_Port,
  STATUS_LED_Pin,
  GPIO_PIN_SET
);
```
```C
// 点灯
HAL_GPIO_WritePin(
    STATUS_LED_GPIO_Port,
    STATUS_LED_Pin,
    GPIO_PIN_SET
);

// 消灯
HAL_GPIO_WritePin(
    STATUS_LED_GPIO_Port,
    STATUS_LED_Pin,
    GPIO_PIN_RESET
);

// 反転
HAL_GPIO_TogglePin(
    STATUS_LED_GPIO_Port,
    STATUS_LED_Pin
);
```

### Error_Handler
```C
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
```

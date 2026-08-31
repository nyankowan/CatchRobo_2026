# STM32/assemble_servo

整理機構(Assemble)のサーボ角度を`CAN_ID_ASSEMBLE_COMMAND`(`common/can_protocol/README.md`参照)経由で制御する．

## settings
### Pin

| 用途      | STM32ピン | CubeMX設定 | Nucleo端子 |
| ------- | ------- | -------- | -------- |
| Assemble | PB4    | TIM3_CH1 | D12      |
| CAN RX  | PA11    | CAN_RX   | D10      |
| CAN TX  | PA12    | CAN_TX   | D2       |
| STATUS_LED | PB3  | GPIO_Output | D13   |

### TIM

TIM3(PWM，20ms周期)．他プロジェクト(lower_arm_servo等)と同じ`Prescaler=7`, `Counter Period=19999`．

### CAN

Prescaler=1, BS1=6TQ, BS2=1TQ, SJW=1TQ (他プロジェクトと共通)．

## Code

```C
/* USER CODE BEGIN PD */
#define SERVO_0   500
#define SERVO_270 2500
#define SERVO_DEG_RANGE 270 //サーボの角度(0-270deg)とパルス幅(SERVO_0-SERVO_270)の換算に使う定数

#define STATUS_LED_BLINK_MS 200 //異常時のLED点滅間隔
```

```C
/* USER CODE BEGIN Private defines */
#define ASSEMBLE_htim htim3
#define ASSEMBLE_TIM_CHANNEL TIM_CHANNEL_1
```

### CAN

`CAN_ID_ASSEMBLE_COMMAND`(1byte，角度指令0〜90°)を受信すると，機構の可動域(`ASSEMBLE_DEG_RANGE`=90，`common/can_protocol`で定義)にクランプした上でサーボPWMに変換して出力する．

```C
/* USER CODE BEGIN 0 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
  CAN_RxHeaderTypeDef rx_header;
  can_data_t rx_data = {0};
  if(hcan->Instance != CAN)return;
  if(HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_header,rx_data.raw) != HAL_OK)return;

  switch (rx_header.StdId) {
  case CAN_ID_ASSEMBLE_COMMAND: {
    assemble_deg_t deg = rx_data.assemble_deg;
    if(deg > ASSEMBLE_DEG_RANGE) deg = ASSEMBLE_DEG_RANGE; //機構の可動域(0-90度)にクランプ
    __HAL_TIM_SET_COMPARE(&ASSEMBLE_htim,ASSEMBLE_TIM_CHANNEL,
      (uint16_t)((double)deg / SERVO_DEG_RANGE * (SERVO_270 - SERVO_0) + SERVO_0));
    break;
  }
  default:
    break;
  }
}
```

Assembleには`common/can_protocol`が定義するHeartbeat CAN IDが無いため，ESP32側の`micon_connection`による生存監視の対象外(詳細は[common/can_protocol/README.md](../../common/can_protocol/README.md)参照)．

### Status_LED

CANのバスオフ／エラーパッシブ／エラーワーニングいずれかを検知したら異常状態としてラッチし，`STATUS_LED_BLINK_MS`(200ms)間隔で点滅させる．正常時は常時点灯．

```C
/* USER CODE BEGIN 3 */
//CANバスエラー(バスオフ/エラーパッシブ/エラーワーニング)を検知したらラッチする
if(__HAL_CAN_GET_FLAG(&hcan, CAN_FLAG_BOF) ||
   __HAL_CAN_GET_FLAG(&hcan, CAN_FLAG_EPV) ||
   __HAL_CAN_GET_FLAG(&hcan, CAN_FLAG_EWG)){
  can_bus_error = 1;
}

if(can_bus_error){
  //異常時: 点滅
  uint32_t now = HAL_GetTick();
  if(now - status_led_last_toggle >= STATUS_LED_BLINK_MS){
    status_led_last_toggle = now;
    HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
  }
}else{
  //正常動作中: 点灯
  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
}
```

### Error_Handler

初期化失敗などでHALの割り込みが止まり`HAL_GetTick()`が進まなくなった場合に備え，`Error_Handler()`内ではタイマー割り込みに頼らないソフトウェアループでLEDを点滅させる．

```C
/* USER CODE BEGIN Error_Handler_Debug */
__disable_irq();
while (1)
{
  HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
  for(volatile uint32_t i = 0; i < 400000; i++){}
}
```

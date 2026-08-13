# STM32/servo
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
### PWM Start
```C
/* USER CODE BEGIN 2 */
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
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
/* USER CODE BEGIN Includes */
#include "../../../../common/can_protocol/inc/can_protocol.h"
#include "../../../../common/coordinate/inc/coordinate.h"
```
```C
/* USER CODE BEGIN PV */
can_data_t rx_data = {
  .arm.hand = 0,
  /*
  *union{
  *  uint8_t hand;
  *  struct{
  *      unsigned int left    : 1;
  *      unsigned int middle  : 1;
  *      unsigned int right   : 1;
  *      unsigned int expand  : 1;
  *      unsigned int         : 4;
  *  };
  *};
  */
};
```
```C
/* USER CODE BEGIN 2 */
HAL_CAN_Start(&hcan);
```

```C
/* USER CODE BEGIN CAN_Init 2 */
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
if (HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
  Error_Handler();
}
```

```C
/* USER CODE BEGIN PFP */
HAL_StatusTypeDef can_send(can_command_data_t *com);
```
```C
/* USER CODE BEGIN 0 */
HAL_StatusTypeDef can_send(can_command_data_t *com){
  CAN_TxHeaderTypeDef tx_header;
  uint32_t tx_mailbox;

  tx_header.StdId = com->id;
  tx_header.ExtId = 0;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = CAN_DLC; //can_protocol.h
  tx_header.TransmitGlobalTime = DISABLE;

  return HAL_CAN_AddTxMessage(&hcan,&tx_header,com->data,&tx_mailbox);
}
```

### LED
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

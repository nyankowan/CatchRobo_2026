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

```C
/* USER CODE END WHILE */
stm_can_send(&hcan, &(can_command_data_t){.id = CAN_ID_LOWER_ARM_HEARTBEAT});
HAL_Delay(HEARTBEAT_MS);
```
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
/* USER CODE BEGIN PV */
CAN_RxHeaderTypeDef rx_header;
can_data_t rx_data = {0};
```
```C
/* USER CODE BEGIN 0 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
  if(hcan->Instance != CAN)return;
  if(HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_header,rx_data.raw) != HAL_OK)return;


  switch (rx_header.StdId) {
  case CAN_ID_LOWER_ARM_COMMAND:

    break;

  case CAN_ID_LOWER_HOMING:

    break;
  

  default:
    break;
  }
}
```
```C
/* USER CODE BEGIN 2 */
HAL_CAN_Start(&hcan);
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
if (HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
  Error_Handler();
}
/* can_rx setting */
```

### LED
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

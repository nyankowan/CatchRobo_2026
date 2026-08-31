# STM32/upper_arm_servo
| 用途      | STM32ピン | CubeMX設定 | Nucleo端子 |
| ------- | ------- | -------- | -------- |
| Shaft   | PA8     | TIM1_CH1 | D9       |
| Z       | PA9     | TIM1_CH2 | D1       |
| CAN RX  | PA11    | CAN_RX   | D10      |
| CAN TX  | PA12    | CAN_TX   | D2       |
| STATUS_LED | PB3  | GPIO_Output | D13   |

## 実装状況

STM32CubeMXが生成したピン・クロック・TIM1(PWM)設定のみで，`CAN_ID_UPPER_ARM_COMMAND`(`common/can_protocol/README.md`参照)を受信してShaft/ZサーボのPWMを更新する処理はまだ実装されていない．PWMの`HAL_TIM_PWM_Start()`も，CANの`HAL_CAN_Start()`もまだ呼ばれておらず，`CMakeLists.txt`も`STM32/common/can`・`common/can_protocol`・`common/coordinate`をまだリンクしていない．

実装する際のピン・タイマー設定・CAN送受信・Status_LEDの実装パターンは，同じ回路基板(STM32F303K8)を使う[STM32/lower_arm_servo/README.md](../lower_arm_servo/README.md)を参照．

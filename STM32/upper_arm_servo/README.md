# STM32/upper_arm_servo
| 用途      | STM32ピン | CubeMX設定 | Nucleo端子 |
| ------- | ------- | -------- | -------- |
| Shaft   | PA8     | TIM1_CH1 | D9       |
| Z       | PB1     | TIM3_CH4 | D6       |
| CAN RX  | PA11    | CAN_RX   | D10      |
| CAN TX  | PA12    | CAN_TX   | D2       |
| STATUS_LED | PB3  | GPIO_Output | D13   |

## 実装状況

`CAN_ID_UPPER_ARM_COMMAND`(`common/can_protocol/README.md`参照)を受信し，x/yから求めた偏角をShaftサーボ，zをZサーボのPWMへ反映する．`CAN_ID_UPPER_HOMING`受信時はShaftのみ`SERVO_0`へ戻す(Zは現在位置を維持する)．

Shaft/Zとも，計算したパルス幅を`SERVO_0`~`SERVO_270`の範囲にクランプしてから`__HAL_TIM_SET_COMPARE()`する．クランプが発生した(=可動域外のコマンドを受信した)場合はSTATUS_LEDを高速点滅(`STATUS_LED_ERROR_BLINK_MS`)させてエラーを知らせる．正常時はSTATUS_LEDを低速点滅(`STATUS_LED_BLINK_MS`)させ，動作中であることを示す(いずれも`status_led_update()`をmainループ毎に呼ぶことで非ブロッキングに実現)．

なお，ESP32側(`ESP32/controller/main/robot/arm_command.c`)でも送信前にx/y/zを可動域(`common/arm/inc/arm.h`の`UPPER_ARM_R_RANGE`/`UPPER_ARM_DEG_RANGE`/`UPPER_ARM_Z_RANGE`)にクランプしているため，STM32側のクランプは想定外の入力に対する保険(二重の安全策)という位置づけ．

実装のピン・タイマー設定・CAN送受信・Status_LEDの基本パターンは，同じ回路基板(STM32F303K8)を使う[STM32/lower_arm_servo/README.md](../lower_arm_servo/README.md)を参照．

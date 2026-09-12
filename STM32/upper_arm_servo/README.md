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

上のアームはハンドの取り付け側が下のアームと逆(180度回転してついている)ため，偏角の可動域は`UPPER_ARM_DEG_MIN`~`MIN+RANGE`(180~360度)になる．Shaftサーボへ反映する際は，`shaft_deg_from_home()`でホーミング原点(`UPPER_ARM_DEG_HOME_DEG`，`common/arm/inc/arm.h`参照)からの相対偏角(0~180度)を計算し，下のアームと同じ0~180度基準として使う．

`UPPER_ARM_DEG_HOME_DEG`は`ROBOT_TEAM`(赤/青チーム。`common/arm/inc/arm.h`参照)に応じて可動域の下限(180度)または上限(360度=`to_polar()`の値域[0,2π)では0度としてラップされる)になる．`shaft_deg_from_home()`は偏角が180度未満ならラップ分の360度を足してから原点との差の絶対値を取ることで，どちらの原点でも0~180度の相対偏角として扱えるようにしている．

`shaft_deg_from_home()`で求めた相対偏角には，さらに`shaft_rotate`(1のときハンドの向きを90度回転させる)と`shaft_fine`(度単位の微調整オフセット)を加算してからShaftサーボのパルス幅に変換する．下のアーム([STM32/lower_arm_servo/README.md](../lower_arm_servo/README.md)参照)と同じ考え方で，270度サーボのうち通常使うのは可動範囲分の180度だけなので，`shaft_rotate=1`による90度回転が残り90度分の余裕を超える場合は下記のクランプにより「それ以上は回転しない」ことを許容する．余裕が生まれる回転方向は整理機構の取り付け側に応じてチームで逆になるため，`ROBOT_TEAM`に応じて青チームは`+90`度，赤チームは`-90`度を加算する．`shaft_fine`は`shaft_rotate`とは独立に加算し，有効範囲は`-15`～`15`度とする(`ESP32/controller/main/robot/arm_command.c`の`UPPER_ARM_SHAFT_FINE_MAX_DEG`)．

Shaft/Zとも，計算したパルス幅を`SERVO_0`~`SERVO_270`の範囲にクランプしてから`__HAL_TIM_SET_COMPARE()`する．クランプが発生した(=可動域外のコマンドを受信した，または上記の`shaft_rotate`/`shaft_fine`加算で可動域を超えた)場合はSTATUS_LEDを高速点滅(`STATUS_LED_ERROR_BLINK_MS`)させてエラーを知らせる．正常時はSTATUS_LEDを点灯させ，動作中であることを示す(いずれも`status_led_update()`をmainループ毎に呼ぶことで非ブロッキングに実現)．

なお，ESP32側(`ESP32/controller/main/robot/arm_command.c`)でも送信前にx/y/zを可動域(`common/arm/inc/arm.h`の`UPPER_ARM_R_RANGE`/`UPPER_ARM_DEG_RANGE`/`UPPER_ARM_Z_RANGE`)にクランプしているため，STM32側のクランプは想定外の入力に対する保険(二重の安全策)という位置づけ．

実装のピン・タイマー設定・CAN送受信・Status_LEDの基本パターンは，同じ回路基板(STM32F303K8)を使う[STM32/lower_arm_servo/README.md](../lower_arm_servo/README.md)を参照．

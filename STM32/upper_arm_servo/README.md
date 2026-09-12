# STM32/upper_arm_servo
| 用途      | STM32ピン | CubeMX設定 | Nucleo端子 |
| ------- | ------- | -------- | -------- |
| Shaft   | PA8     | TIM1_CH1 | D9       |
| Z       | PB1     | TIM3_CH4 | D6       |
| CAN RX  | PA11    | CAN_RX   | D10      |
| CAN TX  | PA12    | CAN_TX   | D2       |
| STATUS_LED | PB3  | GPIO_Output | D13   |

## 実装状況

`CAN_ID_UPPER_ARM_COMMAND`(`common/can_protocol/README.md`参照)を受信し，x/yから求めた偏角(`shaft_rotate`による180度回転を含む)をShaftサーボ，zをZサーボのPWMへ反映する．`CAN_ID_UPPER_HOMING`受信時はShaftのみ`SERVO_0`へ戻す(Zは現在位置を維持する)．

上のアームはハンドの取り付け側が下のアームと逆(180度回転してついている)ため，偏角の可動域は`UPPER_ARM_DEG_MIN`~`MIN+RANGE`(180~360度)になる．Shaftサーボへ反映する際は，`shaft_deg_from_home()`でホーミング原点(`UPPER_ARM_DEG_HOME_DEG`，`common/arm/inc/arm.h`参照)からの相対偏角(0~180度)を計算し，下のアームと同じ0~180度基準として使う．

`UPPER_ARM_DEG_HOME_DEG`は`ROBOT_TEAM`(赤/青チーム。`common/arm/inc/arm.h`参照)に応じて可動域の下限(180度)または上限(360度=`to_polar()`の値域[0,2π)では0度としてラップされる)になる．`shaft_deg_from_home()`は偏角が180度未満ならラップ分の360度を足してから原点との差の絶対値を取ることで，どちらの原点でも0~180度の相対偏角として扱えるようにしている．

`rx_data.upper_arm.shaft_rotate`が1の場合，`shaft_deg_from_home()`で求めた相対偏角(ラジアン)に`M_PI`(180度)を加算してからパルス幅に変換することで，ハンドの向きを180度回転させる．270度サーボのうち普段使うのは可動範囲分の180度だけなので，残り90度の余裕を超える(=アームの相対偏角が90度を超えている)場合は反対側まで回転しきれず，下記のパルス幅クランプによって`SERVO_270`に留まる(=回転が適用されない)．team側で使用可能範囲を切り替えたり，そのたびにサーボを取り替えたりはせず，この制約をそのまま許容する．

Shaft/Zとも，計算したパルス幅を`SERVO_0`~`SERVO_270`の範囲にクランプしてから`__HAL_TIM_SET_COMPARE()`する．クランプが発生した場合はSTATUS_LEDを高速点滅(`STATUS_LED_ERROR_BLINK_MS`)させて知らせる．想定外の可動域外コマンドを受信した場合だけでなく，`shaft_rotate`によって180度回転しきれず(=回転が適用されず)クランプされた場合も同じ表示になるため，STATUS_LEDの高速点滅は「回転できなかった」ことの合図としても使える．正常時はSTATUS_LEDを点灯させ，動作中であることを示す(いずれも`status_led_update()`をmainループ毎に呼ぶことで非ブロッキングに実現)．

なお，ESP32側(`ESP32/controller/main/robot/arm_command.c`)でも送信前にx/y/zを可動域(`common/arm/inc/arm.h`の`UPPER_ARM_R_RANGE`/`UPPER_ARM_DEG_RANGE`/`UPPER_ARM_Z_RANGE`)にクランプしているため，STM32側のクランプは想定外の入力に対する保険(二重の安全策)という位置づけ．

実装のピン・タイマー設定・CAN送受信・Status_LEDの基本パターンは，同じ回路基板(STM32F303K8)を使う[STM32/lower_arm_servo/README.md](../lower_arm_servo/README.md)を参照．

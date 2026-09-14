# STM32/upper_arm_servo

上アームのShaft(ハンドの向き)とZ(昇降)サーボを`CAN_ID_UPPER_ARM_COMMAND`/`CAN_ID_UPPER_HOMING`(`common/can_protocol/README.md`参照)経由で制御する．

同じ回路基板(NUCLEO-F303K8，`Kicad/STM32F303K8_lower_arm_servo`)を使う[STM32/lower_arm_servo](../lower_arm_servo/README.md)と，ピン・タイマー設定・CAN送受信・Status_LEDの基本パターンは共通．CubeMXの詳細な設定値はそちらのREADMEを参照．

## settings
### Pin

| 用途      | STM32ピン | CubeMX設定 | Nucleo端子 |
| ------- | ------- | -------- | -------- |
| Shaft   | PA8     | TIM1_CH1 | D9       |
| Z       | PB1     | TIM3_CH4 | D6       |
| CAN RX  | PA11    | CAN_RX   | D10      |
| CAN TX  | PA12    | CAN_TX   | D2       |
| STATUS_LED | PB3  | GPIO_Output | D13   |

### TIM

TIM1(Shaft)/TIM3(Z)とも，PWM 20ms周期．`lower_arm_servo`と同じ`Prescaler=7`, `Counter Period=19999`．

### CAN

Prescaler=1, BS1=6TQ, BS2=1TQ, SJW=1TQ (他のF303K8プロジェクトと共通，1Mbps)．

## Code

```C
/* USER CODE BEGIN Private defines */
#define Shaft_htim htim1
#define Z_htim htim3

#define Shaft_TIM_CHANNEL TIM_CHANNEL_1
#define Z_TIM_CHANNEL TIM_CHANNEL_4
```

```C
/* USER CODE BEGIN PD */
#define SERVO_0   500
#define SERVO_270 2500

#define LOOP_MS 2

// 受信コマンドが可動域外でクランプされた場合は高速点滅に切り替えてエラーを知らせる
#define STATUS_LED_ERROR_BLINK_MS 100
```

## 実装状況

`CAN_ID_UPPER_ARM_COMMAND`を受信し，x/yから求めたアームの偏角をShaftサーボ，zをZサーボのPWMへ反映する．`CAN_ID_UPPER_HOMING`受信時はShaftのみホーミング原点(`UPPER_ARM_HOME_COORDINATE`)の向き(`shaft_home_pulse()`)へ戻す(Zは現在位置を維持する)．ホーミング開始時点からホーミング完了時と同じ向きにしておくことで，完了時にハンドの向きが変わらないようにしている(下アームの`shaft_home_pulse()`と同じ考え方，[STM32/lower_arm_servo/README.md](../lower_arm_servo/README.md)参照)．

Homing処理そのもの(リミットスイッチ検出・ロボマスの制御)は`robomas_controller`が担当し，このファームウェアは`HOMING_ACK`/`HOMING_DONE`を返さない(詳細は[common/can_protocol/README.md](../../common/can_protocol/README.md)の4章)．

### Shaftの角度計算

上のアームはハンドの取り付け側が下のアームと逆(180度回転してついている)ため，偏角の可動域は`UPPER_ARM_DEG_MIN`~`UPPER_ARM_DEG_MIN + UPPER_ARM_DEG_RANGE`(180~360度)になる(`common/arm/inc/arm.h`参照)．

Shaftは可動範囲270度のうち中央の180度(45~225度)だけを通常使用域とし，`shaft_deg_from_arm_deg()`が，アームの可動域の中央(=アームが真下向きの位置)がShaft使用域の中央(135度)に一致するよう，偏角からオフセットを引くだけの単純な線形対応で目標角度を求める．

```C
static double shaft_deg_from_arm_deg(direct_t direct){
  double deg = to_polar(direct).theta / (2 * M_PI) * 360.0;
  if(deg < UPPER_ARM_DEG_MIN) deg += 360.0;
  // アーム可動域の中央(真下向き)をShaft使用域の中央(135度)に合わせるオフセット
  return deg - (UPPER_ARM_DEG_MIN + UPPER_ARM_DEG_RANGE / 2.0) + 135.0;
}
```

`fabs()`等で折り返さない線形対応にすることで，アームの回転方向とShaftの回転方向が常に一致し，かつ計算式自体はチームによらず共通になる．チーム差は`UPPER_ARM_HOME_COORDINATE`(`common/arm/inc/arm.h`)が可動域のどちら側の座標になるかにのみ現れ，赤チームのホーミング原点(`UPPER_ARM_DEG_MIN`=180度)では`SERVO_45`，青チームのホーミング原点(`UPPER_ARM_DEG_MIN + UPPER_ARM_DEG_RANGE`=360度)では`SERVO_225`になる．

360度は`coordinate.h`の`to_polar()`の値域[0,2π)では0度としてラップされるため，偏角が0度付近で不連続にならないよう，`UPPER_ARM_DEG_MIN`未満の値には360度を足してから使う．

`shaft_deg_from_arm_deg()`で求めた角度には，さらに`shaft_rotate`(1のときハンドの向きを90度回転させる)と`shaft_fine`(度単位の微調整オフセット)を加算してからShaftサーボのパルス幅に変換する．下のアーム([STM32/lower_arm_servo/README.md](../lower_arm_servo/README.md)参照)と同じ考え方で，270度サーボのうち通常使うのは可動範囲分の180度だけなので，`shaft_rotate=1`による90度回転が残り90度分の余裕を超える場合は下記のクランプにより「それ以上は回転しない」ことを許容する．余裕が生まれる回転方向は整理機構の取り付け側に応じてチームで逆になるため，`ROBOT_TEAM`に応じて青チームは`+90`度，赤チームは`-90`度を加算する．`shaft_fine`は`shaft_rotate`とは独立に加算し，有効範囲は`-45`～`45`度とする(`ESP32/controller/main/robot/arm_command.c`の`UPPER_ARM_SHAFT_FINE_MAX_DEG`)．

### Zの角度計算

zは`UPPER_ARM_Z_SERVO_GEAR_DIAMETER`(`common/arm/inc/arm.h`)のギアで昇降量(mm)からサーボ角度に換算する．

```C
double z_pulse = rx_data.upper_arm.z * (SERVO_270 - SERVO_0) / (UPPER_ARM_Z_SERVO_GEAR_DIAMETER * 3 * M_PI_4) + SERVO_0;
```

### Status_LED

Shaft/Zとも，計算したパルス幅を`SERVO_0`~`SERVO_270`の範囲にクランプしてから`__HAL_TIM_SET_COMPARE()`する．クランプが発生した(=可動域外のコマンドを受信した，または上記の`shaft_rotate`/`shaft_fine`加算で可動域を超えた)場合はSTATUS_LEDを高速点滅(`STATUS_LED_ERROR_BLINK_MS`=100ms)させてエラーを知らせる．正常時はSTATUS_LEDを点灯させ，動作中であることを示す(いずれも`status_led_update()`をmainループ毎に呼ぶことで非ブロッキングに実現)．

なお，ESP32側(`ESP32/controller/main/robot/arm_command.c`)でも送信前にx/y/zを可動域(`common/arm/inc/arm.h`の`UPPER_ARM_R_RANGE`/`UPPER_ARM_DEG_RANGE`/`UPPER_ARM_Z_RANGE`)にクランプしているため，STM32側のクランプは想定外の入力に対する保険(二重の安全策)という位置づけ．

### Heartbeat

mainループ内で`HEARTBEAT_MS`(300ms，`common/can_protocol`で定義)周期ごとに`CAN_ID_UPPER_ARM_HEARTBEAT`を送信する．ESP32側はこれの受信有無でUpper Arm Controllerとの通信生存を判定する(詳細は[common/can_protocol/README.md](../../common/can_protocol/README.md)のHeartbeat章参照)．

```C
/* USER CODE BEGIN WHILE */
uint32_t last_heartbeat = HAL_GetTick();
while (1)
{
  status_led_update();

  if(HAL_GetTick() - last_heartbeat > HEARTBEAT_MS){
    last_heartbeat = HAL_GetTick();
    stm_can_send(&hcan, &(can_command_data_t){.id = CAN_ID_UPPER_ARM_HEARTBEAT});
  }
  /* USER CODE END WHILE */

  HAL_Delay(LOOP_MS);
  /* USER CODE BEGIN 3 */
}
```

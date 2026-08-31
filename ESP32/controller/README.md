# ESP32/controller

Main Controller．2台のプロコン(Nintendo Switch用ジョイコン等)をBluetoothで受け取り，CAN通信で各STM32(末端マイコン)に指令を送る．

## Directory structure

```text
ESP32/controller/
├── components/
│   ├── can/
│   │   └── CAN(TWAI)通信のラッパー
│   └── procon/
│       └── Bluetoothプロコン(Nintendo Switchコントローラー等)の接続・入力取得
│
└── main/
    ├── main.c              メインループ．プロコン入力の読み取りとロボット制御の呼び出し
    ├── gpio.h              各種GPIO番号(CAN, ステータスLED)の定義
    ├── led.c/h             ステータスLEDの初期化・点灯制御
    ├── micon_connection.c/h  各STM32のHeartbeat監視(通信生存確認)
    └── robot/
        └── arm_command.c/h   両アームへの座標指令・ホーミングシーケンスの実装
```

## components/can

CAN(TWAI)通信をサポートする．

```c
esp_err_t can_init_and_start(gpio_num_t tx, gpio_num_t rx);
esp_err_t can_tx(can_command_data_t *com);
void can_register_rx_callback(can_id_t id, can_rx_callback_t callback);
```

- `can_init_and_start()` で500kbpsのCAN通信を開始する．
- `can_tx()` で `can_command_data_t` をtxキューに追加して送信する．
- `can_register_rx_callback()` で，特定のCAN ID(`can_id_t`)を受信した際に呼ばれるコールバックを登録できる．コールバックはCAN rx taskから直接呼ばれるため，重い処理を書かないこと．
- `can_error_handling_task()` が，CANのバスオフを検知すると自動的にリカバーする．

CAN ID・データフォーマットの仕様そのものは [common/can_protocol](../../common/can_protocol/README.md) を参照．このプロジェクトでは仕様を実装するだけで，仕様自体はここでは定義しない．

## components/procon

Bluetooth経由でプロコン(ジョイコン等)を接続し，スティック・ボタンの入力を取得する．

## main

### micon_connection

`CAN_ID_ROBOMAS_CONTROLLER_HEARTBEAT` / `CAN_ID_UPPER_ARM_HEARTBEAT` / `CAN_ID_LOWER_ARM_HEARTBEAT` の受信状況を監視し，各STM32との通信が生きているかを管理する．

```c
void micon_connection_init();
void micon_connection_update();      // 周期的に呼び，Heartbeatタイムアウトを判定する
bool get_connection(micon_type_t m);  // 対象のマイコンと通信できているか
```

`MICON_TYPE_ROBOMAS_CONTROLLER` / `MICON_TYPE_UPPER_ARM` / `MICON_TYPE_LOWER_ARM` の3種類とも実装済み．`micon_connection_update()`は`HEARTBEAT_TIMEOUT_MS`(1000ms)を`pdMS_TO_TICKS()`でtick単位に変換してから経過tick数と比較する(逆にすると桁を間違えるので注意)．

### robot/arm_command

両アーム(Upper/Lower)への座標指令とホーミングシーケンスを実装する．

- `upper_arm_move()` : ジョイスティックの差分入力(dx, dy)を現在座標に加算する．可動域(`common/arm`で定義)の外に出る移動は無視する．z軸(dz)は可動域チェックなしにそのまま加算する．ホーミング中は無視する．
- `lower_arm_move()` : 座標(dx, dy)の扱いは`upper_arm_move()`と同様．加えてLeft/Middle/Right/Expandの各ハンドと，ハンドの向きを90度回転させる`shaft_rotate`を，トグル入力(ボタンのPRESSEDエッジ)で切り替える．ホーミング中は座標もハンド操作も無視する．
- `send_lower_arm()` / `send_upper_arm()` : 現在座標をCANで送信する．ホーミング中は送信しない．
- `lower_arm_homing()` / `upper_arm_homing()` : ホーミング要求(`HOMING`)をSTM32(robomas_controller)に送信する．
- `arms_update()` : 周期的に呼び，`HOMING_ACK`未受信時の要求再送(`HOMING_REQUEST_RETRY_MS`=100msごと)や，ホーミング全体のタイムアウト判定を行う．
- CAN受信コールバック(`*_homing_ack_notify`, `*_homing_done_notify`, `error_code_notify`) で，STM32側からのACK・完了通知・エラー通知を処理する．`error_code_notify`はタイムアウト系・拒否系のエラーを受け取った時点で，ESP32側の10秒待ちを待たずに即座に`homing_in_progress`を解除する．

ホーミングのシーケンス(ACK・DONE・再送を含む)の詳細は [STM32/robomas_controller/README.md](../../STM32/robomas_controller/README.md) を参照．

### led / gpio

`gpio.h`で定義された7つのLEDを`main_task`のループ毎に更新する．

| LED (`gpio.h`) | 表示内容 |
| :--- | :--- |
| `CONTROLLER_1_LED_GPIO` / `CONTROLLER_2_LED_GPIO` | 各プロコンが接続されているか |
| `ROBOMAS_CONTROLLER_STATUS_LED_GPIO` | `get_connection(MICON_TYPE_ROBOMAS_CONTROLLER)` |
| `LOWER_ARM_STATUS_LED_GPIO` | `get_connection(MICON_TYPE_LOWER_ARM)` |
| `UPPER_ARM_STATUS_LED_GPIO` | `get_connection(MICON_TYPE_UPPER_ARM)` |
| `CAN_STATUS_LED_GPIO` | `can_is_running()`(CANが`TWAI_STATE_RUNNING`か) |
| `STATUS_LED_GPIO` | `main_task`のループ毎にON/OFFを反転させる生存確認用(約3.8Hzで点滅) |

`can_is_running()`は`components/can`に追加したAPIで，`can_error_handling_task()`内で追跡している現在のTWAI状態を返す．
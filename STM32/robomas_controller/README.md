# STM32/robomas_controller

| 用途 | STM32ピン | CubeMX設定 |
| :--- | :--- | :--- |
| CAN1 (Main Controllerとの通信) | PA11/PA12 | COMMAND_CAN_RX/TX |
| CAN2 (Robomasterとの通信) | PB12/PB13 | ROBOMAS_CAN_RX/TX |
| Status_LED | PA5 | GPIO_Output |
| Upper Arm R軸 リミットスイッチ | PC2 | GPIO_Input (UPPER_ARM_R_LIMIT) |
| Upper Arm Deg軸 下限リミットスイッチ | PC3 | GPIO_Input (UPPER_ARM_DEG_UNDER_LIMIT) |
| Upper Arm Deg軸 上限リミットスイッチ | PC4 | GPIO_Input (UPPER_ARM_DEG_OVER_LIMIT，未使用) |
| Lower Arm R軸 リミットスイッチ | PC5 | GPIO_Input (LOWER_ARM_R_LIMIT) |
| Lower Arm Deg軸 下限リミットスイッチ | PC10 | GPIO_Input (LOWER_ARM_DEG_UNDER_LIMIT) |
| Lower Arm Deg軸 上限リミットスイッチ | PC11 | GPIO_Input (LOWER_ARM_DEG_OVER_LIMIT，未使用) |

各軸の`_UNDER_LIMIT`(ホーミング方向のリミットスイッチ)のみソフトウェアで読み取っている(下記Homingシーケンス参照)．`_OVER_LIMIT`はハードウェアの可動域超過検知用の入力として定義されているのみで，現状のファームウェアからは読み取っていない．

# 概要
両アームのアーム長とアーム偏角をロボマスターにより制御する．

すべてのロボマスターからフィードバックを正常に受け取ることができてから，    
Main Controllerからホーミング指令を受け取り，ホーミング処理が完了して初めて，Main Controllerから制御可能になる．    
ホーミング処理中はMain Controllerからの指令を無視する． 
ホーミング処理はアームごと(r軸・deg軸の2ロボマス1組)に行う．

CAN上のメッセージフォーマットやCAN IDの詳細は [common/can_protocol/README.md](../../common/can_protocol/README.md) を参照．
このREADMEでは，robomas_controller自身がその仕様をどう実装しているか(状態遷移・タイムアウト・再送)を説明する．

## ROBOMASTER_STATE
```C
typedef enum{
  ROBOMAS_INITIAL,  // 初回起動時状態
  ROBOMAS_HOMING,   // 速度制御によって，リミットスイッチ位置まで回転する
  ROBOMAS_IDLE,     // ホーミング終了による他のホーミングを待機
  ROBOMAS_READY,    // ユーザが制御可能な状態
  ROBOMAS_ERROR,    // ロボマスからのフィードバックが途絶えた異常状態
}robomas_state_t;
```
| Value             |  explanation |
| :---------------- | :----------- |
|`ROBOMAS_INITIAL`  | マイコン起動時の初期状態．Main Controllerからのホーミング指令により `ROBOMAS_HOMING` に遷移する．|
|`ROBOMAS_HOMING`   | ホーミング処理中の状態．リミットスイッチからの信号で `ROBOMAS_IDLE` に遷移する．規定時間内に遷移しなければタイムアウトし `ROBOMAS_INITIAL` に戻る．|
|`ROBOMAS_IDLE`     | ホーミング処理を終了し，もう片方(r軸/deg軸)のホーミング完了待ち．両ロボマスがこの状態になると同時に`ROBOMAS_READY`に遷移する．|
|`ROBOMAS_READY`    | Main Controllerからの座標指令をもとに制御を行える状態．ホーミング指令により `ROBOMAS_HOMING` に遷移する．|
|`ROBOMAS_ERROR`    | `ROBOMAS_FEEDBACK_TIMEOUT_MS`(100ms)以上ロボマスからのフィードバックを受信できていない状態．トルク出力は強制的に0にする．フィードバックが復帰すると自動的に`ROBOMAS_INITIAL`に戻る(制御を再開するには再度ホーミングが必要)．|

r軸・deg軸は基本的に独立して状態遷移するが，`ROBOMAS_READY`への遷移(ホーミング完了)だけは両方が`ROBOMAS_IDLE`になった瞬間に同時に行われる．

---

## ロボマスターのCAN ID対応

`robomas_receive()`は受信したCAN ID(0x201〜0x204)をそのまま配列indexに変換して`robomas[]`に格納している．

```c
uint8_t robomas_id = rx_header.StdId - 0x201; // 0x201→0, 0x202→1, 0x203→2, 0x204→3
```

このindexと実際のアーム・軸の対応は以下の通り．**ロボマスター(C620等)側のCAN ID設定は，この対応に合わせて配線・設定すること．**

| CAN ID | `robomas[]` index | マクロ名 | 対応する軸 |
| :---: | :---: | :--- | :--- |
| `0x201` (ID1) | `robomas[0]` | `robomas_lower_deg` | 下アーム 偏角(deg) |
| `0x202` (ID2) | `robomas[1]` | `robomas_lower_r`   | 下アーム 伸縮(r) |
| `0x203` (ID3) | `robomas[2]` | `robomas_upper_deg` | 上アーム 偏角(deg) |
| `0x204` (ID4) | `robomas[3]` | `robomas_upper_r`   | 上アーム 伸縮(r) |

配線側のCAN ID設定とこの対応がずれると，例えば意図したR軸の代わりにDEG軸のエラーが報告される，といった混乱の原因になるので注意．

---

## Homingシーケンス

`common/can_protocol/README.md` に記載の

```text
Main Controller                     Robomas Controller
      |                                   |
      | -------- HOMING ----------------> |
      | <------- HOMING_ACK ------------- |
      |                                   |
      |          Homing処理                |
      |                                   |
      | <------- HOMING_DONE ------------ |
      | -------- HOMING_DONE_ACK -------->|
      |                                   |
```

を，CANフレーム欠落があっても最終的に完了できるよう，双方向に再送を実装している．

### HOMING受信 → HOMING_ACK送信
- `CAN_ID_UPPER_HOMING` / `CAN_ID_LOWER_HOMING` を受信すると，受信したsequence numberを保持し，即座に同じsequence numberで `HOMING_ACK` を返す．
- Main Controller側は `HOMING_ACK` が `HOMING_REQUEST_RETRY_MS`(100ms) 以内に届かなければ同じsequence numberでHOMING要求を再送する(実装は`ESP32/controller/main/robot/arm_command.c`)．
- 該当軸が既に`ROBOMAS_IDLE`(もう片方のホーミング待ち)の場合，sequence numberが今処理中のものと一致すればACK消失による再送とみなしてACKを返し直す．一致しない場合は新規の要求とみなし無視する．
- 該当軸が`ROBOMAS_ERROR`の場合，ホーミングは開始せず，`CAN_ID_ERROR_CODE`(`CAN_ERROR_*_HOMING_REJECTED`)を返す．

### Homing処理
- `HOMING_UPPER_DEG_RPM` / `HOMING_LOWER_DEG_RPM` / `HOMING_UPPER_R_RPM` / `HOMING_LOWER_R_RPM` (`common/arm/inc/arm.h`)で指定される回転数でリミットスイッチ方向へ回転する．
- リミットスイッチ検出で`ROBOMAS_IDLE`に遷移し，その時点のロボマス角度を極座標原点(r軸は`*_ARM_R_MIN`を考慮したオフセット付き)として記録する．
- `HOMING_UPPER_ARM_TIMEOUT_MS` / `HOMING_LOWER_ARM_TIMEOUT_MS`(いずれも10000ms)以内にリミットスイッチを検出できなければ，両軸を`ROBOMAS_INITIAL`に戻し，`CAN_ID_ERROR_CODE`(`CAN_ERROR_*_HOMING_TIMEOUT`)を送信する．リミットスイッチ故障や配線不良で無限に回転し続けることを防ぐための仕組み．
- このタイムアウト通知は，一度のホーミング試行につき`upper/lower_homing_timeout_notified`フラグにより1回だけ送信される．`ROBOMAS_ERROR`(フィードバック途絶)状態の軸は上書きせずそのまま`robomas_update()`に管理を委ねるため，タイムアウト処理と`ROBOMAS_ERROR`検知が互いの状態を打ち消し合って`CAN_ID_ERROR_CODE`を送り続ける(スパムする)ことがない．フラグは新しいホーミング要求を受理した時点でリセットされる．

### HOMING_DONE送信 → HOMING_DONE_ACK待ち
- 両軸が`ROBOMAS_IDLE`になった瞬間，`ROBOMAS_READY`に遷移すると同時に，保持していたsequence numberで`HOMING_DONE`を送信する．
- `HOMING_DONE_ACK_RETRY_MS`(200ms)ごとに`HOMING_DONE_ACK`の到着を確認し，届いていなければ`HOMING_DONE`を再送する．最大`HOMING_DONE_ACK_MAX_RETRY`(5回)まで再送し，それでも届かなければ諦める(この場合もアーム自体は`ROBOMAS_READY`のまま動作可能)．

---

## 異常検知とエラー通知 (`CAN_ID_ERROR_CODE`)

以下のタイミングで`CAN_ID_ERROR_CODE`(詳細は`common/can_protocol/README.md`)を送信する．

| タイミング | 送信するError Code |
| :--- | :--- |
| ロボマスからのフィードバックが途絶え`ROBOMAS_ERROR`に遷移した瞬間 | `CAN_ERROR_{LOWER,UPPER}_{R,DEG}_LOST_CONTROL` |
| ホーミングが`HOMING_*_ARM_TIMEOUT_MS`以内に完了しなかった | `CAN_ERROR_{LOWER,UPPER}_HOMING_TIMEOUT` |
| `ROBOMAS_ERROR`中にホーミング要求を受けて無視した | `CAN_ERROR_{LOWER,UPPER}_HOMING_REJECTED` |

いずれも状態遷移の瞬間に1回だけ送信し，同じ状態が続く間は再送しない(CANバスを埋めないため)．

---

## 主要タイミング定数一覧

| 定数 | 値 | 定義箇所 | 説明 |
| :--- | ---: | :--- | :--- |
| `ROBOMAS_FEEDBACK_TIMEOUT_MS` | 100 ms | `robomas_controller/Core/Src/main.c` | この時間フィードバックが無いと`ROBOMAS_ERROR`へ |
| `HOMING_UPPER_ARM_TIMEOUT_MS` / `HOMING_LOWER_ARM_TIMEOUT_MS` | 10000 ms | `common/arm/inc/arm.h` | ホーミング全体のタイムアウト．ESP32側の待ち時間も同じ値を参照する |
| `HOMING_DONE_ACK_RETRY_MS` | 200 ms | `robomas_controller/Core/Src/main.c` | `HOMING_DONE`の再送間隔 |
| `HOMING_DONE_ACK_MAX_RETRY` | 5 回 | `robomas_controller/Core/Src/main.c` | `HOMING_DONE`の最大再送回数 |
| `HOMING_REQUEST_RETRY_MS` | 100 ms | `ESP32/controller/main/robot/arm_command.c` | `HOMING`要求(Main Controller→Arm)の再送間隔 |

`HOMING_UPPER/LOWER_DEG_RPM`・`HOMING_UPPER/LOWER_R_RPM`は，上記タイムアウトに対して`HOMING_TIMEOUT_MARGIN_RATE`分の余裕(既定60%)を持って完了するよう`common/arm/inc/arm.h`で自動計算している．

---

## Status_LED

`status_led_update()`が，lower/upperそれぞれのアームが`ROBOMAS_READY`(r軸・deg軸とも)かどうかをStatus_LEDの点滅回数で表示する．周期的(mainループ毎)に呼び出す非ブロッキングの実装．

| 状態 | 表示 |
| :--- | :--- |
| 両方`READY` | 常時点灯 |
| lowerのみ未`READY` | 1回点滅 |
| upperのみ未`READY` | 2回点滅 |
| 両方とも未`READY` | 3回点滅 |

点滅回数 = (lowerが未READYなら+1) + (upperが未READYなら+2) で算出しており，1回点滅→短い消灯→1回点滅…を`blink_count`回繰り返した後，`STATUS_LED_BLINK_PAUSE_MS`(700ms)の消灯を挟んで最初から繰り返す．

| 定数 | 値 | 説明 |
| :--- | ---: | :--- |
| `STATUS_LED_BLINK_ON_MS` | 150 ms | 1回の点滅の点灯時間 |
| `STATUS_LED_BLINK_OFF_MS` | 150 ms | 点滅同士の間の消灯時間 |
| `STATUS_LED_BLINK_PAUSE_MS` | 700 ms | 1周(blink_count回)表示し終えた後の消灯時間 |
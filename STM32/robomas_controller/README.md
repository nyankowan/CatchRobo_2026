# STM32/robomas_controller
CAN1            CMMAND
CAN2            ROBOMAS
GPIO_Input_1
GPIO_Input_2
GPIO_Input_3
GPIO_Input_4
GPIO_Input_5
GPIO_Input_6
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
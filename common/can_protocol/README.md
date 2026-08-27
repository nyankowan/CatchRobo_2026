# CAN Communication Protocol

ESP32およびSTM32間で使用するCAN通信プロトコルの仕様を定義する．

## 1. 基本仕様

| 項目             | 仕様                   |
| -------------- | -------------------- |
| CAN            | Classical CAN        |
| CAN ID         | Standard ID (11 bit) |
| 最大DLC          | 8 byte               |
| Byte order     | Little-endian        |
| CAN ID `0x000` | 使用禁止                 |

CANの調停仕様により，CAN IDが小さいメッセージほど高い優先度を持つ．

複数byteの整数値はLittle-endianで格納する．

例として `int16_t value = 0x1234` の場合，

| Byte |  Value |
| ---: | -----: |
|    0 | `0x34` |
|    1 | `0x12` |

となる．

---

## 2. CAN ID一覧

|  CAN ID | Name                           | DLC | Description              |
| ------: | ------------------------------ | --: | ------------------------ |
| `0x001` | `EMERGENCY_STOP`               |   0 | 緊急停止                     |
| `0x005` | `ROBOMAS_CONTROLLER_HEARTBEAT` |   0 | Robomas Controller生存確認   |
| `0x006` | `UPPER_ARM_HEARTBEAT`          |   0 | Upper Arm Controller生存確認 |
| `0x007` | `LOWER_ARM_HEARTBEAT`          |   0 | Lower Arm Controller生存確認 |
| `0x010` | `UPPER_HOMING_ACK`             |   1 | Upper Arm Homing開始確認     |
| `0x011` | `UPPER_HOMING`                 |   1 | Upper Arm Homing開始要求     |
| `0x015` | `UPPER_HOMING_DONE_ACK`        |   1 | Upper Arm Homing完了通知の確認  |
| `0x016` | `UPPER_HOMING_DONE`            |   1 | Upper Arm Homing完了通知     |
| `0x020` | `LOWER_HOMING_ACK`             |   1 | Lower Arm Homing開始確認     |
| `0x021` | `LOWER_HOMING`                 |   1 | Lower Arm Homing開始要求     |
| `0x025` | `LOWER_HOMING_DONE_ACK`        |   1 | Lower Arm Homing完了通知の確認  |
| `0x026` | `LOWER_HOMING_DONE`            |   1 | Lower Arm Homing完了通知     |
| `0x100` | `UPPER_ARM_COMMAND`            |   6 | Upper Armへの操作指令          |
| `0x101` | `LOWER_ARM_COMMAND`            |   5 | Lower Armへの操作指令          |
| `0x102` | `ASSEMBLE_COMMAND`             |   1 | Assemble機構への操作指令         |
| `0x3F0` | `ERROR_CODE`                   |   1 | エラー通知                    |

DLCは `can_protocol_get_dlc()` によりCAN IDから決定する．

未知のCAN IDに対しては `CAN_DLC_INVALID (-1)` を返す．

---

## 3. Heartbeat

Heartbeatは各ControllerからMain Controllerへの生存確認に使用する．

Heartbeatにはデータを含めず，対応するCAN IDのフレームを受信したこと自体を生存通知として扱う．

|  CAN ID | Sender               |
| ------: | -------------------- |
| `0x005` | Robomas Controller   |
| `0x006` | Upper Arm Controller |
| `0x007` | Lower Arm Controller |

DLCはすべて `0` とする．

Heartbeatの送信周期とTimeout時間は以下とする．

```c
#define HEARTBEAT_MS         300
#define HEARTBEAT_TIMEOUT_MS 1000
```

各Controllerは300 ms周期でHeartbeatを送信する．

Main Controllerが1000 ms以上Heartbeatを受信できなかった場合，そのControllerとの通信が失われたものとして扱う．

通信喪失を検出した場合は，対応するError Codeを用いてエラーとして処理する．

---

## 4. Homing

Upper ArmおよびLower ArmのHomingでは，

* Homing要求
* Homing ACK
* Homing完了通知
* Homing完了ACK

の4種類のメッセージを使用する．
ホーミング処理の中枢を担うのはrobomas_contorollerのみで，
サーボ制御基板はHOMINGを受け取ったときのみ，サーボを初期化するだけである．
そして，HOMIG_ACK,HOMING_DONEを返さない．

基本的な通信シーケンスは以下とする．

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

Upper Armでは以下のCAN IDを使用する．

|  CAN ID | Name                    | Direction  |
| ------: | ----------------------- | ---------- |
| `0x010` | `UPPER_HOMING_ACK`      | Arm → Main |
| `0x011` | `UPPER_HOMING`          | Main → Arm |
| `0x015` | `UPPER_HOMING_DONE_ACK` | Main → Arm |
| `0x016` | `UPPER_HOMING_DONE`     | Arm → Main |

Lower Armでは以下のCAN IDを使用する．

|  CAN ID | Name                    | Direction  |
| ------: | ----------------------- | ---------- |
| `0x020` | `LOWER_HOMING_ACK`      | Arm → Main |
| `0x021` | `LOWER_HOMING`          | Main → Arm |
| `0x025` | `LOWER_HOMING_DONE_ACK` | Main → Arm |
| `0x026` | `LOWER_HOMING_DONE`     | Arm → Main |

すべてのHomingメッセージのDLCは `1` とする．

Byte 0にはsequence numberを格納する．

| Byte | Type                         | Name       | Description     |
| ---: | ---------------------------- | ---------- | --------------- |
|    0 | `can_sequence_t` (`uint8_t`) | `sequence` | Sequence Number |

ACKには，対応する要求または通知と同じsequence numberを格納する．

例：

```text
UPPER_HOMING          : sequence = 15
UPPER_HOMING_ACK      : sequence = 15
UPPER_HOMING_DONE     : sequence = 15
UPPER_HOMING_DONE_ACK : sequence = 15
```

sequence numberは `0` ～ `255` を循環して使用する．

---

## 5. Upper Arm Command

### CAN ID

```text
0x100
```

### DLC

```text
6
```

### データフォーマット

| Byte | Type      | Name | Description |
| ---: | --------- | ---- | ----------- |
|  0-1 | `int16_t` | `x`  | X方向指令 (mm)  |
|  2-3 | `int16_t` | `y`  | Y方向指令 (mm)  |
|  4-5 | `int16_t` | `z`  | Z方向指令 (mm)  |

すべてLittle-endianで格納する．

```text
Byte 0 : x LSB
Byte 1 : x MSB

Byte 2 : y LSB
Byte 3 : y MSB

Byte 4 : z LSB
Byte 5 : z MSB
```

C言語上では `upper_arm_t` として表現する．

```c
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} upper_arm_t;
```

---

## 6. Lower Arm Command

### CAN ID

```text
0x101
```

### DLC

```text
5
```

### データフォーマット

| Byte | Type      | Name   | Description |
| ---: | --------- | ------ | ----------- |
|  0-1 | `int16_t` | `x`    | X方向指令 (mm)  |
|  2-3 | `int16_t` | `y`    | Y方向指令 (mm)  |
|    4 | `uint8_t` | `hand` | Hand操作      |

`hand`は各bitを以下のように使用する．

| Bit | Name     | Description |
| --: | -------- | ----------- |
|   0 | `left`   | Left hand   |
|   1 | `middle` | Middle hand |
|   2 | `right`  | Right hand  |
|   3 | `expand` | Expand      |
| 4-7 | Reserved | 使用しない       |

例えば，

```text
0000 0101
```

の場合，

```text
left   = 1
middle = 0
right  = 1
expand = 0
```

となる．

C言語上では `lower_arm_t` として表現する．

```c
typedef struct {
    int16_t x;
    int16_t y;
    union {
        uint8_t hand;
        struct {
            uint8_t left   : 1;
            uint8_t middle : 1;
            uint8_t right  : 1;
            uint8_t expand : 1;
            uint8_t        : 4;
        };
    };
} lower_arm_t;
```

---

## 7. Assemble Command

### CAN ID

```text
0x102
```

### DLC

```text
1
```

Byte 0をAssemble機構への角度指令として使用する．

| Byte | Type                         | Name  | Description |
| ---: | ---------------------------- | ----- | ----------- |
|    0 | `assemble_deg_t` (`uint8_t`) | `deg` | 角度指令 0～90°  |

有効範囲は `0` ～ `90` とする．

---

## 8. Emergency Stop

### CAN ID

```text
0x001
```

### DLC

```text
0
```

Emergency StopはCAN通信上で最も高い優先度を持つメッセージとする．

受信したControllerは直ちに安全な状態へ移行する．

Emergency StopはCAN通信自体が失われた場合には受信できないため，Heartbeat Timeoutによる異常検出と併用する．

---

## 9. Error Code

### CAN ID

```text
0x3F0
```

### DLC

```text
1
```

Byte 0にError Codeを格納する．

| Byte | Type                      | Name         | Description |
| ---: | ------------------------- | ------------ | ----------- |
|    0 | `can_error_t` (`uint8_t`) | `error_code` | Error Code  |

Error Codeは `can_error.h` に定義する．

現在定義されているError Codeは以下の通り．

|  Value | Name                               | Description                                             |
| -----: | ----------------------------------- | -------------------------------------------------------- |
| `0x01` | `CAN_ERROR_LOWER_R_LOST_CONTROL`    | 下アームRロボマスからのフィードバックが途絶えた           |
| `0x02` | `CAN_ERROR_LOWER_DEG_LOST_CONTROL`  | 下アームDEGロボマスからのフィードバックが途絶えた         |
| `0x03` | `CAN_ERROR_UPPER_R_LOST_CONTROL`    | 上アームRロボマスからのフィードバックが途絶えた           |
| `0x04` | `CAN_ERROR_UPPER_DEG_LOST_CONTROL`  | 上アームDEGロボマスからのフィードバックが途絶えた         |
| `0x10` | `CAN_ERROR_LOWER_HOMING_TIMEOUT`    | 下アームのホーミングが規定時間内に完了しなかった          |
| `0x11` | `CAN_ERROR_UPPER_HOMING_TIMEOUT`    | 上アームのホーミングが規定時間内に完了しなかった          |
| `0x20` | `CAN_ERROR_LOWER_HOMING_REJECTED`   | 下アームがROBOMAS_ERROR中のためホーミング要求を無視した   |
| `0x21` | `CAN_ERROR_UPPER_HOMING_REJECTED`   | 上アームがROBOMAS_ERROR中のためホーミング要求を無視した   |

新しいError Codeを追加する場合は，既存の値と重複しない値を `can_error.h` に追加する．

---

## 10. C言語でのデータ表現

CANデータは以下のunionで表現する．

```c
typedef union {
    uint8_t raw[CAN_DLC_MAX];

    can_sequence_t homing_sequence;
    can_error_t error_code;
    lower_arm_t lower_arm;
    upper_arm_t upper_arm;
    assemble_deg_t assemble_deg;
} can_data_t;
```

現在ESP32およびSTM32はいずれもLittle-endianであるため，CANデータと構造体のメモリ表現を共用している．

ただし，C構造体にはpaddingが挿入される可能性がある．

例えば `lower_arm_t` のCAN上のデータサイズは5 byteだが，

```c
sizeof(lower_arm_t)
```

が5 byteになることは保証されない．

したがって，CAN送信時のDLCを `sizeof()` から決定してはならない．

必ず，

```c
can_protocol_get_dlc(id)
```

によってCAN IDに対応するDLCを取得する．

`can_data_t` 自体は，

```c
_Static_assert(sizeof(can_data_t) == CAN_DLC_MAX,
               "can_data_t must be exactly 8 bytes");
```

によって8 byteであることをコンパイル時に確認する．

---

## 11. CAN ID追加時のルール

新しいCANメッセージを追加する場合，以下を定義する．

1. CAN ID
2. メッセージ名
3. 送信元
4. 送信先
5. DLC
6. 各Byte / Bitの意味
7. データ型
8. 単位
9. 有効範囲
10. 送信周期または送信条件
11. Timeout時の処理

CAN IDを決定する際はメッセージの優先度を考慮すること．

CAN IDが小さいほどCANバス上で高い優先度を持つ．

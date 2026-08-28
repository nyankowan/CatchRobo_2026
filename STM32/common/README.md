# STM32/common

STM32側のプロジェクト(`lower_arm_servo`, `upper_arm_servo`, `robomas_controller`)全てから共通で使われるコード．

ESP32とも共有する [common/](../../common/) (CAN ID定義・座標変換)とは別に，STM32のHALを使うコードはこちらに置く．

## Directory structure

```text
STM32/common/
└── can/
    └── STM32 HAL CANを使ったCAN送信のラッパー
```

## can

`STM32/common/can/inc/stm_can.h`

```c
HAL_StatusTypeDef stm_can_send(
    CAN_HandleTypeDef *hcan,
    const can_command_data_t *com
);
```

`common/can_protocol` で定義された `can_command_data_t` を受け取り，CAN IDから `can_protocol_get_dlc()` でDLCを決定した上で `HAL_CAN_AddTxMessage()` を呼び出す．

DLCが不正(未知のCAN IDなど)な場合は送信せず `HAL_ERROR` を返す．

受信については各プロジェクトの `HAL_CAN_RxFifo0MsgPendingCallback()` 内でそれぞれ実装しており，共通化していない．

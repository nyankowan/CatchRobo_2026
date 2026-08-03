# CONTROLLER
Bluetoothによるデータの受け取りと，CAN通信で，末端マイコンに指令を送る．

## components
### can
CAN通信をサポート
can_init_and_start()で，500KBTSCAN通信をスタートする．
.clk_src = TWAI_CLK_SRC_DEFAULT (= SOC_MOD_CLK_APB),
.quanta_resolution_hz = 8000000,
.brp = 0,
.prop_seg = 0,
.tseg_1 = 11,
.tseg_2 = 4,
.sjw = 2,
.ssp_offset = 0,
.triple_sampling = false

tx,rx_xueue = 5,

このとき，can_error_handling_task()により，can通信のバスオフを検知すると，リカバーする．

#### can_protocol.h
データをどこのIDに送るか，どのようなデータが詰め込まれているか管理している．
```c can_protocol.h
typedef enum{
    // 11 or 29 bit identifier
    CAN_ID_ROBOMAS_CONTROLLER = 0x100,
    CAN_ID_SERVO_CONTROLLER_1 = 0x200,
    CAN_ID_SERVO_CONTROLLER_2 = 0x300,
    CAN_ID_SERVO_CONTROLLER_3 = 0x400,
}can_id_t;

typedef struct{
    can_id_t id;
    uint8_t data[8];
}can_command_data_t;
```
データの詰め込みと取り出しを担う関数は

データを送るときはcan_tx(can_command_data_t)で，データをtxキューに追加することにより送る．
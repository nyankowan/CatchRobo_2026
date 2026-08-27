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
ホーミング処理はアームごとに行う.    


### ROBOMASTER_STATE
```C
typedef enum{
  ROBOMAS_INITIAL,  // 初回起動時状態
  ROBOMAS_HOMING,   // 速度制御によって，リミットスイッチ位置まで回転する
  ROBOMAS_IDLE,     // ホーミング終了による他のホーミングを待機
  ROBOMAS_READY,    // ユーザが制御可能な状態
}robomas_state_t;
```
| Value             |  explanation |
| :---------------- | :----------- |
|`ROBOMAS_INITIAL`  | マイコン起動時の初期状態．Main Controllerからの指令により `ROBOMAS_HOMING` に遷移する．|
|`ROBOMAS_HOMING`   | ホーミング処理中の状態．リミットスイッチからの信号で `ROBOMAS_IDLE` に遷移する．|
|`ROBOMAS_IDLE`     | ホーミング処理を終了し，もう片方のホーミングの待機中．両ロボマスがこの状態になると`ROBOMAS_IDLE`に遷移する．|
|`ROBOMAS_READY`    | Main Controllerからの座標指令をもとに制御を行える状態．ホーミング指令により `ROBOMAS_HOMING` に遷移する．|

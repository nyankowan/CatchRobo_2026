
# STM32

キャチロボ2026で使用する4台のSTM32(Nucleo)のプログラムをまとめたディレクトリ．
それぞれSTM32CubeMXで生成された独立したCMakeプロジェクトであり，共通してMain Controller(ESP32)とCANで接続される．

## Directory structure

```text
STM32/
├── common/
│   └── STM32プロジェクト間の共通コード(CANラッパー等)．詳細は common/README.md 参照
│
├── robomas_controller/
│   └── 両アームのr軸・deg軸(Robomaster M3508/M2006)の制御．ホーミング処理の中枢
│
├── lower_arm_servo/
│   └── 下アームのハンド(Left/Middle/Right/Expand)とShaftサーボの制御
│
├── upper_arm_servo/
│   └── 上アームのShaft/Zサーボの制御(未実装，CubeMX生成のスケルトンのみ)
│
└── assemble_servo/
    └── 整理機構(Assemble)のサーボ角度制御
```

各プロジェクトの詳細は，それぞれのディレクトリ内のREADME.mdを参照．

## 共通ライブラリのリンク方法

STM32/common (CANラッパー) と，ESP32とも共有する common/ (CAN ID定義・座標変換) を，各プロジェクトの `CMakeLists.txt` から以下のようにリンクする．

### 各プロジェクトのCMakeLists.txt
```CMake
# STM32共通CAN
add_subdirectory(
    "${CMAKE_CURRENT_LIST_DIR}/../common/can"
    "${CMAKE_BINARY_DIR}/stm_can"
)

# ESP32 / STM32共通 CAN protocol
add_subdirectory(
    "${CMAKE_CURRENT_LIST_DIR}/../../common/can_protocol"
    "${CMAKE_BINARY_DIR}/can_protocol"
)

# ESP32 / STM32共通 座標処理
add_subdirectory(
    "${CMAKE_CURRENT_LIST_DIR}/../../common/coordinate"
    "${CMAKE_BINARY_DIR}/coordinate"
)



target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    # Add user defined libraries
    stm_can
    can_protocol
    coordinate
)
```

`assemble_servo` は座標変換を使わず角度指令をそのままPWMに変換するだけなので，`coordinate` はリンクしていない(`stm_can` と `can_protocol` のみ)．

`robomas_controller` のみ，ホーミングのタイムアウト・速度・可動域の定数を使うため，上記に加えて `common/arm` も追加でリンクしている．

```CMake
# ESP32 / STM32共通 アームパラメータ
add_subdirectory(
    "${CMAKE_CURRENT_LIST_DIR}/../../common/arm"
    "${CMAKE_BINARY_DIR}/arm"
)

target_link_libraries(${CMAKE_PROJECT_NAME}
    ...
    arm
)
```


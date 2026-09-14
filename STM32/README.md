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
│   └── 上アームのShaft/Zサーボの制御
│
└── assemble_servo/
    └── 整理機構(Assemble)のサーボ角度制御
```

各プロジェクトの詳細は，それぞれのディレクトリ内のREADME.mdを参照．

| プロジェクト | Nucleo | 基板 | README |
| :--- | :--- | :--- | :--- |
| `robomas_controller` | NUCLEO-F446RE | `Kicad/STM32F446RE_robomas_controller` | [robomas_controller/README.md](robomas_controller/README.md) |
| `lower_arm_servo` | NUCLEO-F303K8 | `Kicad/STM32F303K8_lower_arm_servo` | [lower_arm_servo/README.md](lower_arm_servo/README.md) |
| `upper_arm_servo` | NUCLEO-F303K8 | `Kicad/STM32F303K8_lower_arm_servo`(流用) | [upper_arm_servo/README.md](upper_arm_servo/README.md) |
| `assemble_servo` | NUCLEO-F303K8 | `Kicad/STM32F303K8_lower_arm_servo`(流用) | [assemble_servo/README.md](assemble_servo/README.md) |

CAN ID・データフォーマットの仕様は [common/can_protocol/README.md](../common/can_protocol/README.md) を参照．

## ビルド

各プロジェクトのディレクトリで以下を実行する．

```bash
cmake --preset Release
cmake --build --preset Release
```

4プロジェクト共通のDev Container(`STM32/.devcontainer`)を用意しており，どのプロジェクトも同じコンテナ内でビルドできる(詳細は[リポジトリ直下のREADME.md](../README.md)を参照)．

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

`robomas_controller` / `lower_arm_servo` / `upper_arm_servo` は，ホーミング原点・可動域・出場チーム(`ROBOT_TEAM`)の定数を使うため，上記に加えて `common/arm` も追加でリンクしている(`assemble_servo` は使わないためリンクしていない)．

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


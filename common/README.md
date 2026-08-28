# common

ESP32(Main Controller)とSTM32(各種Controller)の両方から参照される，プラットフォーム非依存のC言語共有コード．

CANの通信内容や座標変換など，複数のマイコンで同じ実装・同じ値を使う必要があるものをここに置く．
片方のマイコンでしか使わないコードはここに置かず，それぞれのプロジェクト内に置くこと．

## Directory structure

```text
common/
├── arm/
│   └── アームの物理パラメータ・ホーミング関連定数(可動域，タイムアウト，速度など)
│
├── can_protocol/
│   └── CAN IDとデータフォーマットの対応定義
│
└── coordinate/
    └── 直交座標(x, y) ⇔ 極座標(r, θ) の変換
```

## arm

`common/arm/inc/arm.h`

両アーム(Upper/Lower)に共通する物理パラメータと，ホーミング処理に関する定数をまとめる．

- 可動域: `*_ARM_R_RANGE`, `*_ARM_R_MIN`, `*_ARM_DEG_RANGE`, `*_ARM_DEG_MIN`
- 原点座標: `LOWER_ARM_HOME_COORDINATE`, `UPPER_ARM_HOME_COORDINATE`
- ホーミングのタイムアウト・速度: `HOMING_*_ARM_TIMEOUT_MS`, `HOMING_*_DEG_RPM`, `HOMING_*_R_RPM`

STM32(`robomas_controller`)とESP32(`arm_command.c`)の両方が，同じ可動域・同じタイムアウト値を参照するために存在する．
どちらか一方だけ値を変えると，可動域チェックやタイムアウト判定がズレるので注意．

## can_protocol

CAN IDとデータフォーマットの仕様．詳細は [can_protocol/README.md](can_protocol/README.md) を参照．

## coordinate

`common/coordinate/inc/coordinate.h`

```c
typedef struct { double x; double y; } direct_t; // 直交座標
typedef struct { double r; double theta; } polar_t; // 極座標 (theta: rad, 0〜2π)

polar_t to_polar(direct_t d);
```

両アームは物理的には「アームの伸び(r)」と「アームの偏角(θ)」で制御されるが，ジョイスティック入力やCAN通信上は直交座標(x, y)で扱う方が扱いやすいため，この変換関数を共有している．

## ビルドについて

各ディレクトリはそれぞれ独立したCMakeライブラリ(STM32側)／ESP-IDFコンポーネント(ESP32側)として構成されており，`add_subdirectory` または `EXTRA_COMPONENT_DIRS` 経由で各プロジェクトからリンクされる．
具体的なリンク方法は [STM32/README.md](../STM32/README.md) を参照．

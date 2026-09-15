# Kicad

キャチロボ2026で使用する回路データ(KiCad 10.0)と，その部品リスト．

## Directory structure

```text
Kicad/
├── ESP/
│   └── ESP Main Controller基板(ESP32-DevKitC-32E)
│
├── STM32F303K8_lower_arm_servo/
│   └── サーボ制御基板(lower_arm_servo/upper_arm_servo/assemble_servoで共用)
│
├── STM32F446RE_robomas_controller/
│   └── ロボマス制御基板(NUCLEO-F446RE)
│
├── servo/
│   └── サーボ用補助基板
│
└── library/
    └── 共通シンボル・フットプリント
```

各基板に対応するファームウェアは [STM32/README.md](../STM32/README.md) および [ESP32/controller/README.md](../ESP32/controller/README.md) を参照．

## 部品リスト

このディレクトリにあるキャチロボ2026に向けて作った回路データのほかにも，
岡ロボで使われるCANトランシーバー回路も使用している．

### 回路基板実装部品
#### マイコン基板
- lower_arm_servo基板×3（上側アームと整理機構の制御基板に流用）     
以下一つあたりの部品
    - nucleo-f303k8
    - 3φLED×1
    - 470Ω抵抗×1
    - 5ピンソケット×1
    - 6ピンソケット×1
    - 15pin socket×2

- robomas_controller基板×1
    - nucleo-f446re
    - 19*2ピンソケット×2
    - hirose 2pin(DF1EC-2P-2.5DSA(35))×7
    - 5ピンソケット×2
    - 300Ω抵抗×1
    - φ3LED×1

- ESP Main controller基板×1
    - ESP32-DevKitC-32E
    - 19ピンソケット×2
    - 2ピンソケット×2
    - 5ピンソケット×1
    - φ3LED×7
    - 470Ω抵抗×7

#### 電源基板(N先輩制作)
- catchrobo_2026電源回路×1
    - リレー(942H-2C-24DS)×1
    - ヒューズ×3
    - 470μF×3
    - xt30オス×5
    - xt30メス×4
    - φ5LED×3
    - {1200, 330}Ω抵抗×3
- catchrobo_2026分電回路×2
    - 470μF×1
    - φ5LED×1
    - {1200, 330}Ω抵抗×1
    - xt30メス×4
    - xt30オス×1

#### 補助基板
- servo基板×3   
一つあたりの部品
    - 基板取付用XT30オス(XT30PW-M)×1
    - 1kΩ抵抗×1
    - 220Ω抵抗×5
    - φ3LED×1
    - 1000μF(50PX1000MEFC12.5X25)×1
    - 0.1μF(積層セラミックコンデンサー 0.1μF100V X7R 5mmピッチ)
    - 折れ6pin hirose (DF1E-6P-2.5DS(35))×1
    - 3ピンソケット×5（ヘッダでも可）

- canトランシーバ基板ver1.b×2
- canトランシーバ基板ver1.0×1
- canトランシーバ基板ver1.0 終端抵抗なし×3

- 6.6V -> 5V DCDCコンバータ×1
    - xt30オス×1
    - xt30メス×1

### コード類
#### 電源線
- 電源供給用micro-B×3（2m級×2）
- 電源供給用micro-B×1 （もしくは 2pin オスxt30メス×1）
- 電源供給用mini-B×1　（もしくは 2pin オスxt30メス×1）
- ロボマス電源延長用 xt30 オスメス ×4（15cm程度）
- xt30延長ケーブル×∞

#### 信号線
- リミットスイッチ 2pin hirose オス×6（2m級）
- マイコン間canbus接続用2pin hirose オスオス×4（2m級）
- サーボ電源供給用 xt30オスメス×3（2m級）
- ロボマスcan延長用 2pin hirose オスメス×4（15cm程度）
- canトランシーバ接続用6pinオスメスコネクタ×1（1pin NC）

#### その他
- サーボ延長用 3pin オスメス×5

### 非常停止ボタン　動作ランプなど
- プッシュロック・ターンリセットスイッチ×1
- φ20以上緑LED×1(HWシリーズ パイロットライトΦ22(突形 LED))
- 22VLipo×3
    - xt60オス - xt30メス変換ケーブル×1
- 6.6VLipo×1(並列につなぐ場合×2)
    - xt60オス - xt30メス変換ケーブル×1



    
    




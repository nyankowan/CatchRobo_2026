#ifndef PROCON_DATA_H
#define PROCON_DATA_H

#include <stdint.h>
#include "sdkconfig.h"

#define MAX_MYPAD CONFIG_BLUEPAD32_MAX_DEVICES

#define PRESSED(button, pv_button) (button==1 && pv_button==0)

// mypad構造体の定義
typedef struct {
    unsigned int A      : 1;      // Aボタン: 1=押されている, 0=押されていない
    unsigned int B      : 1;      // Bボタン
    unsigned int X      : 1;      // Xボタン
    unsigned int Y      : 1;      // Yボタン
    unsigned int UP     : 1;     // 十字キー上
    unsigned int DOWN   : 1;   // 十字キー下
    unsigned int LEFT   : 1;   // 十字キー左
    unsigned int RIGHT  : 1;  // 十字キー右
    unsigned int L      : 1;      // Lボタン
    unsigned int R      : 1;      // Rボタン
    unsigned int ZL     : 1;     // ZLボタン
    unsigned int ZR     : 1;     // ZRボタン
    unsigned int TL     : 1;     // 左スティックの押し込み
    unsigned int TR     : 1;     // 右スティックの押し込み
    unsigned int MINUS  : 1; // セレクトボタン
    unsigned int PLUS   : 1;  // スタートボタン
    unsigned int HOME   : 1;  // システムボタン oobイベントで取得可能かも
    unsigned int CAPTURE: 1; // キャプチャボタン
    int16_t LX;     // 左スティックのX軸: -512~512
    int16_t LY;     // 左スティックのY軸: -512~512
    int16_t RX;     // 右スティックのX軸: -512~512
    int16_t RY;     // 右スティックのY軸: -512~512
    uint8_t battery_level; // バッテリー残量 (0-255)
    unsigned int connected: 1; // コントローラーが接続されているかどうか
} mypad_t;

extern const mypad_t EMPTY_MYPAD;

void get_mypad(mypad_t mp[MAX_MYPAD]);
void mypad_dump(mypad_t* pad);
#endif//PROCON_DATA_H
#ifndef CAN_ERROR_H
#define CAN_ERROR_H

#include <stdint.h>

typedef uint8_t can_error_t;

/*
 * CAN_ID_ERROR_CODE (0x3F0) のペイロード(1byte)として送信するエラーコード。
 * robomas_controllerが検知した異常をMain Controller(ESP32)へ通知するために使う。
 */

// ロボマスからのフィードバックが ROBOMAS_FEEDBACK_TIMEOUT_MS 途絶えた(ROBOMAS_ERRORへ遷移した)
#define CAN_ERROR_LOWER_R_LOST_CONTROL     0x01
#define CAN_ERROR_LOWER_DEG_LOST_CONTROL   0x02
#define CAN_ERROR_UPPER_R_LOST_CONTROL     0x03
#define CAN_ERROR_UPPER_DEG_LOST_CONTROL   0x04

// ホーミングが規定時間内に完了しなかった(リミットスイッチ未検出など)
#define CAN_ERROR_LOWER_HOMING_TIMEOUT     0x10
#define CAN_ERROR_UPPER_HOMING_TIMEOUT     0x11

// ROBOMAS_ERROR中にホーミング要求を受けたため無視した
#define CAN_ERROR_LOWER_HOMING_REJECTED    0x20
#define CAN_ERROR_UPPER_HOMING_REJECTED    0x21

#endif //CAN_ERROR_H
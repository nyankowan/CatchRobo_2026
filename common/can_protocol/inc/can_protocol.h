#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H
#include <stdint.h>
#include "can_error.h"

#define HEARTBEAT_MS 300
#define HEARTBEAT_TIMEOUT_MS 1000

#define CAN_DLC_MAX 8 //MAX
#define CAN_DLC_INVALID (-1)

typedef int8_t can_dlc_t;

//Don't use ID 0x00
//Smaller ID is higher priority
#define CAN_ID_NUM_ITEMS 16
typedef enum{
    CAN_ID_EMERGENCY_STOP=              0x001,

    CAN_ID_ROBOMAS_CONTROLLER_HEARTBEAT=0x005,
    CAN_ID_UPPER_ARM_HEARTBEAT=         0x006,
    CAN_ID_LOWER_ARM_HEARTBEAT=         0x007,

    CAN_ID_UPPER_HOMING_ACK=            0x010,
    CAN_ID_UPPER_HOMING=                0x011,
    CAN_ID_LOWER_HOMING_ACK=            0x020,
    CAN_ID_LOWER_HOMING=                0x021,

    CAN_ID_UPPER_HOMING_DONE_ACK=       0x015,
    CAN_ID_UPPER_HOMING_DONE=           0x016,
    CAN_ID_LOWER_HOMING_DONE_ACK=       0x025,
    CAN_ID_LOWER_HOMING_DONE=           0x026,

    CAN_ID_UPPER_ARM_COMMAND=           0x100,
    CAN_ID_LOWER_ARM_COMMAND=           0x101,
    CAN_ID_ASSEMBLE_COMMAND=            0x102,

    CAN_ID_ERROR_CODE=                  0x3F0
}can_id_t;

typedef struct{
    int16_t x;//0-1
    int16_t y;//2-3
    union{
        uint8_t hand;//4
        struct{
            uint8_t left         :1;
            uint8_t middle       :1;
            uint8_t right        :1;
            uint8_t expand       :1;
            uint8_t shaft_rotate :1; //1:ハンドの向きを90度回転させる
            uint8_t              :3;
        };
    };
}lower_arm_t;

typedef struct{
    int16_t x;//0-1
    int16_t y;//2-3
    int16_t z;//4-5
}upper_arm_t;

typedef uint8_t can_sequence_t;
typedef uint8_t assemble_deg_t;

// Both ESP and STM use little-endian, so this works.
// Be careful about padding; sizeof(lower_arm_t) may not equal
// can_protocol_get_dlc(CAN_ID_LOWER_ARM_COMMAND).
typedef union{
    uint8_t raw[CAN_DLC_MAX];

    can_sequence_t homing_sequence;
    can_error_t error_code;
    lower_arm_t lower_arm;
    upper_arm_t upper_arm;
    assemble_deg_t assemble_deg;

} can_data_t;

typedef struct{
    can_id_t id;
    can_data_t data;
}can_command_data_t;

static inline can_dlc_t can_protocol_get_dlc(can_id_t id)
{
    switch (id) {
    case CAN_ID_EMERGENCY_STOP:
    case CAN_ID_ROBOMAS_CONTROLLER_HEARTBEAT:
    case CAN_ID_UPPER_ARM_HEARTBEAT:
    case CAN_ID_LOWER_ARM_HEARTBEAT:
        return 0;

    case CAN_ID_UPPER_HOMING_ACK:
    case CAN_ID_UPPER_HOMING:
    case CAN_ID_LOWER_HOMING_ACK:
    case CAN_ID_LOWER_HOMING:
    case CAN_ID_UPPER_HOMING_DONE_ACK:
    case CAN_ID_UPPER_HOMING_DONE:
    case CAN_ID_LOWER_HOMING_DONE_ACK:
    case CAN_ID_LOWER_HOMING_DONE:
        return 1;

    case CAN_ID_UPPER_ARM_COMMAND:
        return 6;

    case CAN_ID_LOWER_ARM_COMMAND:
        return 5;

    case CAN_ID_ASSEMBLE_COMMAND:
    case CAN_ID_ERROR_CODE:
        return 1;

    default:
        return CAN_DLC_INVALID;
    }
}


_Static_assert(sizeof(lower_arm_t) <= CAN_DLC_MAX,
               "lower_arm_t is too large");

_Static_assert(sizeof(upper_arm_t) <= CAN_DLC_MAX,
               "upper_arm_t is too large");

_Static_assert(sizeof(can_data_t) == CAN_DLC_MAX,
               "can_data_t must be exactly 8 bytes");

#endif //CAN_PROTOCOL_H
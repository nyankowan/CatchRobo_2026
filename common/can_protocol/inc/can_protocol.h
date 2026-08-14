#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H
#include <stdint.h>

#define CAN_DLC 8 //MAX

//Don't use ID 0x00
#define CAN_ID_NumItems 6
typedef enum{
    CAN_ID_UPPER_HOMING=0x01,
    CAN_ID_LOWER_HOMING=0x02,
    CAN_ID_UPPER_HOMING_DONE=0x03,
    CAN_ID_LOWER_HOMING_DONE=0x04,
    CAN_ID_UPPER_ARM=0x100,
    CAN_ID_LOWER_ARM=0x101,
}can_id_t;

typedef struct{
    int16_t x;//1,2
    int16_t y;//3.4
    union{
        uint8_t raw;//5
        struct{
            uint8_t left    :1;
            uint8_t middle  :1;
            uint8_t right   :1;
            uint8_t expand  :1;
            uint8_t         :4;
        };
    }hand;
    uint8_t reserved[3]//6,7,8
    
}lower_arm_t;

typedef struct{
    int16_t x;//1.2
    int16_t y;//3,4
    int16_t z;//5.6
    int16_t reserved;//7,8
}upper_arm_t;
typedef union{
    uint8_t raw[8];
    lower_arm_t lower_arm;
    upper_arm_t upper_arm;

} can_data_t;

typedef struct{
    can_id_t id;
    can_data_t data;
}can_command_data_t;
#endif //CAN_PROTOCOL_H
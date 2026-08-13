#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H
#include <stdint.h>

//Don't use ID 0x00
#define CAN_ID_NumItems 5
typedef enum{
    CAN_ID_UPPER_HOMING=0x01,
    CAN_ID_LOWER_HOMING=0x02,
    CAN_ID_UPPER_HOMING_DONE=0x03,
    CAN_ID_LOWER_HOMING_DONE=0x04,
    CAN_ID_COORDINATE=0x100,
}can_id_t;
typedef union{
    uint8_t raw[8];

    struct {
        struct {
            uint8_t x;//1
            uint8_t y;//2
            union{//3
                uint8_t hand;
                struct{
                    unsigned int left    : 1;
                    unsigned int middle  : 1;
                    unsigned int right   : 1;
                    unsigned int expand  : 1;
                    unsigned int         : 4;
                };
            };
        } lower;
        struct {
            uint8_t x;//4
            uint8_t y;//5
            uint8_t z;//6
        } upper;
        uint8_t resserved[2];//7,8
    } arm;
} can_data_t;

typedef struct{
    can_id_t id;
    can_data_t data;
}can_command_data_t;
#endif //CAN_PROTOCOL_H
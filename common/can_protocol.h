#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H
#include <stdint.h>

typedef enum{
    CAN_ID_COORDINATE=0X100,
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
                    uint8_t left    : 1;
                    uint8_t middle  : 1;
                    uint8_t right   : 1;
                    uint8_t expand  : 1;
                    uint8_t         : 4;
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
#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#include <stdint.h>

typedef enum{
    // 11 or 29 bit identifier
    CAN_ID_ROBOMAS_CONTROLLER = 0x100,
    CAN_ID_SERVO_CONTROLLER_1 = 0x200,
    CAN_ID_SERVO_CONTROLLER_2 = 0x300,
    CAN_ID_SERVO_CONTROLLER_3 = 0x400,
}can_id_t;

typedef struct{
    can_id_t id;
    uint8_t data[8];
}can_command_data_t;
#endif //CAN_PROTOCOL_H
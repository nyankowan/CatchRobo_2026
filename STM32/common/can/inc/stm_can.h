#ifndef STM_CAN_H
#define STM_CAN_H

#include <stdint.h>
#include "main.h"
#include "can_protocol.h"

HAL_StatusTypeDef stm_can_send(
    CAN_HandleTypeDef *hcan,
    const can_command_data_t *com
);

#endif //STM_CAN_H
#ifndef CAN_H
#define CAN_H

#include "can_protocol.h"
#include "esp_err.h"
#include "soc/gpio_num.h"

esp_err_t can_tx(can_command_data_t *com);
esp_err_t can_init_and_start(gpio_num_t tx, gpio_num_t rx);
void can_error_handling_task(void *arg);
#endif //CAN_H
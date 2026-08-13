#ifndef CAN_H
#define CAN_H

#include "can_protocol.h"
#include "esp_err.h"
#include "soc/gpio_num.h"

typedef void (*can_rx_callback_t)(const can_data_t *data);

//重い処理はタスクとして分離すること
void can_register_rx_callback(can_id_t id,can_rx_callback_t callback);

esp_err_t can_tx(can_command_data_t *com);
esp_err_t can_init_and_start(gpio_num_t tx, gpio_num_t rx);
void can_error_handling_task(void *arg);
void can_rx_task(void *arg);
#endif //CAN_H
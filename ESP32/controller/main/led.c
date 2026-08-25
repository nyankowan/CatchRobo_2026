#include <driver/gpio.h>
#include "gpio.h"
#include "led.h"

void led_init(){
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << CONTROLLER_1_LED_GPIO) |
                        (1ULL << CONTROLLER_2_LED_GPIO) |
                        (1ULL << CAN_STATUS_LED_GPIO) |
                        (1ULL << LOWER_ARM_STATUS_LED_GPIO) |
                        (1ULL << UPPER_ARM_STATUS_LED_GPIO) |
                        (1ULL << ROBOMAS_CONTROLLER_STATUS_LED_GPIO) |
                        (1ULL << STATUS_LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE, //割り込み
    };
    gpio_config(&io_conf);
}
/*
* @param level 0:low 1:high
**/
void led_set_level(gpio_num_t gpio_num, uint32_t level){
    gpio_set_level(gpio_num, level);
}
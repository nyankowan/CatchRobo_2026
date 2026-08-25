#include "gpio.h"
void led_init();
/*
* @param level 0:low 1:high
**/
void led_set_level(gpio_num_t gpio_num, uint32_t level);
#ifndef ARM_H
#define ARM_H
#include "coordinate.h"
#include <esp_err.h>
#include <stdbool.h>

#define ARM_TAG "ARM"

//homing中は動かない
void lower_arm_move(
    int16_t dx, int16_t dy,
    bool left_toggle, bool middle_toggle, 
    bool right_toggle, bool expand_toggle
);
void upper_arm_move(int16_t dx, int16_t dy, int16_t dz);

esp_err_t send_lower_arm();
esp_err_t send_upper_arm();
esp_err_t arms_init();
esp_err_t lower_arm_homing();
esp_err_t upper_arm_homing();
void arms_update();
void lower_arm_dump();
void upper_arm_dump();
void arms_dump();

#endif //ARM_H
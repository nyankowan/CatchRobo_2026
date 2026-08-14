#include "arm.h"
#include "can.h"
#include <uni.h>
#include <esp_log.h>
#include <stdint.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ARM_TAG "ARM"
#define TOGGLE(button, num) if(button==0){button=num;}else{button=0;}

void lower_arm_homing_done_notify();
void upper_arm_homing_done_notify();

static lower_arm_t lower_arm = {0};
static upper_arm_t upper_arm = {0};

static bool arms_init_already_done = false;
static bool lower_arm_homing_in_progress = false;
static bool upper_arm_homing_in_progress = false;

//homing中は動かない
void lower_arm_move(
    int16_t dx, int16_t dy,
    bool left_toggle, bool middle_toggle, 
    bool right_toggle, bool expand_toggle
    )
{
    if(lower_arm_homing_in_progress)return;

    lower_arm.x += dx;
    lower_arm.y += dy;
    if(left_toggle){TOGGLE(lower_arm.hand.left,1);}
    if(middle_toggle){TOGGLE(lower_arm.hand.middle,1);}
    if(right_toggle){TOGGLE(lower_arm.hand.right,1);}
    if(expand_toggle){TOGGLE(lower_arm.hand.expand,1);}

}

//homing中は動かない
void upper_arm_move(int16_t dx, int16_t dy, int16_t dz){
    if(upper_arm_homing_in_progress)return;

    upper_arm.x += dx;
    upper_arm.y += dy;
    upper_arm.z += dz;
}


esp_err_t send_lower_arm(){
    if(lower_arm_homing_in_progress)return ESP_ERR_NOT_ALLOWED;
    can_command_data_t com = {
        .id = CAN_ID_LOWER_ARM,
        .data.lower_arm = lower_arm,
    };
    esp_err_t e = can_tx(&com);
    if(e)ESP_LOGE(ARM_TAG, "CAN_ID_LOWER_ARM failed.");
    return e;
}

esp_err_t send_upper_arm(){
    if(upper_arm_homing_in_progress)return ESP_ERR_NOT_ALLOWED;
    can_command_data_t com = {
        .id = CAN_ID_UPPER_ARM,
        .data.upper_arm = upper_arm,
    };
    esp_err_t e = can_tx(&com);
    if(e)ESP_LOGE(ARM_TAG, "CAN_ID_UPPER_ARM failed.");
    return e;
}

esp_err_t arms_init(){
    if(arms_init_already_done)return ESP_ERR_NOT_ALLOWED;
    can_register_rx_callback(CAN_ID_LOWER_HOMING_DONE, lower_arm_homing_done_notify);
    can_register_rx_callback(CAN_ID_UPPER_HOMING_DONE, upper_arm_homing_done_notify);
    arms_init_already_done = true;
    return ESP_OK;
}

esp_err_t lower_arm_homing(){
    if(lower_arm_homing_in_progress)return ESP_ERR_NOT_FINISHED;
    if(can_tx(&(can_command_data_t){.id = CAN_ID_LOWER_HOMING}))return ESP_FAIL;
    lower_arm_homing_in_progress= true;
    
    return ESP_OK;
}

esp_err_t upper_arm_homing(){
    if(lower_arm_homing_in_progress)return ESP_ERR_NOT_FINISHED;
    if(can_tx(&(can_command_data_t){.id = CAN_ID_LOWER_HOMING}))return ESP_FAIL;
    upper_arm_homing_in_progress = true;

    return ESP_OK;
}

//Avoid heaby proccess
void lower_arm_homing_done_notify(){
    lower_arm_homing_in_progress = false;
}

//Avoid heaby proccess
void upper_arm_homing_done_notify(){
    upper_arm_homing_in_progress = false;
}

void lower_arm_dump(){
    polar_t pol = to_polar((direct_t){.x = lower_arm.x, .y = lower_arm.y});
    logi("lower_arm: CART(%4dmm,%4dmm), POR(%4.2fmm,%3.2f°), \\
          HAND{left %1d, middle %1d, right %d, expand, %1d}\n",
          lower_arm.x,lower_arm.y, pol.r, pol.theta/(2*M_PI)*360,
          lower_arm.hand.left, lower_arm.hand.middle, lower_arm.hand.right, lower_arm.hand.expand);
}

void upper_arm_dump(){
    polar_t pol = to_polar((direct_t){.x = upper_arm.x, .y = upper_arm.y});
    logi("upper_arm: CART(%4dmm,%4dmm), POR(%4.2f,%3.2f°), Z %d\n",
          upper_arm.x, upper_arm.y, pol.r, pol.theta/(2*M_PI)*360, upper_arm.z);
}

void arms_dump(){
    lower_arm_dump();
    upper_arm_dump();
}

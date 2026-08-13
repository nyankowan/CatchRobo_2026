#include "arm.h"
#include <uni.h>
#include "can.h"
#include <esp_log.h>

#define ARM_TAG "arm"

unsigned int upper_arm_z = 0;
direct_t arms_cartesian_coordinate[ARM_NUM] = {0};//[0]:lower, [1]:upepr
lower_hand_t lower_hand = {
    .left = 0,
    .middle = 0,
    .right = 0,
    .expand = 0,
};

void get_pointer_arms_cartesian_coordinate(direct_t *arm[]){
    for(int i = 0; i<ARM_NUM; i++){
        arm[i] = &arms_cartesian_coordinate[i];
    }
}

void get_pointer_lower_hand(lower_hand_t **h){
    *h = &lower_hand;
}

void get_pointer_upper_arm_z(unsigned int **z){
    *z = &upper_arm_z;
}

void set_arms(){
    can_command_data_t com = {
        .id = CAN_ID_COORDINATE,
        .data = {
            .arm.lower.x = arms_cartesian_coordinate[0].x,
            .arm.lower.y = arms_cartesian_coordinate[0].y,
            .arm.upper.x = arms_cartesian_coordinate[1].x,
            .arm.upper.y = arms_cartesian_coordinate[1].y,
            .arm.upper.z = upper_arm_z,
            .arm.lower.left = lower_hand.left,
            .arm.lower.middle = lower_hand.middle,
            .arm.lower.right = lower_hand.right,
            .arm.lower.expand = lower_hand.expand,
        }
    };
    if(can_tx(&com))ESP_LOGE(ARM_TAG, "set_arms() failed.");
    
}

//極座標アームの位置を初期化
void homing(){

}

void arms_dump(){
    logi("arm[0](lower): CART(%.3f,%.3f), POR(%.3f,%.3f), \\
          HAND{left %d, middle %d, right %d, expand, %d}\n",
          arms_cartesian_coordinate[0].x,arms_cartesian_coordinate[0].y,to_polar(arms_cartesian_coordinate[0]).r, to_polar(arms_cartesian_coordinate[0]).theta,
          lower_hand.left, lower_hand.middle, lower_hand.right, lower_hand.expand);

    logi("arm[1](upper): CART(%.3f,%.3f), POR(%.3f,%.3f), Z %d\n",
          arms_cartesian_coordinate[1].x,arms_cartesian_coordinate[1].y,to_polar(arms_cartesian_coordinate[1]).r, to_polar(arms_cartesian_coordinate[1]).theta,
          upper_arm_z);
}

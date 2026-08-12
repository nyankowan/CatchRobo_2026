#include "arm.h"
#include <uni.h>

direct_t arms_cartesian_coordinate[ARM_NUM] = {0};
void get_arms_cartesian_coordinate(direct_t *arm[]){
    for(int i = 0; i<ARM_NUM; i++){
        arm[i] = &arms_cartesian_coordinate[i];
    }
}

void coordinate_dump(direct_t *xy){
    logi("X: %.2f, Y: %.2f, R: %.2f, Theta: %.2f\n", xy->x, xy->y, to_polar(*xy).r, to_polar(*xy).theta);
}

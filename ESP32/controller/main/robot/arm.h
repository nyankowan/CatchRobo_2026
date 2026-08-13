#ifndef ARM_H
#define ARM_H
#define ARM_NUM 2
#include "coordinate.h"
typedef struct{
    unsigned int left   : 1;
    unsigned int middle : 1;
    unsigned int right  : 1;
    unsigned int expand : 1;
    unsigned int        : 4;
}lower_hand_t;
void get_pointer_arms_cartesian_coordinate(direct_t *arm[]);
void get_pointer_lower_hand(lower_hand_t **h);
void get_pointer_upper_arm_z(unsigned int **z);
void arms_dump();
void set_arms();
void homing();
#endif //ARM_H
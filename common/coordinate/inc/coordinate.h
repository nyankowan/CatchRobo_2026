#ifndef COORDINATE_H
#define COORDINATE_H

/*ARM SETTINGS*/
#define ARM_NUM 2
#define UPPER_ARM_R_RANGE 700.0 //ToDo: 上側アームのアーム長可動域(mm)
#define LOWER_ARM_R_RANGE 830.0 //ToDo: 下側アームのアーム長可動域(mm)
#define LOWER_ARM_R_MIN 200.0   //ToDo: アーム長を一番短くしたときのR(mm)を測る
#define UPPER_ARM_R_MIN 200.0   //ToDo: アーム長を一番短くしたときのR(mm)を測る
#define LOWER_ARM_HOME_COORDINATE {\
  .x = LOWER_ARM_R_MIN,\
  .y = 0,\
}
#define UPPER_ARM_HOME_COORDINATE {\
  .x = UPPER_ARM_R_MIN,\
  .y = 0,\
}\
/*ARM SETTINGS*/

struct direct_s {
    double x;
    double y;
} typedef direct_t;
struct polar_s {
    double r;
    double theta;//rad
} typedef polar_t;

polar_t to_polar(direct_t d);
#endif //COORDINATE_H
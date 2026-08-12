#ifndef COORDINATE_H
#define COORDINATE_H

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
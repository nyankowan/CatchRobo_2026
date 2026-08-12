#include "coordinate.h"
#include <math.h>   //コンパイルするとき-lmでリンクする必要がある

polar_t to_polar(direct_t d){
    polar_t p;
    p.r = sqrt(d.x*d.x + d.y*d.y);
    p.theta = atan2(d.y, d.x);
    if(p.theta<0)p.theta+=2*M_PI; // atan2の返り値は-π～πの範囲なので、0～2πに変換
    return p;
}
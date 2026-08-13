#include "procon_data.h"
#include <uni.h>

const mypad_t EMPTY_MYPAD = {
    .A = 0,
    .B = 0,
    .X = 0,
    .Y = 0,
    .UP = 0,
    .DOWN = 0,
    .LEFT = 0,
    .RIGHT = 0,
    .L = 0,
    .R = 0,
    .ZL = 0,
    .ZR = 0,
    .TL = 0,
    .TR = 0,
    .MINUS = 0,
    .PLUS = 0,
    .HOME = 0,
    .CAPTURE = 0,
    .LX = 0,
    .LY = 0,
    .RX = 0,
    .RY = 0,
    .battery_level = 0,
    .connected = 0
};

void mypad_dump(mypad_t* pad) {
    logi("A:  %d, B:    %d, X:    %d, Y:     %d\n\\
          UP: %d, DOWN: %d, LEFT: %d, RIGHT: %d\n\\
          L:  %d, R:    %d, ZL:   %d, ZR:    %d, TL: %d, TR: %d\n\\
          MINUS: %d, PLUS: %d, HOME: %d, CAPTURE: %d\n\\
          LX: %d, LY:   %d, RX:   %d, RY:    %d\n\\
          battery: %3d, connected: %d\n",
        pad->A, pad->B, pad->X, pad->Y, 
        pad->UP, pad->DOWN, pad->LEFT, pad->RIGHT, 
        pad->L, pad->R, pad->ZL, pad->ZR, pad->TL, pad->TR, 
        pad->MINUS, pad->PLUS, pad->HOME, pad->CAPTURE, 
        pad->LX, pad->LY, pad->RX, pad->RY,
        pad->battery_level, pad->connected);
}
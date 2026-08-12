#include "procon_data.h"
#include <uni.h>

const mypad_t EMPTY_MYPAD = {
    .A = false,
    .B = false,
    .X = false,
    .Y = false,
    .UP = false,
    .DOWN = false,
    .LEFT = false,
    .RIGHT = false,
    .L = false,
    .R = false,
    .ZL = false,
    .ZR = false,
    .TL = false,
    .TR = false,
    .MINUS = false,
    .PLUS = false,
    .HOME = false,
    .CAPTURE = false,
    .LX = 0,
    .LY = 0,
    .RX = 0,
    .RY = 0,
    .battery_level = 0,
    .connected = false
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
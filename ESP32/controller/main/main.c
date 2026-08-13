// SPDX-License-Identifier: Apache-2.0
// Copyright 2019 Ricardo Quesada
// http://retro.moe/unijoysticle2

#include <stdlib.h>

#include <btstack_port_esp32.h>
#include <btstack_run_loop.h>

#include <btstack_stdio_esp32.h>
#include <hci_dump.h>
#include <hci_dump_embedded_stdout.h>
#include <uni.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sdkconfig.h"

#include "gpio.h"

#include "can.h"
#include "procon_data.h"
#include "coordinate.h"
#include "arm.h"

// Sanity check
#ifndef CONFIG_BLUEPAD32_PLATFORM_CUSTOM
#error "Must use BLUEPAD32_PLATFORM_CUSTOM"
#endif

// Defined in my_platform.c
struct uni_platform* get_my_platform(void);

void main_task(void* arg);
void dump_task(void* arg);

int app_main(void) {
    // hci_dump_open(NULL, HCI_DUMP_STDOUT);

// Don't use BTstack buffered UART. It conflicts with the console.
#ifdef CONFIG_ESP_CONSOLE_UART
#ifndef CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE
    btstack_stdio_init();
#endif  // CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE
#endif  // CONFIG_ESP_CONSOLE_UART



    // Configure BTstack for ESP32 VHCI Controller
    btstack_init();

    // Must be called before uni_init()
    uni_platform_set_custom(get_my_platform());

    // Init Bluepad32.
    uni_init(0 /* argc */, NULL /* argv */);

    can_init_and_start(CAN_TX_GPIO, CAN_RX_GPIO);
    xTaskCreatePinnedToCore(main_task, "main_task", 4096, NULL, 1, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(dump_task, "dump_task", 4096, NULL, 1, NULL, APP_CPU_NUM);
    // Does not return.
    btstack_run_loop_execute();

    return 0;
}

#define MAIN_TASK_LOOP_MS 30
#define DUMP_TASK_LOOP_MS 2000
#define GET_MYPAD_WAIT_MS 2000

void dump_task(void* arg){
    mypad_t mp[MAX_MYPAD] = {0};
    while(1){
        get_mypad(mp);
        logi("mypad[0]: ");
        mypad_dump(&mp[0]);
        logi("\n");
        logi("mypad[1]: ");
        mypad_dump(&mp[1]);
        logi("\n");
        arms_dump();
        vTaskDelay(pdMS_TO_TICKS(DUMP_TASK_LOOP_MS));
    }
}
#define TOGGLE(button, num) if(button==0){button=num;}else{button=0;}
void main_task(void* arg){
    mypad_t mypad[MAX_MYPAD] = {0}, prev_mypad[MAX_MYPAD] = {0};
    direct_t *arm_coordinate[ARM_NUM];
    lower_hand_t *lower_hand;
    unsigned int *upper_arm_z;
    get_mypad(mypad);
    get_pointer_arms_cartesian_coordinate(arm_coordinate);
    get_pointer_lower_hand(&lower_hand);
    get_pointer_upper_arm_z(&upper_arm_z);
    while(1){
        memcpy(prev_mypad,mypad,sizeof(mypad));
        get_mypad(mypad);
        if(mypad[0].LEFT) {arm_coordinate[0]->x -= 0.1;}
        if(mypad[0].RIGHT){arm_coordinate[0]->x += 0.1;}
        if(mypad[0].UP)   {arm_coordinate[0]->y += 0.1;}
        if(mypad[0].DOWN) {arm_coordinate[0]->y -= 0.1;}
        if(PRESSED(mypad[0].A, prev_mypad[0].A)){TOGGLE(lower_hand->right,1);}
        if(PRESSED(mypad[0].X, prev_mypad[0].X)){TOGGLE(lower_hand->middle,1);}
        if(PRESSED(mypad[0].Y, prev_mypad[0].Y)){TOGGLE(lower_hand->left,1);}
        if(PRESSED(mypad[0].B, prev_mypad[0].B)){TOGGLE(lower_hand->expand,1);}
        
        

        if(mypad[1].LEFT) {arm_coordinate[1]->x -= 0.1;}
        if(mypad[1].RIGHT){arm_coordinate[1]->x += 0.1;}
        if(mypad[1].UP)   {arm_coordinate[1]->y += 0.1;}
        if(mypad[1].DOWN) {arm_coordinate[1]->y -= 0.1;}
        if(PRESSED(mypad[1].A, prev_mypad[1].A)){TOGGLE(*upper_arm_z,10);}
        //set_arms();
        vTaskDelay(pdMS_TO_TICKS(MAIN_TASK_LOOP_MS));
    }
}

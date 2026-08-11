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

// Sanity check
#ifndef CONFIG_BLUEPAD32_PLATFORM_CUSTOM
#error "Must use BLUEPAD32_PLATFORM_CUSTOM"
#endif

// Defined in my_platform.c
struct uni_platform* get_my_platform(void);

void main_task(void* arg);

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
    // Does not return.
    btstack_run_loop_execute();

    return 0;
}

#define MAIN_TASK_LOOP_MS 10
#define MYPAD_CONNCTION_WAIT_MS 2000
mypad_t *mypad[MAX_MYPAD] = {0};

void dump(){
    logi("mypad[0]: ");
    mypad_dump(mypad[0]);
    logi("\n");
    logi("mypad[1]: ");
    mypad_dump(mypad[1]);
    logi("\n");
}

void main_task(void* arg){
    while(1){
        get_mypad(mypad);
        while(mypad[0]==NULL || mypad[1]==NULL){
            logi("Can't get mypad.\n");
            get_mypad(mypad);
            vTaskDelay(pdMS_TO_TICKS(MYPAD_CONNCTION_WAIT_MS));
        }
        if(mypad[0]->A){
            printf("player 1 press A\n");
        }
        if(mypad[1]->A){
            printf("player 2 press A\n");
        }
        vTaskDelay(pdMS_TO_TICKS(MAIN_TASK_LOOP_MS));
        dump();
    }
}

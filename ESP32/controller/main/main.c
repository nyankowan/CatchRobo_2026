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
#include "arm_command.h"
#include "led.h"
#include "micon_connection.h"

// Sanity check
#ifndef CONFIG_BLUEPAD32_PLATFORM_CUSTOM
#error "Must use BLUEPAD32_PLATFORM_CUSTOM"
#endif

// Defined in my_platform.c
struct uni_platform* get_my_platform(void);

void main_task(void* arg);
void dump_task(void* arg);

void btstack_run_loop_task(void* arg) {
    (void)arg;
    // execute()を呼ぶのと同じタスクでinitする（ここがBTstack側のタスクハンドルとして記憶される）
    // Configure BTstack for ESP32 VHCI Controller
    btstack_init();

    // Must be called before uni_init()
    uni_platform_set_custom(get_my_platform());

    // Init Bluepad32.
    uni_init(0 /* argc */, NULL /* argv */);
    // Does not return.
    btstack_run_loop_execute();
}

int app_main(void) {
    // hci_dump_open(NULL, HCI_DUMP_STDOUT);

// Don't use BTstack buffered UART. It conflicts with the console.
#ifdef CONFIG_ESP_CONSOLE_UART
#ifndef CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE
    btstack_stdio_init();
#endif  // CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE
#endif  // CONFIG_ESP_CONSOLE_UART





    led_init();
    arms_init();
    micon_connection_init();

    can_init_and_start(CAN_TX_GPIO, CAN_RX_GPIO);
    xTaskCreatePinnedToCore(main_task, "main_task", 4000, NULL, 1, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(dump_task, "dump_task", 5000, NULL, 1, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(btstack_run_loop_task, "btstack_run_loop_task", 6000, NULL, 1, NULL, APP_CPU_NUM);

    return 0;
}

#define MAIN_TASK_LOOP_MS 130
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
        micon_connection_dump();
        logi("\n");
        vTaskDelay(pdMS_TO_TICKS(DUMP_TASK_LOOP_MS));
    }
}

void main_task(void* arg){
    mypad_t mypad[MAX_MYPAD] = {0}, prev_mypad[MAX_MYPAD] = {0};
    #define lower_mypad mypad[0]
    #define upper_mypad mypad[1]
    #define lower_prev_mypad prev_mypad[0]
    #define upper_prev_mypad prev_mypad[1]
    bool status_led_state = false; // STATUS_LED点滅(生存確認)用
    get_mypad(mypad);
    while(1){
        memcpy(prev_mypad,mypad,sizeof(mypad));
        get_mypad(mypad);

        if(PRESSED(lower_mypad.HOME, lower_prev_mypad.HOME))lower_arm_homing();
        if(PRESSED(upper_mypad.HOME, upper_prev_mypad.HOME))upper_arm_homing();
        

        led_set_level(CONTROLLER_1_LED_GPIO, mypad[0].connected);
        led_set_level(CONTROLLER_2_LED_GPIO, mypad[1].connected);
        led_set_level(ROBOMAS_CONTROLLER_STATUS_LED_GPIO, get_connection(MICON_TYPE_ROBOMAS_CONTROLLER));
        led_set_level(LOWER_ARM_STATUS_LED_GPIO, get_connection(MICON_TYPE_LOWER_ARM));
        led_set_level(UPPER_ARM_STATUS_LED_GPIO, get_connection(MICON_TYPE_UPPER_ARM));
        led_set_level(CAN_STATUS_LED_GPIO, can_is_running());

        status_led_state = !status_led_state; // main_taskが生きている限り点滅し続ける
        led_set_level(STATUS_LED_GPIO, status_led_state);
        
        lower_arm_move(
            lower_mypad.RIGHT - lower_mypad.LEFT + lower_mypad.LX,
            lower_mypad.UP -    lower_mypad.DOWN - lower_mypad.LY,
            PRESSED(lower_mypad.Y, lower_prev_mypad.Y),
            PRESSED(lower_mypad.X, lower_prev_mypad.X),
            PRESSED(lower_mypad.A, lower_prev_mypad.A),
            PRESSED(lower_mypad.B, lower_prev_mypad.B),
            PRESSED(lower_mypad.L, lower_prev_mypad.L) || PRESSED(lower_mypad.R, lower_prev_mypad.R)
        );
        send_lower_arm();

        upper_arm_move(
            upper_mypad.RIGHT - upper_mypad.LEFT + upper_mypad.LX,
            upper_mypad.UP -    upper_mypad.DOWN - upper_mypad.LY,
            upper_mypad.A -     upper_mypad.B
        );
        send_upper_arm();

        micon_connection_update();
        arms_update();
        vTaskDelay(pdMS_TO_TICKS(MAIN_TASK_LOOP_MS));
    }
}

#include "micon_connection.h"
#include "can.h"
#include <uni.h>
#include <esp_log.h>

#include "freertos/FreeRTOS.h"

#define MICON_CONNECTION_TAG "micon_connection"

static TickType_t robomas_controller_last_update;
static TickType_t upper_arm_last_update;
static TickType_t lower_arm_last_update;

static bool robomas_controller_connected;
static bool upper_arm_connected;
static bool lower_arm_connected;

static void robomas_controller_heartbeat_notify(const can_data_t *data);
static void lower_arm_heartbeat_notify(const can_data_t *data);
static void upper_arm_heartbeat_notify(const can_data_t *data);

void micon_connection_init(){
    can_register_rx_callback(CAN_ID_ROBOMAS_CONTROLLER_HEARTBEAT, robomas_controller_heartbeat_notify);
    can_register_rx_callback(CAN_ID_LOWER_ARM_HEARTBEAT, lower_arm_heartbeat_notify);
    can_register_rx_callback(CAN_ID_UPPER_ARM_HEARTBEAT, upper_arm_heartbeat_notify);
}

static void robomas_controller_heartbeat_notify(const can_data_t *data){
    robomas_controller_last_update = xTaskGetTickCount();
    ESP_LOGV(MICON_CONNECTION_TAG, "robomas_controller heart beat!!\n");
}

static void lower_arm_heartbeat_notify(const can_data_t *data){
    lower_arm_last_update = xTaskGetTickCount();
    ESP_LOGV(MICON_CONNECTION_TAG, "lower_arm heart beat!!\n");
}

static void upper_arm_heartbeat_notify(const can_data_t *data){
    upper_arm_last_update = xTaskGetTickCount();
    ESP_LOGV(MICON_CONNECTION_TAG, "upper_arm heart beat!!\n");
}

bool get_connection(micon_type_t m){
    switch (m)
    {
    case MICON_TYPE_ROBOMAS_CONTROLLER:
        return robomas_controller_connected;
    case MICON_TYPE_UPPER_ARM:
        return upper_arm_connected;
    case MICON_TYPE_LOWER_ARM:
        return lower_arm_connected;
    default:
        return false;
    }
}


/*
* 周期的に呼び出す
*/
void micon_connection_update(){
    TickType_t now = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(HEARTBEAT_TIMEOUT_MS);

    robomas_controller_connected = (now - robomas_controller_last_update) < timeout_ticks;
    upper_arm_connected          = (now - upper_arm_last_update)          < timeout_ticks;
    lower_arm_connected          = (now - lower_arm_last_update)          < timeout_ticks;
}

void micon_connection_dump(){
    logi("ROBOMAS_CONTROLLER :%d\n", get_connection(MICON_TYPE_ROBOMAS_CONTROLLER));
    logi("UPPER_ARM          :%d\n", get_connection(MICON_TYPE_UPPER_ARM));
    logi("LOWER_ARM          :%d\n", get_connection(MICON_TYPE_LOWER_ARM));
}
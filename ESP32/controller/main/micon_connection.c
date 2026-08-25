#include "micon_connection.h"
#include "can.h"
#include <uni.h>
#include <esp_log.h>

#include "freertos/FreeRTOS.h"

#define MICON_CONNECTION_TAG "micon_connection"

TickType_t robomas_controller_last_update;
bool robomas_controller_connected;

static void robomas_controller_heartbeat_notify(const can_data_t *data);
//static void lower_arm_heartbeat_notify(const can_data_t *data); ToDo
//static void upper_arm_heartbeat_notify(const can_data_t *data); ToDo

void micon_connection_init(){
    can_register_rx_callback(CAN_ID_ROBOMAS_CONTROLLER_HEARTBEAT, robomas_controller_heartbeat_notify);

}

static void robomas_controller_heartbeat_notify(const can_data_t *data){
    robomas_controller_last_update = xTaskGetTickCount();
    ESP_LOGV(MICON_CONNECTION_TAG, "robomas_controller heart beat!!\n");
}

bool get_connection(micon_type_t m){
    switch (m)
    {
    case MICON_TYPE_ROBOMAS_CONTROLLER:
        return robomas_controller_connected;
    
    default:
        return false;
    }
}


/*
* 周期的に呼び出す
*/
void micon_connection_update(){
    robomas_controller_connected = pdMS_TO_TICKS(xTaskGetTickCount() - robomas_controller_last_update) < HEARTBEAT_TIMEOUT_MS;
}

void micon_connection_dump(){
    logi("ROBOMAS_CONTROLLER :%d\n", get_connection(MICON_TYPE_ROBOMAS_CONTROLLER));
}
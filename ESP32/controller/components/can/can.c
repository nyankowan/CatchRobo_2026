#include "can.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define CAN_TAG "CAN"
typedef struct{
    can_rx_callback_t callback;
    can_id_t id;
}can_rx_callback_entry_t;

static can_rx_callback_entry_t* find_callback(can_id_t id);

static bool already_can_init_and_start = false;
static int callback_entry_num = 0;
static can_rx_callback_entry_t can_rx_callback_entry_register[CAN_ID_NUM_ITEMS] = {0};

esp_err_t can_tx(can_command_data_t *com_data){
    if(com_data == NULL){
        ESP_LOGE(CAN_TAG, "Argument is invalid");
        return ESP_ERR_INVALID_ARG;
    }
    twai_message_t msg = {
        .data_length_code = can_protocol_get_dlc(com_data->id),
        .identifier = com_data->id,
    };
    memcpy(msg.data, com_data->data.raw, sizeof(msg.data));
    switch(twai_transmit(&msg, 0)){
        case ESP_OK:
            return ESP_OK;
        case ESP_ERR_INVALID_ARG:
            ESP_LOGE(CAN_TAG, "Arguments is invalid.");
            return ESP_ERR_INVALID_ARG;
        case ESP_ERR_TIMEOUT:
            ESP_LOGE(CAN_TAG, "Timed out waiting for space on TX queue.");
            return ESP_ERR_TIMEOUT;
        case ESP_ERR_INVALID_STATE:
            ESP_LOGE(CAN_TAG, "Driver is not in running state, or is not installed.");
            return ESP_ERR_INVALID_STATE;
        case ESP_FAIL:
            ESP_LOGE(CAN_TAG, "TX queue is disabled and another message is currently transmitting.");
            return ESP_FAIL;
        case ESP_ERR_NOT_SUPPORTED:
            ESP_LOGE(CAN_TAG, "Listen Only Mode does not support transmissions.");
            return ESP_ERR_NOT_SUPPORTED;
    }
    return ESP_FAIL;
}

esp_err_t can_init_and_start(gpio_num_t tx, gpio_num_t rx){
    if(already_can_init_and_start){
        ESP_LOGE(CAN_TAG, "CAN has alerady init and start.");
        return ESP_FAIL;
    }
      
    // TWAI初期化  
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx, rx, TWAI_MODE_NORMAL);  
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS(); 
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    esp_err_t e;
    e = twai_driver_install(&g_config, &t_config, &f_config);
    switch (e) {
        case ESP_ERR_NO_MEM:
            ESP_LOGE(CAN_TAG, "Insufficient memory.");
            return e;
        case ESP_ERR_INVALID_STATE:
            ESP_LOGE(CAN_TAG, "CAN driver is already installed.");
            return e;
        default:
            ESP_LOGV(CAN_TAG, "CAN driver installed successfully.");
    }
        
    e = twai_start();
    if(e){
        ESP_LOGE(CAN_TAG, "CAN is not stopped.");
        return e;
    }
    already_can_init_and_start = true;
    xTaskCreatePinnedToCore(can_error_handling_task, "can_error_handling_task", 2048, NULL, 5, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(can_rx_task, "can_rx_task", 2048, NULL, 4, NULL, APP_CPU_NUM);
    
    return ESP_OK;
    
}

can_rx_callback_entry_t* find_callback(can_id_t id){
    for(int i=0; i<CAN_ID_NUM_ITEMS; i++){
        if(can_rx_callback_entry_register[i].id == id)return &can_rx_callback_entry_register[i];
    }
    return NULL;
}
void can_register_rx_callback(can_id_t id, can_rx_callback_t callback){
    can_rx_callback_entry_register[callback_entry_num].id = id;
    can_rx_callback_entry_register[callback_entry_num].callback = callback;
    callback_entry_num++;
}

void can_rx_task(void *arg){
    twai_message_t msg;

    while (1) {
        if (twai_receive(&msg, portMAX_DELAY) == ESP_OK) {
            // 登録された対応するcallbackを探す
            can_rx_callback_entry_t* entry = find_callback(msg.identifier);
            if (entry != NULL) {
                can_data_t data;
                memcpy(data.raw,msg.data,sizeof(data.raw));
                entry->callback(&data);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void can_error_handling_task(void *arg)
{
    twai_status_info_t s;
    while (1) {
        if(twai_get_status_info(&s)){
            ESP_LOGE(CAN_TAG, "CAN is not installed.");
            vTaskDelete(NULL);
        }
        switch (s.state) {
            case TWAI_STATE_BUS_OFF:
                ESP_LOGE(CAN_TAG, "BUS OFF");
                twai_initiate_recovery();
                ESP_LOGE(CAN_TAG, "CAN recovery.");
                vTaskDelay(pdMS_TO_TICKS(500));
                break;
            
            case TWAI_STATE_STOPPED:
                ESP_LOGE(CAN_TAG, "CAN STOPPED");
                twai_start();
                ESP_LOGE(CAN_TAG, "CAN start.");
                break;
            
            case TWAI_STATE_RECOVERING:
                ESP_LOGE(CAN_TAG, "CAN RECOVERING");
                vTaskDelay(pdMS_TO_TICKS(500));
                break;
            default:
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}



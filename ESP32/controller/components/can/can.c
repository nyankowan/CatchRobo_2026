#include "can.h"
#include "can_protcol.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
static bool ALREADY_CAN_INIT_AND_START = false;
#define CAN_TAG "CAN"

esp_err_t can_tx(can_command_data_t *com){
    if(com == NULL){
        ESP_LOGE(CAN_TAG, "Argument is invalid");
        return ESP_ERR_INVALID_ARG;
    }
    twai_message_t msg = {
        .data_length_code = 8,
        .identifier = com->id,
    };
    memcpy(msg.data, com->data, sizeof(msg.data));
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
    if(ALREADY_CAN_INIT_AND_START){
        ESP_LOGE(CAN_TAG, "CAN has alerady init and start.");
        return ESP_FAIL;
    }
      
    // TWAI初期化  
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx, rx, TWAI_MODE_NORMAL);  
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();  
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
    ALREADY_CAN_INIT_AND_START = true;
    xTaskCreatePinnedToCore(can_error_handling_task, "can_error_handling_task", 1024, NULL, 5, NULL, APP_CPU_NUM);
    return ESP_OK;
    
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



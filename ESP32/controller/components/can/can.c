#include "can.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define CAN_TAG "CAN"

#define DEFAULT_TX_GPIO 21
#define DEFAULT_RX_GPIO 22

#define CAN_ERROR_HANDLING_TASK_LOOP_MS 10
typedef struct{
    can_rx_callback_t callback;
    can_id_t id;
}can_rx_callback_entry_t;

static can_rx_callback_entry_t* find_callback(can_id_t id);

static bool already_can_init_and_start = false;
static int callback_entry_num = 0;
static can_rx_callback_entry_t can_rx_callback_entry_register[CAN_ID_NUM_ITEMS] = {0};
/*prevent spaming error message*/
static esp_err_t can_tx_error = ESP_OK;

static volatile bool can_running = false;

bool can_is_running(){
    return can_running;
}

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
    esp_err_t prev_can_tx_error = can_tx_error;
    can_tx_error = twai_transmit(&msg, 0);
    if(can_tx_error == prev_can_tx_error)return can_tx_error;
    switch(can_tx_error){
        case ESP_OK:
            break;
        case ESP_ERR_INVALID_ARG:
            ESP_LOGE(CAN_TAG, "Arguments is invalid.");
            break;
        case ESP_ERR_TIMEOUT:
            //Prevent spaming error message.
            ESP_LOGW(CAN_TAG, "Timed out waiting for space on TX queue.");
            break;
        case ESP_ERR_INVALID_STATE:
            ESP_LOGW(CAN_TAG, "Driver is not in running state, or is not installed.");
            break;
        case ESP_FAIL:
            ESP_LOGW(CAN_TAG, "TX queue is disabled and another message is currently transmitting.");
            break;
        case ESP_ERR_NOT_SUPPORTED:
            ESP_LOGE(CAN_TAG, "Listen Only Mode does not support transmissions.");
            break;
    }
    return can_tx_error;
}

esp_err_t can_install_and_start(gpio_num_t tx, gpio_num_t rx){
    // TWAI初期化  
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx, rx, TWAI_MODE_NORMAL);  
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS(); 
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    esp_err_t e;
    ESP_LOGI(CAN_TAG, "CAN driver install.");
    e = twai_driver_install(&g_config, &t_config, &f_config);
    switch (e) {
        case ESP_ERR_NO_MEM:
            ESP_LOGE(CAN_TAG, "Insufficient memory.");
            return e;
        case ESP_ERR_INVALID_STATE:
            ESP_LOGW(CAN_TAG, "CAN driver is already installed.");
            return e;
        default:
            ESP_LOGI(CAN_TAG, "CAN driver installed successfully.");
    }
    ESP_LOGI(CAN_TAG, "CAN start.");    
    e = twai_start();
    if(e){
        ESP_LOGW(CAN_TAG, "CAN is not stopped.");
        return e;
    }else{
        ESP_LOGI(CAN_TAG, "CAN started successfully.");
    }
    return ESP_OK;
}

esp_err_t can_init_and_start(gpio_num_t tx, gpio_num_t rx){
    if(already_can_init_and_start){
        ESP_LOGW(CAN_TAG, "CAN has alerady init and start.");
        return ESP_FAIL;
    }
    esp_err_t e = can_install_and_start(tx,rx);
    if(e)return e;
    
    xTaskCreatePinnedToCore(can_error_handling_task, "can_error_handling_task", 3000, NULL, 5, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(can_rx_task, "can_rx_task", 3000, NULL, 4, NULL, APP_CPU_NUM);
    already_can_init_and_start = true;
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
    }
}

void can_error_handling_task(void *arg)
{
    twai_status_info_t s;
    while (1) {
        if(twai_get_status_info(&s)){
            ESP_LOGW(CAN_TAG, "CAN is not installed.");
            can_running = false;
            ESP_LOGI(CAN_TAG, "CAN install default gpio: tx = %2d, rx = %2d", DEFAULT_TX_GPIO, DEFAULT_RX_GPIO);
            if(can_install_and_start(DEFAULT_TX_GPIO, DEFAULT_RX_GPIO)){
                ESP_LOGE(CAN_TAG, "CAN install default failed. Delete \"can_error_handling_task\".");
                vTaskDelete(NULL);
                vTaskDelay(pdMS_TO_TICKS(CAN_ERROR_HANDLING_TASK_LOOP_MS));
                continue;
            }
        }
        can_running = (s.state == TWAI_STATE_RUNNING);
        switch (s.state) {
            case TWAI_STATE_BUS_OFF:
                ESP_LOGW(CAN_TAG, "BUS OFF");
                twai_initiate_recovery();
                ESP_LOGW(CAN_TAG, "CAN recovery.");
                vTaskDelay(pdMS_TO_TICKS(500));
                break;
            
            case TWAI_STATE_STOPPED:
                ESP_LOGW(CAN_TAG, "CAN STOPPED");
                twai_start();
                ESP_LOGE(CAN_TAG, "CAN start.");
                break;
            
            case TWAI_STATE_RECOVERING:
                ESP_LOGW(CAN_TAG, "CAN RECOVERING");
                vTaskDelay(pdMS_TO_TICKS(500));
                break;
            default:
        }
        vTaskDelay(pdMS_TO_TICKS(CAN_ERROR_HANDLING_TASK_LOOP_MS));
    }
}



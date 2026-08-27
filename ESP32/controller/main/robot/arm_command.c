#include "arm_command.h"
#include "arm.h"
#include "can.h"
#include <uni.h>
#include <esp_log.h>
#include <stdint.h>
#include <math.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ARM_TAG "ARM"

#define TOGGLE(button, num) \
    if ((button) == 0) {    \
        (button) = (num);   \
    } else {                 \
        (button) = 0;       \
    }


static void lower_arm_homing_done_notify(const can_data_t *data);
static void upper_arm_homing_done_notify(const can_data_t *data);
static void error_code_notify(const can_data_t *data);

/* ToDo 
static void lower_arm_homing_ack_notify(const can_data_t *data);
static void upper_arm_homing_ack_notify(const can_data_t *data);
*/


static lower_arm_t lower_arm = {0};
static upper_arm_t upper_arm = {0};

static uint16_t upper_arm_homing_sequence = 0;
static uint16_t lower_arm_homing_sequence = 0;

static bool arms_init_already_done = false;

static bool lower_arm_homing_in_progress = false;
static bool upper_arm_homing_in_progress = false;

/* homing開始時刻 */
static TickType_t lower_arm_homing_start_tick = 0;
static TickType_t upper_arm_homing_start_tick = 0;


// homing中は動かない
void lower_arm_move(int16_t dx,int16_t dy, bool left_toggle, bool middle_toggle, bool right_toggle, bool expand_toggle){
    if(lower_arm_homing_in_progress)return;
    direct_t d = {
        .x = lower_arm.x,
        .y = lower_arm.y, 
    };
    d.x += dx;
    d.y += dy;
    polar_t p = to_polar(d);
    if( LOWER_ARM_R_MIN <= p.r && p.r <= LOWER_ARM_R_MIN + LOWER_ARM_R_RANGE && 
        LOWER_ARM_DEG_MIN <= p.theta / (2*M_PI) * 360 && p.theta / (2*M_PI) * 360 <= LOWER_ARM_DEG_MIN + LOWER_ARM_DEG_RANGE){
        lower_arm.x = d.x;
        lower_arm.y = d.y;
    }

    if(left_toggle)     {TOGGLE(lower_arm.left, 1);}
    if(middle_toggle)   {TOGGLE(lower_arm.middle, 1);}
    if(right_toggle)    {TOGGLE(lower_arm.right, 1);}
    if(expand_toggle)   {TOGGLE(lower_arm.expand, 1);}
}


// homing中は動かない
void upper_arm_move(int16_t dx, int16_t dy, int16_t dz){
    if (upper_arm_homing_in_progress)return;

    direct_t d = {
        .x = upper_arm.x,
        .y = upper_arm.y, 
    };
    d.x += dx;
    d.y += dy;
    polar_t p = to_polar(d);
    if( UPPER_ARM_R_MIN <= p.r && p.r <= UPPER_ARM_R_MIN + UPPER_ARM_R_RANGE && 
        UPPER_ARM_DEG_MIN <= p.theta / (2*M_PI) * 360 && p.theta / (2*M_PI) * 360 <= UPPER_ARM_DEG_MIN + UPPER_ARM_DEG_RANGE){
        upper_arm.x = d.x;
        upper_arm.y = d.y;
    }
    upper_arm.z += dz;
}


esp_err_t send_lower_arm(){
    if(lower_arm_homing_in_progress)return ESP_ERR_NOT_ALLOWED;

    can_command_data_t com = {
        .id = CAN_ID_LOWER_ARM_COMMAND,
        .data.lower_arm = lower_arm,
    };
    esp_err_t err = can_tx(&com);

    if(err != ESP_OK){
        ESP_LOGE(ARM_TAG, "CAN_ID_LOWER_ARM_COMMAND failed.");
    }
    return err;
}


esp_err_t send_upper_arm(){
    if(upper_arm_homing_in_progress)return ESP_ERR_NOT_ALLOWED;

    can_command_data_t com = {
        .id = CAN_ID_UPPER_ARM_COMMAND,
        .data.upper_arm = upper_arm,
    };
    esp_err_t err = can_tx(&com);

    if (err != ESP_OK) {
        ESP_LOGE(ARM_TAG, "CAN_ID_UPPER_ARM_COMMAND failed.");
    }
    return err;
}


/*
 * CAN RX callbackを登録する
 *
 * ACKは使用しない。
 * STM32からHOMING_DONEが返ってきたらhoming終了とする。
 */
esp_err_t arms_init(){
    if(arms_init_already_done)return ESP_ERR_NOT_ALLOWED;

    can_register_rx_callback(
        CAN_ID_LOWER_HOMING_DONE,
        lower_arm_homing_done_notify
    );

    can_register_rx_callback(
        CAN_ID_UPPER_HOMING_DONE,
        upper_arm_homing_done_notify
    );

    can_register_rx_callback(
        CAN_ID_ERROR_CODE,
        error_code_notify
    );

    arms_init_already_done = true;

    return ESP_OK;
}


/*can_tx(&com)
 * Lower Arm Homing開始
 *
 * ESP32 -> STM32
 * CAN_ID_LOWER_HOMING
 */
esp_err_t lower_arm_homing(){
    if(lower_arm_homing_in_progress)return ESP_ERR_NOT_FINISHED;

    can_command_data_t command = {
        .id = CAN_ID_LOWER_HOMING,
        .data.homing_sequence = lower_arm_homing_sequence,
    };

    esp_err_t err = can_tx(&command);

    if (err != ESP_OK) {
        ESP_LOGE(ARM_TAG,"lower arm homing command failed.");
        return err;
    }

    lower_arm_homing_in_progress = true;
    lower_arm_homing_start_tick = xTaskGetTickCount();

    ESP_LOGI(ARM_TAG,"lower arm homing start. sequence=%u",lower_arm_homing_sequence);

    return ESP_OK;
}


/*
 * Upper Arm Homing開始
 *
 * ESP32 -> STM32
 * CAN_ID_UPPER_HOMING
 */
esp_err_t upper_arm_homing(){
    if (upper_arm_homing_in_progress)return ESP_ERR_NOT_FINISHED;

    can_command_data_t command = {
        .id = CAN_ID_UPPER_HOMING,
        .data.homing_sequence = upper_arm_homing_sequence,
    };

    esp_err_t err = can_tx(&command);

    if (err != ESP_OK) {
        ESP_LOGE(ARM_TAG,"upper arm homing command failed.");
        return err;
    }

    upper_arm_homing_in_progress = true;
    upper_arm_homing_start_tick = xTaskGetTickCount();

    ESP_LOGI(ARM_TAG,"upper arm homing start. sequence=%u",upper_arm_homing_sequence);

    return ESP_OK;
}


/*
 * STM32 -> ESP32
 * CAN_ID_LOWER_HOMING_DONE
 *
 * CAN rx taskからcallbackされるので重い処理はしない。
 */
static void lower_arm_homing_done_notify(const can_data_t *data){
    direct_t larm = LOWER_ARM_HOME_COORDINATE;
    lower_arm.x = larm.x;
    lower_arm.y = larm.y;
    if (!lower_arm_homing_in_progress) {
        ESP_LOGW(ARM_TAG,"lower homing DONE received while not homing.");
        return;
    }

    if (data->homing_sequence != lower_arm_homing_sequence) {
        ESP_LOGE(ARM_TAG,"lower homing sequence error: rx=%u expected=%u",
            data->homing_sequence,lower_arm_homing_sequence
        );
        return;
    }

    lower_arm_homing_in_progress = false;

    ESP_LOGI(ARM_TAG, "lower arm homing done. sequence=%u", lower_arm_homing_sequence);

    lower_arm_homing_sequence++;
}


/*
 * STM32 -> ESP32
 * CAN_ID_UPPER_HOMING_DONE
 */
static void upper_arm_homing_done_notify(const can_data_t *data){
    direct_t uarm = UPPER_ARM_HOME_COORDINATE;
    upper_arm.x = uarm.x;
    upper_arm.y = uarm.y;
    if (!upper_arm_homing_in_progress) {
        ESP_LOGW(ARM_TAG, "upper homing DONE received while not homing.");
        return;
    }

    if (data->homing_sequence != upper_arm_homing_sequence) {
        ESP_LOGE(ARM_TAG, "upper homing sequence error: rx=%u expected=%u",
            data->homing_sequence, upper_arm_homing_sequence);
        return;
    }

    upper_arm_homing_in_progress = false;

    ESP_LOGI(ARM_TAG, "upper arm homing done. sequence=%u", upper_arm_homing_sequence);

    upper_arm_homing_sequence++;
}


/*
 * STM32(robomas_controller) -> ESP32
 * CAN_ID_ERROR_CODE
 *
 * ロボマス制御側で検知した異常(フィードバック途絶・ホーミングタイムアウト等)を通知される。
 * CAN rx taskからcallbackされるので重い処理はしない。
 */
static void error_code_notify(const can_data_t *data){
    switch(data->error_code){
    case CAN_ERROR_LOWER_R_LOST_CONTROL:
        ESP_LOGE(ARM_TAG, "[ERROR] lower arm R robomas lost feedback.");
        break;
    case CAN_ERROR_LOWER_DEG_LOST_CONTROL:
        ESP_LOGE(ARM_TAG, "[ERROR] lower arm DEG robomas lost feedback.");
        break;
    case CAN_ERROR_UPPER_R_LOST_CONTROL:
        ESP_LOGE(ARM_TAG, "[ERROR] upper arm R robomas lost feedback.");
        break;
    case CAN_ERROR_UPPER_DEG_LOST_CONTROL:
        ESP_LOGE(ARM_TAG, "[ERROR] upper arm DEG robomas lost feedback.");
        break;
    case CAN_ERROR_LOWER_HOMING_TIMEOUT:
        ESP_LOGE(ARM_TAG, "[ERROR] lower arm homing timed out. re-homing required.");
        lower_arm_homing_in_progress = false;
        break;
    case CAN_ERROR_UPPER_HOMING_TIMEOUT:
        ESP_LOGE(ARM_TAG, "[ERROR] upper arm homing timed out. re-homing required.");
        upper_arm_homing_in_progress = false;
        break;
    case CAN_ERROR_LOWER_HOMING_REJECTED:
        ESP_LOGE(ARM_TAG, "[ERROR] lower arm homing rejected (robomas in ERROR state).");
        lower_arm_homing_in_progress = false;
        break;
    case CAN_ERROR_UPPER_HOMING_REJECTED:
        ESP_LOGE(ARM_TAG, "[ERROR] upper arm homing rejected (robomas in ERROR state).");
        upper_arm_homing_in_progress = false;
        break;
    default:
        ESP_LOGE(ARM_TAG, "[ERROR] unknown error_code=0x%02x", data->error_code);
        break;
    }
}


/*
 * 定期的に呼ぶ
 *
 * 例:
 * control taskなどから10～100ms周期程度で呼ぶ。
 *
 * HOMING_TIMEOUT_MS以内にDONEが来なければ
 * homing失敗としてin_progressを解除する。
 */
void arms_update(){
    TickType_t now = xTaskGetTickCount();

    /* Lower Arm */
    if (lower_arm_homing_in_progress) {
        if ((now - lower_arm_homing_start_tick) >= pdMS_TO_TICKS(HOMING_LOWER_ARM_TIMEOUT_MS)) {
            ESP_LOGE(ARM_TAG, "lower arm homing timeout. sequence=%u", lower_arm_homing_sequence);

            lower_arm_homing_in_progress = false;
        }
    }

    /* Upper Arm */
    if (upper_arm_homing_in_progress) {
        if ((now - upper_arm_homing_start_tick) >= pdMS_TO_TICKS(HOMING_UPPER_ARM_TIMEOUT_MS)) {
            ESP_LOGE(ARM_TAG, "upper arm homing timeout. sequence=%u", upper_arm_homing_sequence);

            upper_arm_homing_in_progress = false;
        }
    }
}


void lower_arm_dump(){
    polar_t pol = to_polar(
        (direct_t){
            .x = lower_arm.x,
            .y = lower_arm.y
        }
    );

    logi(
        "lower_arm: CART(%4dmm,%4dmm), POR(%4.2fmm,%3.2f°), "
        "HAND{left %1d, middle %1d, right %d, expand %1d}\n",
        lower_arm.x,
        lower_arm.y,
        pol.r,
        pol.theta / (2 * M_PI) * 360,
        lower_arm.left,
        lower_arm.middle,
        lower_arm.right,
        lower_arm.expand
    );
}


void upper_arm_dump(){
    polar_t pol = to_polar(
        (direct_t){
            .x = upper_arm.x,
            .y = upper_arm.y
        }
    );

    logi(
        "upper_arm: CART(%4dmm,%4dmm), POR(%4.2f,%3.2f°), Z %d\n",
        upper_arm.x,
        upper_arm.y,
        pol.r,
        pol.theta / (2 * M_PI) * 360,
        upper_arm.z
    );
}

void arms_dump(){
    lower_arm_dump();
    upper_arm_dump();
}
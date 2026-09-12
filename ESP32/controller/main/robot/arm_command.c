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

/*
 * Left/Middle/Rightハンドのシングルクリック/ダブルクリック判定用の状態。
 * シングルクリック : HAND_STATE_CATCH <-> HAND_STATE_HOLD をトグル(RELEASEからはCATCHへ)
 * ダブルクリック   : HAND_STATE_RELEASE <-> HAND_STATE_HOLD をトグル(CATCHからはRELEASEへ)
 *
 * 押下エッジが来た時点ではシングルクリックかダブルクリックか確定しないため，
 * DOUBLE_CLICK_WINDOW_MS以内に次の押下が来るかどうかで判定する(来なければタイムアウトで
 * シングルクリックとして確定する)。
 */
typedef struct{
    bool pending_single;
    TickType_t press_tick;
}hand_click_state_t;

static hand_click_state_t left_click_state = {0};
static hand_click_state_t middle_click_state = {0};
static hand_click_state_t right_click_state = {0};

#define DOUBLE_CLICK_WINDOW_MS 300

// シャフト角度の微調整(shaft_fine)の可動域(±度)
#define LOWER_ARM_SHAFT_FINE_MAX_DEG 45
#define UPPER_ARM_SHAFT_FINE_MAX_DEG 45

static void lower_arm_homing_done_notify(const can_data_t *data);
static void upper_arm_homing_done_notify(const can_data_t *data);
static void error_code_notify(const can_data_t *data);
static void lower_arm_homing_ack_notify(const can_data_t *data);
static void upper_arm_homing_ack_notify(const can_data_t *data);


static lower_arm_t lower_arm = {0};
static upper_arm_t upper_arm = {0};

static can_sequence_t upper_arm_homing_sequence = 0;
static can_sequence_t lower_arm_homing_sequence = 0;

static bool arms_init_already_done = false;

static bool lower_arm_homing_in_progress = false;
static bool upper_arm_homing_in_progress = false;

/* HOMING要求送信後，HOMING_ACKを受信できたか */
static bool lower_arm_homing_ack_received = false;
static bool upper_arm_homing_ack_received = false;

/* homing開始時刻 */
static TickType_t lower_arm_homing_start_tick = 0;
static TickType_t upper_arm_homing_start_tick = 0;

/* HOMING要求を最後に(再)送信した時刻。ACK未受信ならこれを基準に再送する */
static TickType_t lower_arm_homing_request_sent_tick = 0;
static TickType_t upper_arm_homing_request_sent_tick = 0;

/*spaming 防止用*/
static TickType_t send_lower_arm_error_loged_tick = 0;
static TickType_t send_upper_arm_error_loged_tick = 0;
static esp_err_t send_lower_arm_error = ESP_OK;
static esp_err_t send_upper_arm_error = ESP_OK;
#define PREVENT_SPAMING_MS 2000 //前回と同じエラー内容なら，この間隔で表示する．異なるエラーなら表示．

#define HOMING_REQUEST_RETRY_MS 100 //ACKが届かない場合，この間隔でHOMING要求を再送する


/*
 * 1本のハンド(left/middle/right)のシングルクリック/ダブルクリックを判定し，状態を更新する。
 *
 * シングルクリック : HAND_STATE_CATCH <-> HAND_STATE_HOLD をトグル(RELEASEからはCATCHへ)
 * ダブルクリック   : HAND_STATE_RELEASE <-> HAND_STATE_HOLD をトグル(CATCHからはRELEASEへ)
 *
 * pressedは呼び出しごとの押下エッジ(このループでボタンが押された瞬間か)。
 * 押下がなくても毎回呼び，保留中のシングルクリックのタイムアウト判定を行う。
 *
 * lower_arm.left等はビットフィールドでアドレスを取れないため，現在の状態を受け取り
 * 更新後の状態を返す形にしている。
 */
static uint8_t apply_hand_click(uint8_t state, hand_click_state_t *click, bool pressed){
    TickType_t now = xTaskGetTickCount();

    // 保留中のシングルクリックがダブルクリックウィンドウを過ぎていれば，
    // ダブルクリックではなかったと確定してシングルクリックの動作を適用する
    if(click->pending_single &&
       (now - click->press_tick) >= pdMS_TO_TICKS(DOUBLE_CLICK_WINDOW_MS)){
        state = (state == HAND_STATE_CATCH) ? HAND_STATE_HOLD : HAND_STATE_CATCH;
        click->pending_single = false;
    }

    if(!pressed)return state;

    if(click->pending_single){
        // ウィンドウ内の2回目の押下 = ダブルクリック確定
        state = (state == HAND_STATE_RELEASE) ? HAND_STATE_HOLD : HAND_STATE_RELEASE;
        click->pending_single = false;
    }else{
        // 1回目の押下。ダブルクリックかどうか確定するまで保留する
        click->pending_single = true;
        click->press_tick = now;
    }
    return state;
}


// homing中は動かない
void lower_arm_move(
    int16_t dx, int16_t dy,
    bool left_pressed, bool middle_pressed, bool right_pressed,
    bool release_all_pressed,
    bool expand_toggle,
    int16_t d_shaft_fine,
    bool shaft_rotate_toggle
){
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

    lower_arm.left   = apply_hand_click(lower_arm.left,   &left_click_state,   left_pressed);
    lower_arm.middle = apply_hand_click(lower_arm.middle, &middle_click_state, middle_pressed);
    lower_arm.right  = apply_hand_click(lower_arm.right,  &right_click_state,  right_pressed);

    // 3本とも保持状態のときだけ，プラスボタンで3本同時にリリースする
    // (いずれかがキャッチ状態のときは切り替わらない)
    if(release_all_pressed &&
       lower_arm.left   == HAND_STATE_HOLD &&
       lower_arm.middle == HAND_STATE_HOLD &&
       lower_arm.right  == HAND_STATE_HOLD){
        lower_arm.left   = HAND_STATE_RELEASE;
        lower_arm.middle = HAND_STATE_RELEASE;
        lower_arm.right  = HAND_STATE_RELEASE;
    }else if(release_all_pressed &&
       lower_arm.left   == HAND_STATE_RELEASE &&
       lower_arm.middle == HAND_STATE_RELEASE &&
       lower_arm.right  == HAND_STATE_RELEASE){
        lower_arm.left   = HAND_STATE_HOLD;
        lower_arm.middle = HAND_STATE_HOLD;
        lower_arm.right  = HAND_STATE_HOLD;
       }

    if(expand_toggle)       {TOGGLE(lower_arm.expand, 1);}
    if(shaft_rotate_toggle) {TOGGLE(lower_arm.shaft_rotate, 1);} // ハンドの向きを180度回転させる

    // シャフト角度の微調整。L/Rの押しっぱなしで±LOWER_ARM_SHAFT_FINE_MAX_DEGの範囲に収める
    int32_t fine = (int32_t)lower_arm.shaft_fine + d_shaft_fine;
    if(fine < -LOWER_ARM_SHAFT_FINE_MAX_DEG) fine = -LOWER_ARM_SHAFT_FINE_MAX_DEG;
    if(fine >  LOWER_ARM_SHAFT_FINE_MAX_DEG) fine =  LOWER_ARM_SHAFT_FINE_MAX_DEG;
    lower_arm.shaft_fine = (int8_t)fine;
}


// homing中は動かない
void upper_arm_move(
    int16_t dx, int16_t dy, int16_t dz,
    int16_t d_shaft_fine,
    bool shaft_rotate_toggle
){
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

    // int32_tで加算してからクランプする(int16_tのまま加算し続けるとオーバーフローで
    // 値が反転し，Zサーボへ急激な指令が飛ぶ恐れがあるため)
    int32_t z = (int32_t)upper_arm.z + dz;
    if(z < UPPER_ARM_Z_MIN) z = UPPER_ARM_Z_MIN;
    if(z > UPPER_ARM_Z_MIN + UPPER_ARM_Z_RANGE) z = UPPER_ARM_Z_MIN + UPPER_ARM_Z_RANGE;
    upper_arm.z = (int16_t)z;

    if(shaft_rotate_toggle) {TOGGLE(upper_arm.shaft_rotate, 1);} // ハンドの向きを90度回転させる

    // シャフト角度の微調整。L/Rの押しっぱなしで±UPPER_ARM_SHAFT_FINE_MAX_DEGの範囲に収める
    int32_t fine = (int32_t)upper_arm.shaft_fine + d_shaft_fine;
    if(fine < -UPPER_ARM_SHAFT_FINE_MAX_DEG) fine = -UPPER_ARM_SHAFT_FINE_MAX_DEG;
    if(fine >  UPPER_ARM_SHAFT_FINE_MAX_DEG) fine =  UPPER_ARM_SHAFT_FINE_MAX_DEG;
    upper_arm.shaft_fine = (int8_t)fine;
}


esp_err_t send_lower_arm(){
    if(lower_arm_homing_in_progress)return ESP_ERR_NOT_ALLOWED;

    can_command_data_t com = {
        .id = CAN_ID_LOWER_ARM_COMMAND,
        .data.lower_arm = lower_arm,
    };
    
    esp_err_t prev_send_lower_arm_error = send_lower_arm_error;
    send_lower_arm_error = can_tx(&com);

    if(send_lower_arm_error != ESP_OK){
        
        if( (send_lower_arm_error == prev_send_lower_arm_error) && 
            (xTaskGetTickCount() - send_lower_arm_error_loged_tick < pdMS_TO_TICKS(PREVENT_SPAMING_MS)) ){
            return send_lower_arm_error;
        }
        send_lower_arm_error_loged_tick = xTaskGetTickCount();
        ESP_LOGE(ARM_TAG, "CAN_ID_LOWER_ARM_COMMAND failed.");
    }
    return send_lower_arm_error;
}


esp_err_t send_upper_arm(){
    if(upper_arm_homing_in_progress)return ESP_ERR_NOT_ALLOWED;

    can_command_data_t com = {
        .id = CAN_ID_UPPER_ARM_COMMAND,
        .data.upper_arm = upper_arm,
    };

    esp_err_t prev_send_upper_arm_error = send_upper_arm_error;
    send_upper_arm_error = can_tx(&com);

    if(send_upper_arm_error != ESP_OK){
        
        if( (send_upper_arm_error == prev_send_upper_arm_error) && 
            (xTaskGetTickCount() - send_upper_arm_error_loged_tick < pdMS_TO_TICKS(PREVENT_SPAMING_MS)) ){
            return send_upper_arm_error;
        }
        send_upper_arm_error_loged_tick = xTaskGetTickCount();
        ESP_LOGE(ARM_TAG, "CAN_ID_UPPER_ARM_COMMAND failed.");
    }
    return send_upper_arm_error;
}


/*
 * CAN RX callbackを登録する
 *
 * can_protocol記載の
 *   HOMING -> HOMING_ACK -> (Homing処理) -> HOMING_DONE -> HOMING_DONE_ACK
 * のシーケンスに従う。
 */
esp_err_t arms_init(){
    if(arms_init_already_done)return ESP_ERR_NOT_ALLOWED;

    can_register_rx_callback(
        CAN_ID_LOWER_HOMING_ACK,
        lower_arm_homing_ack_notify
    );

    can_register_rx_callback(
        CAN_ID_UPPER_HOMING_ACK,
        upper_arm_homing_ack_notify
    );

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


/*
 * Lower/Upper Arm Homing要求(HOMING)を送信する内部ヘルパー。
 * 新規送信・再送どちらからも使う。
 */
static esp_err_t send_lower_homing_request(){
    can_command_data_t command = {
        .id = CAN_ID_LOWER_HOMING,
        .data.homing_sequence = lower_arm_homing_sequence,
    };
    return can_tx(&command);
}

static esp_err_t send_upper_homing_request(){
    can_command_data_t command = {
        .id = CAN_ID_UPPER_HOMING,
        .data.homing_sequence = upper_arm_homing_sequence,
    };
    return can_tx(&command);
}


/*can_tx(&com)
 * Lower Arm Homing開始
 *
 * ESP32 -> STM32
 * CAN_ID_LOWER_HOMING
 */
esp_err_t lower_arm_homing(){
    if(lower_arm_homing_in_progress)return ESP_ERR_NOT_FINISHED;

    lower_arm_homing_sequence++; // 新しいホーミング試行ごとに番号を進める(0-255循環)
    lower_arm_homing_ack_received = false;

    esp_err_t err = send_lower_homing_request();

    if (err != ESP_OK) {
        ESP_LOGE(ARM_TAG,"lower arm homing command failed.");
        return err;
    }

    lower_arm_homing_in_progress = true;
    lower_arm_homing_start_tick = xTaskGetTickCount();
    lower_arm_homing_request_sent_tick = lower_arm_homing_start_tick;

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

    upper_arm_homing_sequence++; // 新しいホーミング試行ごとに番号を進める(0-255循環)
    upper_arm_homing_ack_received = false;

    esp_err_t err = send_upper_homing_request();

    if (err != ESP_OK) {
        ESP_LOGE(ARM_TAG,"upper arm homing command failed.");
        return err;
    }

    upper_arm_homing_in_progress = true;
    upper_arm_homing_start_tick = xTaskGetTickCount();
    upper_arm_homing_request_sent_tick = upper_arm_homing_start_tick;

    ESP_LOGI(ARM_TAG,"upper arm homing start. sequence=%u",upper_arm_homing_sequence);

    return ESP_OK;
}


/*
 * STM32 -> ESP32
 * CAN_ID_LOWER_HOMING_ACK
 *
 * Homing要求が受理され，ホーミング処理が開始されたことの確認。
 * CAN rx taskからcallbackされるので重い処理はしない。
 */
static void lower_arm_homing_ack_notify(const can_data_t *data){
    if (data->homing_sequence != lower_arm_homing_sequence) {
        ESP_LOGW(ARM_TAG, "lower homing ACK sequence mismatch: rx=%u expected=%u",
            data->homing_sequence, lower_arm_homing_sequence);
        return;
    }
    lower_arm_homing_ack_received = true;
    ESP_LOGI(ARM_TAG, "lower arm homing ACK received. sequence=%u", lower_arm_homing_sequence);
}


/*
 * STM32 -> ESP32
 * CAN_ID_UPPER_HOMING_ACK
 */
static void upper_arm_homing_ack_notify(const can_data_t *data){
    if (data->homing_sequence != upper_arm_homing_sequence) {
        ESP_LOGW(ARM_TAG, "upper homing ACK sequence mismatch: rx=%u expected=%u",
            data->homing_sequence, upper_arm_homing_sequence);
        return;
    }
    upper_arm_homing_ack_received = true;
    ESP_LOGI(ARM_TAG, "upper arm homing ACK received. sequence=%u", upper_arm_homing_sequence);
}


/*
 * STM32 -> ESP32
 * CAN_ID_LOWER_HOMING_DONE
 *
 * 受信したらCAN_ID_LOWER_HOMING_DONE_ACKを返す。
 * STM32側はACKが届くまでHOMING_DONEを再送してくるため，
 * 既にhoming完了済み(in_progress==false)でも同じsequenceならACKを返し直す(冪等)。
 *
 * CAN rx taskからcallbackされるので重い処理はしない。
 */
static void lower_arm_homing_done_notify(const can_data_t *data){
    direct_t larm = LOWER_ARM_HOME_COORDINATE;
    lower_arm.x = larm.x;
    lower_arm.y = larm.y;
    lower_arm.shaft_rotate = 0; // homingでハンドの向きは基準位置に戻るので回転も解除する
    lower_arm.shaft_fine = 0;   // 微調整オフセットも基準位置(0度)へ戻す

    if (data->homing_sequence != lower_arm_homing_sequence) {
        ESP_LOGE(ARM_TAG,"lower homing DONE sequence error: rx=%u expected=%u",
            data->homing_sequence,lower_arm_homing_sequence
        );
        return;
    }

    if (lower_arm_homing_in_progress) {
        lower_arm_homing_in_progress = false;
        ESP_LOGI(ARM_TAG, "lower arm homing done. sequence=%u", lower_arm_homing_sequence);
    }

    can_command_data_t ack = {
        .id = CAN_ID_LOWER_HOMING_DONE_ACK,
        .data.homing_sequence = lower_arm_homing_sequence,
    };
    can_tx(&ack);
}


/*
 * STM32 -> ESP32
 * CAN_ID_UPPER_HOMING_DONE
 *
 * (lower側と同様，STM32からの再送に対して冪等にDONE_ACKを返す)
 */
static void upper_arm_homing_done_notify(const can_data_t *data){
    direct_t uarm = UPPER_ARM_HOME_COORDINATE;
    upper_arm.x = uarm.x;
    upper_arm.y = uarm.y;
    upper_arm.shaft_rotate = 0; // homingでハンドの向きは基準位置に戻るので回転も解除する
    upper_arm.shaft_fine = 0;   // 微調整オフセットも基準位置(0度)へ戻す

    if (data->homing_sequence != upper_arm_homing_sequence) {
        ESP_LOGE(ARM_TAG, "upper homing DONE sequence error: rx=%u expected=%u",
            data->homing_sequence, upper_arm_homing_sequence);
        return;
    }

    if (upper_arm_homing_in_progress) {
        upper_arm_homing_in_progress = false;
        ESP_LOGI(ARM_TAG, "upper arm homing done. sequence=%u", upper_arm_homing_sequence);
    }

    can_command_data_t ack = {
        .id = CAN_ID_UPPER_HOMING_DONE_ACK,
        .data.homing_sequence = upper_arm_homing_sequence,
    };
    can_tx(&ack);
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
        if (!lower_arm_homing_ack_received &&
            (now - lower_arm_homing_request_sent_tick) >= pdMS_TO_TICKS(HOMING_REQUEST_RETRY_MS)) {
            ESP_LOGW(ARM_TAG, "lower arm homing request retry. sequence=%u", lower_arm_homing_sequence);
            send_lower_homing_request();
            lower_arm_homing_request_sent_tick = now;
        }

        if ((now - lower_arm_homing_start_tick) >= pdMS_TO_TICKS(HOMING_LOWER_ARM_TIMEOUT_MS)) {
            ESP_LOGE(ARM_TAG, "lower arm homing timeout. sequence=%u", lower_arm_homing_sequence);

            lower_arm_homing_in_progress = false;
        }
    }

    /* Upper Arm */
    if (upper_arm_homing_in_progress) {
        if (!upper_arm_homing_ack_received &&
            (now - upper_arm_homing_request_sent_tick) >= pdMS_TO_TICKS(HOMING_REQUEST_RETRY_MS)) {
            ESP_LOGW(ARM_TAG, "upper arm homing request retry. sequence=%u", upper_arm_homing_sequence);
            send_upper_homing_request();
            upper_arm_homing_request_sent_tick = now;
        }

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
        "lower_arm: CART(%4dmm,%4dmm), POR(%4.2fmm,%3.2f°), shaft_rotate %1d, shaft_fine %3d, "
        "HAND{left %1d, middle %1d, right %d, expand %1d}\n",
        lower_arm.x,
        lower_arm.y,
        pol.r,
        pol.theta / (2 * M_PI) * 360,
        lower_arm.shaft_rotate,
        lower_arm.shaft_fine,
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
        "upper_arm: CART(%4dmm,%4dmm), POR(%4.2f,%3.2f°), Z %d, shaft_rotate %1d, shaft_fine %3d\n",
        upper_arm.x,
        upper_arm.y,
        pol.r,
        pol.theta / (2 * M_PI) * 360,
        upper_arm.z,
        upper_arm.shaft_rotate,
        upper_arm.shaft_fine
    );
}

void arms_dump(){
    lower_arm_dump();
    upper_arm_dump();
}
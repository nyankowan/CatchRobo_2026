#include "stm_can.h"

HAL_StatusTypeDef stm_can_send(CAN_HandleTypeDef *hcan, const can_command_data_t *com){
    CAN_TxHeaderTypeDef tx_header = {0};
    uint32_t tx_mailbox;

    can_dlc_t dlc = can_protocol_get_dlc(com->id);

    if (dlc < 0 || dlc > CAN_DLC_MAX) {
        return HAL_ERROR;
    }

    tx_header.StdId = com->id;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = dlc;
    tx_header.TransmitGlobalTime = DISABLE;

    return HAL_CAN_AddTxMessage(
        hcan,
        &tx_header,
        com->data.raw,
        &tx_mailbox
    );
}
#include "fdcan_bsp_poll.h"
#include <stddef.h>
#include <string.h>

static uint8_t valid_handle(FDCAN_HandleTypeDef *h)
{
    return h != NULL && h->Instance == FDCAN1;
}

HAL_StatusTypeDef FDCAN_BSP_Init(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef filter = {0};
    HAL_StatusTypeDef status;
    if (!valid_handle(hfdcan) || hfdcan->State != HAL_FDCAN_STATE_READY ||
        hfdcan->Init.StdFiltersNbr < 1U ||
        hfdcan->Init.RxFifo0ElmtsNbr == 0U ||
        hfdcan->Init.TxFifoQueueElmtsNbr == 0U ||
        hfdcan->Init.FrameFormat != FDCAN_FRAME_CLASSIC ||
        hfdcan->Init.RxFifo0ElmtSize != FDCAN_DATA_BYTES_8 ||
        hfdcan->Init.TxElmtSize != FDCAN_DATA_BYTES_8 ||
        (hfdcan->Init.Mode != FDCAN_MODE_NORMAL &&
         hfdcan->Init.Mode != FDCAN_MODE_INTERNAL_LOOPBACK))
        return HAL_ERROR;

    /* 掩码=0：所有标准ID通过。电机层再只取0x201。 */
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0;
    filter.FilterID2 = 0;
    status = HAL_FDCAN_ConfigFilter(hfdcan, &filter);
    if (status != HAL_OK) return status;
    status = HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT,
                                         FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    if (status != HAL_OK) return status;
    return HAL_FDCAN_Start(hfdcan);
}

HAL_StatusTypeDef FDCAN_BSP_SendData(FDCAN_HandleTypeDef *hfdcan,
                                    const uint8_t data[8], uint16_t id)
{
    FDCAN_TxHeaderTypeDef header = {0};
    uint8_t bytes[8];
    if (!valid_handle(hfdcan) || data == NULL || id > 0x7FFU)
        return HAL_ERROR;
    if (hfdcan->State != HAL_FDCAN_STATE_BUSY) return HAL_ERROR;
    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0U) return HAL_BUSY;
    header.Identifier = id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0;
    memcpy(bytes, data, sizeof(bytes));
    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &header, bytes);
}

HAL_StatusTypeDef FDCAN_BSP_ReceiveData(FDCAN_HandleTypeDef *hfdcan,
                                       FDCAN_RxHeaderTypeDef *header,
                                       uint8_t data[64])
{
    if (!valid_handle(hfdcan) || header == NULL || data == NULL ||
        hfdcan->State != HAL_FDCAN_STATE_BUSY) return HAL_ERROR;
    if (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) == 0U)
        return HAL_BUSY;
    return HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, header, data);
}

uint8_t FDCAN_BSP_TakeRxLoss(FDCAN_HandleTypeDef *hfdcan)
{
    if (!valid_handle(hfdcan)) return 0;
    if (__HAL_FDCAN_GET_FLAG(hfdcan, FDCAN_FLAG_RX_FIFO0_MESSAGE_LOST) != 0U)
    {
        __HAL_FDCAN_CLEAR_FLAG(hfdcan, FDCAN_FLAG_RX_FIFO0_MESSAGE_LOST);
        return 1;
    }
    return 0;
}

HAL_StatusTypeDef FDCAN_BSP_CancelPending(FDCAN_HandleTypeDef *hfdcan)
{
    uint32_t pending;
    if (!valid_handle(hfdcan) || hfdcan->State != HAL_FDCAN_STATE_BUSY)
        return HAL_ERROR;
    pending = hfdcan->Instance->TXBRP;
    if (pending == 0U) return HAL_OK;
    return HAL_FDCAN_AbortTxRequest(hfdcan, pending);
}

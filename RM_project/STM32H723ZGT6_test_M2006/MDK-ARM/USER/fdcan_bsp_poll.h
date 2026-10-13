#ifndef FDCAN_BSP_POLL_H
#define FDCAN_BSP_POLL_H

#include "fdcan.h"
#ifdef __cplusplus
extern "C" {
#endif

/* 在 MX_FDCAN1_Init 后调用一次。只配置过滤器并启动，不改时钟/模式/引脚，
 * 不开启中断。支持正常模式和内部回环，要求标准过滤器>=1、收发队列非空。 */
HAL_StatusTypeDef FDCAN_BSP_Init(FDCAN_HandleTypeDef *hfdcan);

/* 标准数据帧，固定8字节。HAL_OK=入队，HAL_BUSY=队列满，绝不等待空位。 */
HAL_StatusTypeDef FDCAN_BSP_SendData(FDCAN_HandleTypeDef *hfdcan,
                                    const uint8_t data[8], uint16_t id);

/* 轮询取一帧，HAL_BUSY=暂无数据。data须有64字节空间，调用者检查帧格式/DLC。
 * 不调用电机解析；通用回环测试也能用。 */
HAL_StatusTypeDef FDCAN_BSP_ReceiveData(FDCAN_HandleTypeDef *hfdcan,
                                       FDCAN_RxHeaderTypeDef *header,
                                       uint8_t data[64]);
/* 查询并清除FIFO0丢帧标志：1=发生过丢帧，0=没有。无需中断。 */
uint8_t FDCAN_BSP_TakeRxLoss(FDCAN_HandleTypeDef *hfdcan);
/* 请求撤销本外设全部待发送帧。正在总线上发送的帧不保证能撤销。 */
HAL_StatusTypeDef FDCAN_BSP_CancelPending(FDCAN_HandleTypeDef *hfdcan);

#ifdef __cplusplus
}
#endif
#endif

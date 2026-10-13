#ifndef M2006_POLL_H
#define M2006_POLL_H

#include "fdcan_bsp_poll.h"
#ifdef __cplusplus
extern "C" {
#endif

/* 单台M2006+C610，固定电调ID=1，FDCAN1。所有接口由同一主循环/任务调用。 */
#define M2006_REDUCTION_RATIO       36.0f
#define M2006_COUNTS_PER_REV        8192
#define M2006_FEEDBACK_TIMEOUT_MS   20U
#define M2006_CURRENT_LIMIT         1000  /* 原始指令1000=1A，初调限幅非安全保证 */
#define M2006_SPEED_LIMIT_RPM       30.0f /* 输出轴目标速度限幅 */

typedef struct {
    float Kp, Ki, Kd;
    float OutputLimit;
    float IntegralLimit;  /* I项的限幅，单位与输出相同 */
    float Integral, PreviousError;
    uint8_t HasPrevious;
} M2006_PID;

typedef struct {
    uint16_t Encoder;      /* 转子单圈计数0~8191 */
    int16_t RotorSpeedRPM; /* 有符号转子转速rpm */
    int16_t CurrentRaw;    /* 有符号电流反馈原始值；不是机械转矩 */
    int64_t TotalCount;    /* 相对零点累计转子计数 */
    uint32_t LastRxMs, RxCount;
    uint8_t Received, PositionValid;
} M2006_Feedback;

extern M2006_PID M2006_Speed_Pid;
extern M2006_PID M2006_Pos_Pid;
extern volatile uint32_t M2006_TxFailCount;
extern volatile uint32_t M2006_RxLossCount;

/* 初始化仅调用一次，内部调用FDCAN_BSP_Init。模式必须是NORMAL，否则报错，
 * 不自动改模式；不要先在main里另调CAN_Init或HAL_FDCAN_Start。 */
HAL_StatusTypeDef M2006_Init(FDCAN_HandleTypeDef *hfdcan);
/* 尽可能频繁调用：取当前队列快照内的帧，只解析0x201，执行离线检查。
 * HAL_OK=轮询正常(不保证收到帧)；HAL_TIMEOUT=轮询/反馈超时；
 * HAL_ERROR=丢帧或驱动错误。异常撤销使能，尝试零电流；下次继续轮询。
 * 接收队列只有3帧时，禁止用长时间HAL_Delay/阻塞串口占住主循环。 */
HAL_StatusTypeDef M2006_Poll(void);
uint8_t M2006_Enable(void); /* 只在用户明确启动时调用；成功=1，需已有新鲜反馈 */
uint8_t M2006_IsEnabled(void);
uint8_t M2006_IsOnline(void);
HAL_StatusTypeDef M2006_Stop(void); /* 取消待发指令并尝试发零电流；不等于机械抱死 */

/* 三选一，约每1ms调用一次；同一毫秒重复调用返回HAL_BUSY。
 * Set_I：原始电流指令，1000=1A；Set_Speed：输出轴rpm；
 * Set_Pos/Set_Ang：输出轴相对角度，单位度，内部位置->速度->电流双环。
 * PID使用实际毫秒间隔，Ki乘dt，Kd除dt。初始增益全0，需要自行整定。
 * 未使能/反馈超时时尝试发0；HAL_OK仅代表入队成功。 */
HAL_StatusTypeDef M2006_Set_I(int32_t current_raw);
HAL_StatusTypeDef M2006_Set_Speed(float output_rpm);
HAL_StatusTypeDef M2006_Set_Pos(float output_deg);
HAL_StatusTypeDef M2006_Set_Ang(float output_deg);

uint8_t M2006_ReadFeedback(M2006_Feedback *out);
float M2006_Get_Speed(void); /* 输出轴rpm，使用前检查IsOnline */
float M2006_Get_Pos(void);   /* 输出轴度，使用前检查PositionValid */
int16_t M2006_Get_CurrentRaw(void);
uint8_t M2006_ZeroPosition(void); /* 停机且在线时，当前点设为零；成功=1 */

#ifdef __cplusplus
}
#endif
#endif

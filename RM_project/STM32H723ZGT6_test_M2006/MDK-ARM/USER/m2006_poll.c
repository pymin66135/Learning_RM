#include "m2006_poll.h"
#include <math.h>
#include <string.h>

M2006_PID M2006_Speed_Pid = {0, 0, 0, M2006_CURRENT_LIMIT, 300, 0, 0, 0};
M2006_PID M2006_Pos_Pid = {0, 0, 0, M2006_SPEED_LIMIT_RPM, 5, 0, 0, 0};
volatile uint32_t M2006_TxFailCount = 0;
volatile uint32_t M2006_RxLossCount = 0;
static FDCAN_HandleTypeDef *motor_can;
static M2006_Feedback feedback;
static uint8_t enabled, mode, position_lost;
static uint32_t last_poll_ms, last_control_ms;

volatile uint32_t can_raw_rx_count = 0;
volatile uint32_t can_last_rx_id = 0;

static float limitf(float x, float limit)
{
    if (x > limit) return limit;
    if (x < -limit) return -limit;
    return x;
}

static void reset_pid(void)
{
    M2006_Speed_Pid.Integral = 0;
    M2006_Speed_Pid.PreviousError = 0;
    M2006_Speed_Pid.HasPrevious = 0;
    M2006_Pos_Pid.Integral = 0;
    M2006_Pos_Pid.PreviousError = 0;
    M2006_Pos_Pid.HasPrevious = 0;
}

static void disarm(void)
{
    enabled = 0;
    mode = 0;
    reset_pid();
}

static int16_t read_i16(const uint8_t *data)
{
    uint16_t u = ((uint16_t)data[0] << 8) | data[1];
    return (int16_t)((u & 0x8000U) ? (int32_t)u - 65536 : (int32_t)u);
}

static HAL_StatusTypeDef send_current(int32_t current)
{
    uint8_t data[8] = {0};
    uint16_t packed;
    HAL_StatusTypeDef status;
    if (motor_can == NULL) return HAL_ERROR;
    if (current > M2006_CURRENT_LIMIT) current = M2006_CURRENT_LIMIT;
    if (current < -M2006_CURRENT_LIMIT) current = -M2006_CURRENT_LIMIT;
    if (current > 10000) current = 10000;
    if (current < -10000) current = -10000;
    packed = (uint16_t)(int16_t)current;
    data[2] = (uint8_t)(packed >> 8);
    data[3] = (uint8_t)packed; /* ID2~4的6个字节保持0 */
    status = FDCAN_BSP_SendData(motor_can, data, 0x200);
    if (status != HAL_OK)
    {
        ++M2006_TxFailCount;
        disarm();
        (void)FDCAN_BSP_CancelPending(motor_can);
    }
    return status;
}

HAL_StatusTypeDef M2006_Stop(void)
{
    HAL_StatusTypeDef cancelled;
    disarm();
    cancelled = FDCAN_BSP_CancelPending(motor_can);
    if (cancelled != HAL_OK) ++M2006_TxFailCount;
    /* 撤销可能异步完成；若仍满则返回BUSY，下个控制周期继续尝试零电流。 */
    return send_current(0);
}

static void lose_feedback(void)
{
    position_lost = 1;
    feedback.PositionValid = 0;
    feedback.Received = 0; /* 不能拿故障前的旧反馈重新使能 */
    (void)M2006_Stop();
}

static void parse_feedback(const uint8_t data[8], uint32_t now)
{
    uint16_t angle = ((uint16_t)data[0] << 8) | data[1];
    int32_t delta;
    if (angle >= M2006_COUNTS_PER_REV) return;
    if (!feedback.Received)
    {
        if (!position_lost) feedback.TotalCount = 0;
        feedback.PositionValid = (uint8_t)!position_lost;
    }
    else if (feedback.PositionValid)
    {
        delta = (int32_t)angle - feedback.Encoder;
        if (delta > 4096) delta -= 8192;
        else if (delta < -4096) delta += 8192;
        if (delta == 4096 || delta == -4096)
        {
            position_lost = 1;
            feedback.PositionValid = 0;
            (void)M2006_Stop(); /* 恰好半圈，方向无法确定 */
        }
        else feedback.TotalCount += delta;
    }
    feedback.Encoder = angle;
    feedback.RotorSpeedRPM = read_i16(data + 2);
    feedback.CurrentRaw = read_i16(data + 4);
    feedback.LastRxMs = now;
    feedback.Received = 1;
    ++feedback.RxCount;
    /* C610的字节6/7是保留字段，不是3508温度。 */
}

HAL_StatusTypeDef M2006_Init(FDCAN_HandleTypeDef *hfdcan)
{
    HAL_StatusTypeDef status;
    if (hfdcan == NULL || hfdcan->Instance != FDCAN1 ||
        hfdcan->Init.Mode != FDCAN_MODE_NORMAL) return HAL_ERROR;
    status = FDCAN_BSP_Init(hfdcan);
    if (status != HAL_OK) return status;
    motor_can = hfdcan;
    memset(&feedback, 0, sizeof(feedback));
    disarm();
    position_lost = 0;
    M2006_TxFailCount = M2006_RxLossCount = 0;
    last_poll_ms = last_control_ms = HAL_GetTick();
    return send_current(0);
}

HAL_StatusTypeDef M2006_Poll(void)
{
    FDCAN_RxHeaderTypeDef header = {0};
    uint8_t data[64];
    uint32_t now, count, i;
    uint8_t lost, gap;
    HAL_StatusTypeDef status;
    if (motor_can == NULL) return HAL_ERROR;
    now = HAL_GetTick();
    lost = FDCAN_BSP_TakeRxLoss(motor_can);
    gap = (uint32_t)(now - last_poll_ms) > M2006_FEEDBACK_TIMEOUT_MS;
    if (feedback.Received &&
        (uint32_t)(now - feedback.LastRxMs) > M2006_FEEDBACK_TIMEOUT_MS) gap = 1;
    last_poll_ms = now;
    if (enabled && (uint32_t)(now - last_control_ms) > M2006_FEEDBACK_TIMEOUT_MS)
        (void)M2006_Stop();
    count = HAL_FDCAN_GetRxFifoFillLevel(motor_can, FDCAN_RX_FIFO0);
    if (lost || gap)
    {
        ++M2006_RxLossCount;
        lose_feedback();
    }
    /* 只处理调用时队列内的帧：有界轮询，不在持续流量下死循环。 */
    for (i = 0; i < count; ++i)
    {
        status = FDCAN_BSP_ReceiveData(motor_can, &header, data);
        if (status != HAL_OK)
        {
            ++M2006_RxLossCount;
            lose_feedback();
            return HAL_ERROR;
        }
        /* 故障时丢弃旧队列；下一次轮询才能接受新鲜反馈。 */
		
		can_raw_rx_count++;
		can_last_rx_id = header.Identifier;
        if (lost || gap) continue;
        if (header.IdType == FDCAN_STANDARD_ID &&
            header.RxFrameType == FDCAN_DATA_FRAME &&
            header.FDFormat == FDCAN_CLASSIC_CAN &&
            header.DataLength == FDCAN_DLC_BYTES_8 &&
            header.Identifier == 0x202U)
            parse_feedback(data, now);
    }
    if (FDCAN_BSP_TakeRxLoss(motor_can))
    {
        ++M2006_RxLossCount;
        lose_feedback();
        return HAL_ERROR;
    }
    if (lost) return HAL_ERROR;
    return gap ? HAL_TIMEOUT : HAL_OK;
}

uint8_t M2006_IsOnline(void)
{
    return feedback.Received && motor_can != NULL &&
        (uint32_t)(HAL_GetTick() - feedback.LastRxMs) <= M2006_FEEDBACK_TIMEOUT_MS &&
        (uint32_t)(HAL_GetTick() - last_poll_ms) <= M2006_FEEDBACK_TIMEOUT_MS;
}

uint8_t M2006_Enable(void)
{
    if (!M2006_IsOnline()) return 0;
    reset_pid();
    mode = 0;
    last_control_ms = HAL_GetTick() - 1U;
    enabled = 1;
    return 1;
}

uint8_t M2006_IsEnabled(void)
{
    if (enabled && (!M2006_IsOnline() ||
        (uint32_t)(HAL_GetTick() - last_control_ms) > M2006_FEEDBACK_TIMEOUT_MS))
        (void)M2006_Stop();
    return enabled;
}

/* 0=禁止，1=可控制，2=同一毫秒内重复调用。 */
static uint8_t ready(uint8_t wanted_mode, float *dt)
{
    uint32_t now = HAL_GetTick(), elapsed = now - last_control_ms;
    if (!M2006_IsEnabled() || (wanted_mode == 3U && !feedback.PositionValid))
    {
        if (enabled) (void)M2006_Stop();
        else disarm();
        return 0;
    }
    if (elapsed == 0U) return 2;
    *dt = (float)elapsed * 0.001f;
    last_control_ms = now;
    if (mode != wanted_mode)
    {
        reset_pid();
        mode = wanted_mode;
    }
    return 1;
}

static uint8_t pid_step(M2006_PID *pid, float error, float dt,
                         float hard_limit, float *out)
{
    float limit, candidate, derivative, sum;
    if (!isfinite(error) || !isfinite(pid->Kp) || !isfinite(pid->Ki) ||
        !isfinite(pid->Kd) || !isfinite(pid->Integral) ||
        !isfinite(pid->OutputLimit) || !isfinite(pid->IntegralLimit) ||
        pid->OutputLimit <= 0 || pid->IntegralLimit < 0) return 0;
    limit = pid->OutputLimit < hard_limit ? pid->OutputLimit : hard_limit;
    derivative = pid->HasPrevious ? (error - pid->PreviousError) / dt : 0;
    candidate = limitf(pid->Integral + pid->Ki * error * dt, pid->IntegralLimit);
    sum = pid->Kp * error + candidate + pid->Kd * derivative;
    if (!isfinite(sum)) return 0;
    if (!((sum > limit && error > 0) || (sum < -limit && error < 0)))
        pid->Integral = candidate; /* 输出饱和时抑制积分继续增大 */
    sum = pid->Kp * error + pid->Integral + pid->Kd * derivative;
    if (!isfinite(sum)) return 0;
    *out = limitf(sum, limit);
    pid->PreviousError = error;
    pid->HasPrevious = 1;
    return 1;
}

HAL_StatusTypeDef M2006_Set_I(int32_t current_raw)
{
    float dt;
    uint8_t state = ready(1, &dt);
    if (state == 2U) return HAL_BUSY;
    return send_current(state ? current_raw : 0);
}

HAL_StatusTypeDef M2006_Set_Speed(float output_rpm)
{
    float current, dt;
    uint8_t state = ready(2, &dt);
    if (state == 2U) return HAL_BUSY;
    if (!state) return send_current(0);
    if (!isfinite(output_rpm) || !pid_step(&M2006_Speed_Pid,
        limitf(output_rpm, M2006_SPEED_LIMIT_RPM) - M2006_Get_Speed(),
        dt, M2006_CURRENT_LIMIT, &current)) return M2006_Stop();
    return send_current((int32_t)current);
}

HAL_StatusTypeDef M2006_Set_Pos(float output_deg)
{
    float speed, current, dt;
    uint8_t state = ready(3, &dt);
    if (state == 2U) return HAL_BUSY;
    if (!state) return send_current(0);
    if (!isfinite(output_deg) || !pid_step(&M2006_Pos_Pid,
        output_deg - M2006_Get_Pos(), dt, M2006_SPEED_LIMIT_RPM, &speed) ||
        !pid_step(&M2006_Speed_Pid, speed - M2006_Get_Speed(), dt,
                   M2006_CURRENT_LIMIT, &current)) return M2006_Stop();
    return send_current((int32_t)current);
}

HAL_StatusTypeDef M2006_Set_Ang(float output_deg) { return M2006_Set_Pos(output_deg); }
uint8_t M2006_ReadFeedback(M2006_Feedback *out)
{
    if (out == NULL) return 0;
    *out = feedback;
    return out->Received;
}
float M2006_Get_Speed(void) { return feedback.RotorSpeedRPM / M2006_REDUCTION_RATIO; }
float M2006_Get_Pos(void)
{
    return (float)((double)feedback.TotalCount * 360.0 /
        (M2006_COUNTS_PER_REV * (double)M2006_REDUCTION_RATIO));
}
int16_t M2006_Get_CurrentRaw(void) { return feedback.CurrentRaw; }
uint8_t M2006_ZeroPosition(void)
{
    if (enabled || !M2006_IsOnline()) return 0;
    feedback.TotalCount = 0;
    feedback.PositionValid = 1;
    position_lost = 0;
    reset_pid();
    return 1;
}

#include "m2006_simple.h"

M2006Simple_Feedback m2006_simple_feedback = {0};
M2006Simple_PI m2006_simple_pi = {0.5f, 0.0f, 0.0f};

static FDCAN_HandleTypeDef *motor_can;

float m2006_position_deg = 0.0f;


/* Unit: rotor rpm / output-shaft degree. */
float m2006_position_kp = 100.0f;

/* Maximum target ROTOR speed. */
float m2006_position_speed_limit = 10000.0f;

static uint8_t position_initialized = 0;

float m2006_speed_filtered = 0.0f;
float m2006_speed_alpha = 0.3f;
static uint8_t speed_filter_initialized = 0;





static float clamp_current(float value)
{
    if (value > M2006_SIMPLE_CURRENT_LIMIT) return M2006_SIMPLE_CURRENT_LIMIT;
    if (value < -M2006_SIMPLE_CURRENT_LIMIT) return -M2006_SIMPLE_CURRENT_LIMIT;
    return value;
}

static int16_t read_i16(const uint8_t *data)
{
    int32_t value = ((uint16_t)data[0] << 8) | data[1];
    if (value >= 32768) value -= 65536;
    return (int16_t)value;
}

HAL_StatusTypeDef M2006Simple_Init(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef filter = {0};
    HAL_StatusTypeDef status;

    motor_can = hfdcan;
    m2006_simple_feedback = (M2006Simple_Feedback){0};
    m2006_simple_pi.integral = 0.0f;
	m2006_speed_filtered = 0.0f;
	speed_filter_initialized = 0;

    /* Accept all standard IDs into FIFO0; choose ID2 in Poll(). */
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0;
    filter.FilterID2 = 0;
    status = HAL_FDCAN_ConfigFilter(motor_can, &filter);
    if (status != HAL_OK) return status;

    status = HAL_FDCAN_ConfigGlobalFilter(motor_can,
        FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    if (status != HAL_OK) return status;

    return HAL_FDCAN_Start(motor_can);
}


/* Called for every received motor feedback frame. */
static void update_position(uint16_t encoder)
{
    static uint16_t last_encoder;
    static int64_t total_count = 0;
    int32_t delta;

    if (!position_initialized)
    {
        last_encoder = encoder;
        position_initialized = 1;
        return;
    }

    delta = (int32_t)encoder - (int32_t)last_encoder;

    /* Unwrap the rotor encoder across 0 / 8191. */
    if (delta > 4096)
        delta -= 8192;
    else if (delta < -4096)
        delta += 8192;

    total_count += delta;
    last_encoder = encoder;

    /* Rotor counts -> output-shaft degrees, reduction ratio 36:1. */
    m2006_position_deg =
        (float)total_count * (360.0f / (8192.0f * 36.0f));
}

HAL_StatusTypeDef M2006Simple_SetPositionDeg(float target_deg, float dt_s)
{
    float error;
    float target_rotor_rpm;
	float current;

    /* The first feedback establishes the position origin. */
    if (!position_initialized)
        return HAL_BUSY;

    error = target_deg - m2006_position_deg;
    target_rotor_rpm = m2006_position_kp * error;

    if (target_rotor_rpm > m2006_position_speed_limit)
        target_rotor_rpm = m2006_position_speed_limit;

    if (target_rotor_rpm < -m2006_position_speed_limit)
        target_rotor_rpm = -m2006_position_speed_limit;

    return M2006Simple_SetSpeedRotorRPM(target_rotor_rpm, dt_s,&current);
}

HAL_StatusTypeDef M2006Simple_Poll(void)
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[64];
    uint32_t count = HAL_FDCAN_GetRxFifoFillLevel(motor_can, FDCAN_RX_FIFO0);
    HAL_StatusTypeDef status;

    /* Fixed count: do not wait for another frame. */
    for (uint32_t i = 0; i < count; ++i)
    {
        status = HAL_FDCAN_GetRxMessage(motor_can, FDCAN_RX_FIFO0, &header, data);
        if (status != HAL_OK) return status;

        if (header.Identifier != 0x202 ||
            header.IdType != FDCAN_STANDARD_ID ||
            header.RxFrameType != FDCAN_DATA_FRAME ||
            header.FDFormat != FDCAN_CLASSIC_CAN ||
            header.DataLength != FDCAN_DLC_BYTES_8) continue;

        m2006_simple_feedback.encoder = ((uint16_t)data[0] << 8) | data[1];
        m2006_simple_feedback.rotor_rpm = read_i16(data + 2);
        m2006_simple_feedback.torque_raw = read_i16(data + 4);
        ++m2006_simple_feedback.rx_count;
	if (!speed_filter_initialized)
{
    /* Start from the first measured speed. */
    m2006_speed_filtered = (float)m2006_simple_feedback.rotor_rpm;
    speed_filter_initialized = 1;
}
else
{
    m2006_speed_filtered += m2006_speed_alpha *
        ((float)m2006_simple_feedback.rotor_rpm - m2006_speed_filtered);
}
		update_position(m2006_simple_feedback.encoder);
		
    }
    return HAL_OK;
}



HAL_StatusTypeDef M2006Simple_SetCurrent(int32_t current)
{
    FDCAN_TxHeaderTypeDef header = {0};
    uint8_t data[8] = {0};
    uint16_t packed;

    if (HAL_FDCAN_GetTxFifoFreeLevel(motor_can) == 0) return HAL_BUSY;

    if (current > M2006_SIMPLE_CURRENT_LIMIT) current = M2006_SIMPLE_CURRENT_LIMIT;
    if (current < -M2006_SIMPLE_CURRENT_LIMIT) current = -M2006_SIMPLE_CURRENT_LIMIT;
    packed = (uint16_t)(int16_t)current;
    data[2] = (uint8_t)(packed >> 8);
    data[3] = (uint8_t)packed;

    header.Identifier = 0x200;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    return HAL_FDCAN_AddMessageToTxFifoQ(motor_can, &header, data);
}

HAL_StatusTypeDef M2006Simple_SetSpeedRotorRPM(float target_rpm, float dt_s,float *current)
{
    float error = target_rpm - m2006_speed_filtered;
    //float current;

    m2006_simple_pi.integral += m2006_simple_pi.ki * error * dt_s;
    m2006_simple_pi.integral = clamp_current(m2006_simple_pi.integral);
    *current = m2006_simple_pi.kp * error + m2006_simple_pi.integral;
    *current = clamp_current(*current);

    return M2006Simple_SetCurrent((int32_t)(*current));
}

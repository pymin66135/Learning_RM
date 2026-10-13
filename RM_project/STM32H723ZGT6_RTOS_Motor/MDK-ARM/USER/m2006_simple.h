#ifndef M2006_SIMPLE_H
#define M2006_SIMPLE_H

#include "fdcan.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Single C610 ID 2: TX 0x200 bytes 2/3, RX 0x202. */
#define M2006_SIMPLE_CURRENT_LIMIT 10000

typedef struct {
    uint16_t encoder;       /* Rotor angle: 0..8191. */
    int16_t rotor_rpm;      /* Rotor rpm, NOT outputa-shaft rpm. */
    int16_t torque_raw;     /* Feedback bytes 4/5, no unit conversion. */
    uint32_t rx_count;
} M2006Simple_Feedback;

typedef struct {
    float kp;              /* Raw current command / rotor rpm. */
    float ki;              /* Raw current command / (rotor rpm * second). */
    float integral;        /* Integral contribution to current command. */
} M2006Simple_PI;

extern M2006Simple_Feedback m2006_simple_feedback;
extern M2006Simple_PI m2006_simple_pi;
extern float m2006_position_deg;
extern float m2006_position_kp;
extern float m2006_position_speed_limit;
extern float m2006_speed_filtered;
extern float m2006_speed_alpha;

HAL_StatusTypeDef M2006Simple_SetPositionDeg(float target_deg, float dt_s);
/* Call once after MX_FDCAN1_Init; configures filter and starts CAN. */
HAL_StatusTypeDef M2006Simple_Init(FDCAN_HandleTypeDef *hfdcan);
/* Drain frames present at entry. HAL_OK does not imply a matching frame. */
HAL_StatusTypeDef M2006Simple_Poll(void);
/* Raw command, clipped to +/-1000. HAL_OK means queued, HAL_BUSY means full. */
HAL_StatusTypeDef M2006Simple_SetCurrent(int32_t current);
/* Poll first; call every control period. dt_s is in seconds, e.g. 0.001f. */
HAL_StatusTypeDef M2006Simple_SetSpeedRotorRPM(float target_rpm, float dt_s,float *current);

/* All calls and access to these globals belong to ONE task. */
/* No online check, enable state, feedback timeout, or automatic stop. */

#ifdef __cplusplus
}
#endif
#endif

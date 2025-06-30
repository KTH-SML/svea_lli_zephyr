#include "common.h"

#define SERVO_NODE DT_NODELABEL(steeringservo)
const struct pwm_dt_spec steering_pwm = PWM_DT_SPEC_GET(SERVO_NODE);
const uint32_t steering_pwm_min_ns = DT_PROP(SERVO_NODE, min_pulse);
const uint32_t steering_pwm_max_ns = DT_PROP(SERVO_NODE, max_pulse);

bool override_mode = true;
float remote_steering_norm_value = 0.0f;
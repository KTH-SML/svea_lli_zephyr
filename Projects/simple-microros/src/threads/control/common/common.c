#include "common.h"

// Steering servo
#define STEERING_SERVO_NODE DT_NODELABEL(steeringservo)
const struct pwm_dt_spec steering_pwm = PWM_DT_SPEC_GET(STEERING_SERVO_NODE);
const uint32_t steering_pwm_min_ns = DT_PROP(STEERING_SERVO_NODE, min_pulse);
const uint32_t steering_pwm_max_ns = DT_PROP(STEERING_SERVO_NODE, max_pulse);

// Differential servo
#define DIFF_SERVO_NODE DT_NODELABEL(diffservo)
const struct pwm_dt_spec diff_pwm = PWM_DT_SPEC_GET(DIFF_SERVO_NODE);
const uint32_t diff_pwm_min_ns = DT_PROP(DIFF_SERVO_NODE, min_pulse);
const uint32_t diff_pwm_max_ns = DT_PROP(DIFF_SERVO_NODE, max_pulse);

// Gear servo
#define GEAR_SERVO_NODE DT_NODELABEL(gearservo)
const struct pwm_dt_spec gear_pwm = PWM_DT_SPEC_GET(GEAR_SERVO_NODE);
const uint32_t gear_pwm_min_ns = DT_PROP(GEAR_SERVO_NODE, min_pulse);
const uint32_t gear_pwm_max_ns = DT_PROP(GEAR_SERVO_NODE, max_pulse);

// Throttle ESC
#define THROTTLE_ESC_NODE DT_NODELABEL(throttleesc)
const struct pwm_dt_spec throttle_pwm = PWM_DT_SPEC_GET(THROTTLE_ESC_NODE);
const uint32_t throttle_pwm_min_ns = DT_PROP(THROTTLE_ESC_NODE, min_pulse);
const uint32_t throttle_pwm_max_ns = DT_PROP(THROTTLE_ESC_NODE, max_pulse);

bool override_mode = true;
float remote_steering_norm_value = 0.0f;
float remote_diff_norm_value = 0.0f;
float remote_gear_norm_value = 0.0f;
float remote_esc_norm_value = 0.0f;
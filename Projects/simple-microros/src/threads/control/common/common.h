// Sorry future person this is definetely not the most efficient or abstracted way of doing it,
// but it is the easiest to understand. the override mode is set by pwm channel from remote
// that flips if statements in the ros callbacks for the steering and throttle and also flips ifs in the pwm publishers

// This can easily be made abstract, but in the name of making it easy to copy paste functions
// and make it easy to change stuff without breaking someting else, i think this is ok

#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/drivers/pwm.h>
extern bool override_mode;

// Steering PWM
extern const struct pwm_dt_spec steering_pwm;
extern const uint32_t steering_pwm_min_ns;
extern const uint32_t steering_pwm_max_ns;
extern float remote_steering_norm_value;

// Differential PWM
extern const struct pwm_dt_spec diff_pwm;
extern const uint32_t diff_pwm_min_ns;
extern const uint32_t diff_pwm_max_ns;
extern float remote_diff_norm_value;

// Gear PWM
extern const struct pwm_dt_spec gear_pwm;

// Throttle PWM
extern const struct pwm_dt_spec throttle_pwm;

// Thread priority for PWM input thread
#define REMOTE_PWM_THREAD_PRIORITY 4

#define REMOTE_PWM_MIN_US 50
#define REMOTE_PWM_MAX_US 250

extern float remote_gear_norm_value;
extern float remote_esc_norm_value;
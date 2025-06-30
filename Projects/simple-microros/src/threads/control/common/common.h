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
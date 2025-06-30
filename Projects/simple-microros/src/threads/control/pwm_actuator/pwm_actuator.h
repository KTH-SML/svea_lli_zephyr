#pragma once
#include <zephyr/drivers/pwm.h>

// Set PWM pulse for a given Zephyr pwm_dt_spec and normalized value [-1, 1]
void set_pwm_norm(const struct pwm_dt_spec *pwm, float norm, uint32_t min_ns, uint32_t max_ns);
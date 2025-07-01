#include "pwm.h"
#include <stdio.h>

void set_pwm_norm(const struct pwm_dt_spec *pwm, float norm, uint32_t min_ns, uint32_t max_ns) {
    if (norm > 1.0f)
        norm = 1.0f;
    if (norm < -1.0f)
        norm = -1.0f;
    uint32_t pulse = (uint32_t)((norm + 1.f) * 0.5f * (max_ns - min_ns) + min_ns);
    printf("pwm_actuator: Calculated pulse: %u ns (min: %u, max: %u)\n", pulse, min_ns, max_ns);

    int ret = pwm_set_pulse_dt(pwm, pulse);
    if (ret) {
        printf("pwm_actuator: PWM set failed: %d\n", ret);
    } else {
        printf("pwm_actuator: PWM set OK\n");
    }
}
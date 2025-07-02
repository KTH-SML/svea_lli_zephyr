#include "pwm.h"
#include <stdio.h>

void set_pwm_norm(const struct pwm_dt_spec *pwm,
                  float norm,
                  uint32_t min_ns,
                  uint32_t max_ns) {
    if (rc_remote_disconnected) {
        printf("pwm_actuator: RC remote disconnected — setting null PWM signal\n");
        int ret = pwm_set_pulse_dt(pwm, 0);
        if (ret)
            printf("pwm_actuator: Failed to set null PWM: %d\n", ret);
        return;
    }

    if (norm > 1.0f)
        norm = 1.0f;
    if (norm < -1.0f)
        norm = -1.0f;

    uint32_t pulse_ns = (uint32_t)((norm + 1.f) * 0.5f * (max_ns - min_ns) + min_ns);
    printf("pwm_actuator: Setting PWM pulse to %u ns (norm %.2f)\n", pulse_ns, norm);
    int ret = pwm_set_pulse_dt(pwm, pulse_ns);

    if (ret)
        printf("pwm_actuator: pwm_set_pulse_dt failed: %d\n", ret);
}

/* Pass‑through helper used by RC override -------------------------------- */
void set_pwm_pulse_us(const struct pwm_dt_spec *pwm, uint32_t pulse_us) {
    if (rc_remote_disconnected) {
        printf("pwm_actuator: RC remote disconnected — setting null PWM signal\n");
        int ret = pwm_set_pulse_dt(pwm, 0);
        if (ret)
            printf("pwm_actuator: Failed to set null PWM: %d\n", ret);
        return;
    }

    if (pulse_us < REMOTE_PWM_CLIP_MIN_US)
        pulse_us = REMOTE_PWM_CLIP_MIN_US;
    if (pulse_us > REMOTE_PWM_CLIP_MAX_US)
        pulse_us = REMOTE_PWM_CLIP_MAX_US;
    printf("pwm_actuator: Setting PWM pulse to %u µs\n", pulse_us);
    int ret = pwm_set_pulse_dt(pwm, pulse_us * 1000u); /* µs → ns */

    if (ret)
        printf("pwm_actuator: pwm_set_pulse_dt failed: %d\n", ret);
}
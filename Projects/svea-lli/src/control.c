/* src/control.c  –  fast, non-blocking servo output -------------------- */
#include "control.h"
#include "rc_input.h" /* rc_get_pulse_us() prototype        */

#include <stdlib.h>       // <-- Add this line
#include <stm32_ll_tim.h> /* low-level TIM helpers               */
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h> // Make sure this is included
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <stm32_ll_tim.h> /* already included, but make sure */

LOG_MODULE_REGISTER(control, LOG_LEVEL_INF);

/* ───── 1. servo bookkeeping (unchanged) ──────────────────────────────── */
bool servos_initialized = false;
bool remote_connected = false;

servo_t servos[SERVO_COUNT] = {
    [SERVO_STEERING] = {.spec = PWM_DT_SPEC_GET(DT_NODELABEL(steeringservo))},
    [SERVO_GEAR] = {.spec = PWM_DT_SPEC_GET(DT_NODELABEL(gearservo))},
    [SERVO_THROTTLE] = {.spec = PWM_DT_SPEC_GET(DT_NODELABEL(throttleesc))},
    [SERVO_DIFF] = {.spec = PWM_DT_SPEC_GET(DT_NODELABEL(diffservo))},
    [SERVO_DIFF_REAR] = {.spec = PWM_DT_SPEC_GET(DT_NODELABEL(diffservorear))},
};

void servo_init(void) {
    /* Let the Zephyr driver set up every timer & pin exactly once */
    for (int i = 0; i < SERVO_COUNT; ++i) {
        pwm_set_cycles(servos[i].spec.dev,
                       servos[i].spec.channel,
                       servos[i].spec.period,
                       0,
                       0);
    }

    /* Now kill preload so future raw writes land immediately */
    // for (int i = 0; i < SERVO_COUNT; ++i) {
    //     servo_raw_init_once(&servos[i].spec);
    // }

    servos_initialized = true;
}
static inline void servo_set_ticks(const struct pwm_dt_spec *s, uint32_t t_us) {
    uint32_t t_ns = t_us * 1000; // Convert microseconds to nanoseconds
    pwm_set_pulse_dt(s, t_ns);
}
/* ───── 3. control thread – barebones, no filtering ─────────────────── */
#define LOOP_MS 20

static void control_thread(void *, void *, void *) {
    while (!servos_initialized) {
        k_sleep(K_MSEC(10));
    }

    LOG_INF("Control thread started, servos initialized.");

    for (;;) {
        uint32_t steer = rc_get_pulse_us(RC_STEER);
        uint32_t thr = rc_get_pulse_us(RC_THROTTLE);
        uint32_t gear = rc_get_pulse_us(RC_HIGH_GEAR);

        servo_set_ticks(&servos[SERVO_STEERING].spec, steer);
        servo_set_ticks(&servos[SERVO_THROTTLE].spec, thr);
        servo_set_ticks(&servos[SERVO_GEAR].spec, gear);

        k_sleep(K_MSEC(LOOP_MS));
    }
}

K_THREAD_DEFINE(control_tid, 2048, control_thread, NULL, NULL, NULL,
                2 /* prio */, 0, 0);

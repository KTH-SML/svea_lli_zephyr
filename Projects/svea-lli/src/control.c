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
bool remote_connected = true;
bool in_override_mode = true;

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
    servos_initialized = true;
}
static inline void servo_set_ticks(const struct pwm_dt_spec *s, uint32_t t_us) {
    uint32_t t_ns = t_us * 1000; // Convert microseconds to nanoseconds
    pwm_set_pulse_dt(s, t_ns);
}
/* ───── 3. control thread – barebones, no filtering ─────────────────── */
#define OVERRIDE_AGE_DISCONNECT_US 15000
#define RECONNECT_WINDOW_MS 500
#define LOOP_MS 25 // 400 Hz control loop

// Remove old pulse_to_us, use new int8 mapping
static inline uint32_t int8_to_us(int8_t val) {
    // Map [-127,127] to [1000,2000]us, 0 = 1500us
    return 1500 + ((int32_t)val * 500) / 127;
}

static void control_thread(void *, void *, void *) {
    while (!servos_initialized) {
        k_sleep(K_MSEC(10));
    }

    LOG_INF("Control thread started, servos initialized.");

    int log_counter = 0;
    int reconnect_counter = 0;
    int reconnect_samples = RECONNECT_WINDOW_MS / LOOP_MS;

    for (;;) {
        uint64_t loop_start_us = k_ticks_to_us_floor64(k_uptime_ticks());

        uint32_t override_age = rc_get_age_us(RC_OVERRIDE);

        uint32_t steer = 0;
        uint32_t thr = 0;
        uint32_t gear = 0;
        uint32_t override = 0;
        uint32_t diff = 0;
        uint32_t diff_rear = 0;
        // Disconnect if override age exceeds threshold
        if (override_age > OVERRIDE_AGE_DISCONNECT_US) {
            remote_connected = false;
            reconnect_counter = 0; // reset reconnect averaging
        } else {
            // If currently disconnected, check for reconnection
            if (!remote_connected) {
                reconnect_counter++;
                if (reconnect_counter >= reconnect_samples) {
                    // If we've seen reconnect_samples consecutive "good" ages, reconnect
                    remote_connected = true;
                    reconnect_counter = 0;
                }
            }
        }

        if (remote_connected) {
            in_override_mode = rc_get_pulse_us(RC_OVERRIDE) > 1500;
            if (in_override_mode) {
                steer = rc_get_pulse_us(RC_STEER);
                thr = rc_get_pulse_us(RC_THROTTLE);
                gear = rc_get_pulse_us(RC_HIGH_GEAR);

            } else {
                // Use signed int8 from g_ros_ctrl, map to us
                steer = int8_to_us(g_ros_ctrl.steering);
                thr = int8_to_us(g_ros_ctrl.throttle);
                gear = g_ros_ctrl.high_gear ? 2000 : 1000;
                diff = g_ros_ctrl.diff ? 2000 : 1000;
                diff_rear = g_ros_ctrl.diff ? 1000 : 2000;

                // LOG_INF("Using fallback control: steer %d, throttle %d, gear %d, diff %d",
                //         g_ros_ctrl.steering, g_ros_ctrl.throttle,
                //         g_ros_ctrl.high_gear, g_ros_ctrl.diff);
            }
        }

        servo_set_ticks(&servos[SERVO_STEERING].spec, steer);
        servo_set_ticks(&servos[SERVO_THROTTLE].spec, thr);
        servo_set_ticks(&servos[SERVO_GEAR].spec, gear);
        servo_set_ticks(&servos[SERVO_DIFF].spec, diff);
        servo_set_ticks(&servos[SERVO_DIFF_REAR].spec, diff_rear);

        uint64_t elapsed_us = k_ticks_to_us_floor64(k_uptime_ticks()) - loop_start_us;
        int32_t sleep_time = LOOP_MS * 1000 - elapsed_us;
        if (log_counter++ >= 25) { // Log every 500ms
            // LOG_DBG("steer %u us  throttle %u us  gear %u us  override %u us  override_age %u us  elapsed %llu us  remote_connected %d",
            //         steer, thr, gear, override, override_age, elapsed_us, remote_connected);
            log_counter = 0;
        }
        if (sleep_time > 0) {
            k_sleep(K_USEC(sleep_time));
        }
    }
}

K_THREAD_DEFINE(control_tid, 2048, control_thread, NULL, NULL, NULL,
                3 /* prio */, 0, 0);

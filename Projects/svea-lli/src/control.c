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
bool forward_guess = true; // Guess forward direction
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
static inline void servo_set_ticks(const struct pwm_dt_spec *s, uint16_t t_us) {
    // Clamp t_us to [1000, 2000] (1-2 ms)
    // if (t_us == 0) {
    //     pwm_set_pulse_dt(s, 0);
    //     return;
    // }
    // if (t_us < 1000)
    //     t_us = 1000;
    // if (t_us > 2000)
    //     t_us = 2000;
    uint32_t t_ns = t_us * 1000; // Convert microseconds to nanoseconds
    pwm_set_pulse_dt(s, t_ns);
}
/* ───── 3. control thread – barebones, no filtering ─────────────────── */
#define OVERRIDE_AGE_DISCONNECT_US 15000
#define RECONNECT_WINDOW_MS 500
#define LOOP_MS 25                          // 400 Hz control loop
#define ROS_THROTTLE_AGE_TIMEOUT_US 1000000 // 1 seconds max age for ROS throttle command

#define ACCELERATION_CLAMP_LIMIT_US 1000 // 1 ms max change per loop

// Throttle ramp: time (ms) to go from 1000us to 1500us (500us change)
#define THROTTLE_RAMP_DELTA_US 500
#define THROTTLE_RAMP_TIME_MS 500

// New: Deceleration clamp (towards 1500us)
#define THROTTLE_DECEL_DELTA_US 500
#define THROTTLE_DECEL_TIME_MS 200

// Remove old pulse_to_us, use new int8 mapping
static inline uint32_t int8_to_us(int8_t val) {
    // Map [-127,127] to [1000,2000]us, 0 = 1500us
    return 1500 + ((int32_t)val * 500) / 127;
}

static inline int32_t clamp_delta(int32_t value, int32_t prev, int32_t max_step) {
    int32_t delta = value - prev;
    if (delta > max_step)
        return prev + max_step;
    if (delta < -max_step)
        return prev - max_step;
    return value;
}

static inline int32_t clamp_throttle(int32_t value, int32_t prev, int32_t max_accel, int32_t max_decel) {
    int32_t delta = value - prev;
    // Determine if moving away from 1500 (acceleration) or towards 1500 (deceleration)
    int32_t prev_dist = abs(prev - 1500);
    int32_t value_dist = abs(value - 1500);

    if (value_dist > prev_dist) {
        // Acceleration (moving away from 1500)
        if (delta > max_accel)
            return prev + max_accel;
        if (delta < -max_accel)
            return prev - max_accel;
    } else {
        // Deceleration (moving towards 1500)
        if (delta > max_decel)
            return prev + max_decel;
        if (delta < -max_decel)
            return prev - max_decel;
    }
    return value;
}

// Place these static variables at file scope (top of file, after global variables)
static uint64_t forward_start_us = 0;
static uint64_t reverse_start_us = 0;

static void update_forward_guess(uint32_t thr) {
    const int threshold_us = 50;             // 50 us away from neutral
    const uint64_t required_time_us = 10000; // 10 ms required in one direction

    uint64_t now_us = k_ticks_to_us_floor64(k_uptime_ticks());

    if (thr > 1500 + threshold_us) {
        // Going forward
        if (forward_start_us == 0) {
            forward_start_us = now_us;
        }
        reverse_start_us = 0;
        if ((now_us - forward_start_us) >= required_time_us) {
            forward_guess = true;
        }
    } else if (thr < 1500 - threshold_us) {
        // Going reverse
        if (reverse_start_us == 0) {
            reverse_start_us = now_us;
        }
        forward_start_us = 0;
        if ((now_us - reverse_start_us) >= required_time_us) {
            forward_guess = false;
        }
    } else {
        // Near neutral, reset both timers
        forward_start_us = 0;
        reverse_start_us = 0;
    }
}

static void control_thread(void *, void *, void *) {
    while (!servos_initialized) {
        k_sleep(K_MSEC(10));
    }

    LOG_INF("Control thread started, servos initialized.");

    int log_counter = 0;
    int reconnect_counter = 0;
    int reconnect_samples = RECONNECT_WINDOW_MS / LOOP_MS;

    uint32_t prev_thr = 1500; // Start at neutral
    uint32_t diff = 1100;
    uint32_t diff_rear = 1900;

    g_ros_ctrl.diff = true;

    // Calculate max allowed step per loop
    const int32_t max_thr_accel = (THROTTLE_RAMP_DELTA_US * LOOP_MS) / THROTTLE_RAMP_TIME_MS;
    const int32_t max_thr_decel = (THROTTLE_DECEL_DELTA_US * LOOP_MS) / THROTTLE_DECEL_TIME_MS;

    for (;;) {
        uint64_t loop_start_us = k_ticks_to_us_floor64(k_uptime_ticks());

        uint32_t override_age = rc_get_age_us(RC_OVERRIDE);

        uint32_t steer = 0;
        uint32_t thr = 1500;
        uint32_t gear = 0;
        uint32_t override = 0;

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
        in_override_mode = rc_get_pulse_us(RC_OVERRIDE) > 1500;

        if (remote_connected) {
            if (in_override_mode) {
                steer = rc_get_pulse_us(RC_STEER);
                thr = rc_get_pulse_us(RC_THROTTLE);
                gear = (rc_get_pulse_us(RC_HIGH_GEAR) > 1500) ? 1100 : 1900;

                // Update g_ros_ctrl with current remote values
                g_ros_ctrl.steering = (int8_t)((int32_t)steer - 1500) * 127 / 500;
                g_ros_ctrl.throttle = (int8_t)((int32_t)thr - 1500) * 127 / 500;
                g_ros_ctrl.high_gear = (gear > 1500);
            } else {
                // Use signed int8 from g_ros_ctrl, map to us
                steer = int8_to_us(g_ros_ctrl.steering);
                thr = int8_to_us(g_ros_ctrl.throttle);
                gear = g_ros_ctrl.high_gear ? 1900 : 1100;
                diff = g_ros_ctrl.diff ? 2000 : 1000;
                diff_rear = g_ros_ctrl.diff ? 1000 : 2000;
            }

            // Double ramp rates if gear is high
            int32_t accel = max_thr_accel;
            int32_t decel = max_thr_decel;
            if (gear < 1500) { // When in high gear, decrease ramp rates
                accel /= 2;
                decel /= 1;
            }

            thr = clamp_throttle(thr, prev_thr, accel, decel);
            prev_thr = thr;

            // if (log_counter++ >= 25) { // Log every 500ms
            //     LOG_INF("steer %u us  throttle %u us  gear %u us  override %u us  override_age %u us remote_connected %d  max_thr_accel %d  max_thr_decel %d",
            //             steer, thr, gear, override, override_age, remote_connected, accel, decel);
            //     log_counter = 0;
            // }
            update_forward_guess(thr);
        }

        servo_set_ticks(&servos[SERVO_STEERING].spec, steer);
        servo_set_ticks(&servos[SERVO_THROTTLE].spec, thr);
        servo_set_ticks(&servos[SERVO_GEAR].spec, gear);
        servo_set_ticks(&servos[SERVO_DIFF].spec, diff);
        servo_set_ticks(&servos[SERVO_DIFF_REAR].spec, diff_rear);

        uint64_t elapsed_us = k_ticks_to_us_floor64(k_uptime_ticks()) - loop_start_us;
        int32_t sleep_time = LOOP_MS * 1000 - elapsed_us;
        if (sleep_time > 0) {
            k_sleep(K_USEC(sleep_time));
        }
    }
}

K_THREAD_DEFINE(control_tid, 2048, control_thread, NULL, NULL, NULL, 3 /* prio */, 0, 0);

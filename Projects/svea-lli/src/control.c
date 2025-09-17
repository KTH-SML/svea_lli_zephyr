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
        /* Initialize with 0 ns pulse to avoid 16-bit overflow warnings */
        pwm_set_pulse_dt(&servos[i].spec, 0);
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
/* ───── 3. control thread – readable, explicit conditions ───────────── */
#define LOOP_MS 25 // 40 Hz

// Neutral/safe outputs
#define SERVO_NEUTRAL_US 1500
#define GEAR_HIGH_US 1100
#define GEAR_LOW_US 1900

// Throttle ramp profile, prevents overcurrent trips but shouldn't be noticable
#define THROTTLE_RAMP_DELTA_US 500  // change to consider (us)
#define THROTTLE_RAMP_TIME_MS 100   // time to make that change (ms)
#define THROTTLE_DECEL_DELTA_US 500 // decel towards neutral
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

static void control_thread(void *, void *, void *) {
    while (!servos_initialized) {
        k_sleep(K_MSEC(10));
    }

    LOG_INF("Control thread started, servos initialized.");

    uint32_t prev_thr = SERVO_NEUTRAL_US; // Start at neutral
    uint32_t diff = 1000;                 // default diff positions
    uint32_t diff_rear = 2000;
    g_ros_ctrl.diff = true;

    // Per-loop ramp limits (us per loop)
    const int32_t max_thr_accel = (THROTTLE_RAMP_DELTA_US * LOOP_MS) / THROTTLE_RAMP_TIME_MS;
    const int32_t max_thr_decel = (THROTTLE_DECEL_DELTA_US * LOOP_MS) / THROTTLE_DECEL_TIME_MS;

    for (;;) {
        const uint64_t loop_start_us = k_ticks_to_us_floor64(k_uptime_ticks());

        // 1) Connection + override state (three-way)
        remote_connected = rc_input_connected();
        rc_override_mode_t override_mode = rc_get_override_mode();
        in_override_mode = (override_mode != RC_OVERRIDE_ROS); // for ros publish

        // 2) Decide command sources and compute targets
        uint32_t steer = SERVO_NEUTRAL_US;
        uint32_t thr = SERVO_NEUTRAL_US;
        uint32_t gear = GEAR_LOW_US;
        bool mute_outputs = false;

        if (!remote_connected) {
            // Disconnected: null all primary outputs
            steer = SERVO_NEUTRAL_US;
            thr = SERVO_NEUTRAL_US;
            gear = GEAR_LOW_US;
        } else {
            switch (override_mode) {
            case RC_OVERRIDE_REMOTE:
                // Manual override passthrough
                steer = rc_get_pulse_us(RC_STEER);
                thr = rc_get_pulse_us(RC_THROTTLE);
                gear = (rc_get_pulse_us(RC_HIGH_GEAR) > SERVO_NEUTRAL_US) ? GEAR_HIGH_US : GEAR_LOW_US;
                // Update ROS shadow for visibility/tools
                g_ros_ctrl.steering = (int8_t)(((int32_t)steer - SERVO_NEUTRAL_US) * 127 / 500);
                g_ros_ctrl.throttle = (int8_t)(((int32_t)thr - SERVO_NEUTRAL_US) * 127 / 500);
                g_ros_ctrl.high_gear = (gear == GEAR_HIGH_US);
                break;
            case RC_OVERRIDE_MUTE:
                // Override engaged, but mute all outputs
                mute_outputs = true;
                break;
            case RC_OVERRIDE_ROS:
            default:
                // ROS control
                steer = int8_to_us(g_ros_ctrl.steering);
                thr = int8_to_us(g_ros_ctrl.throttle);
                gear = g_ros_ctrl.high_gear ? GEAR_LOW_US : GEAR_HIGH_US; // keep legacy polarity
                diff = g_ros_ctrl.diff ? 2000 : 1000;
                diff_rear = g_ros_ctrl.diff ? 1000 : 2000;
                break;
            }
        }

        // 3) Apply throttle ramping (accel/decel) unless muted
        if (!mute_outputs) {
            int32_t accel = max_thr_accel;
            int32_t decel = max_thr_decel;
            if (gear == GEAR_HIGH_US) {
                // In high gear, be gentler on accel
                accel /= 2;
            }
            thr = clamp_throttle(thr, prev_thr, accel, decel);
            prev_thr = thr;
            forward_guess = thr > 1450; // simple heuristic
        } else {
            // Keep prev_thr as-is so ramp resumes smoothly after unmute
            steer = 0;
            thr = 0;
            gear = 0;
            diff = 0;
            diff_rear = 0;
        }

        // 4) Write outputs
        servo_set_ticks(&servos[SERVO_STEERING].spec, steer);
        servo_set_ticks(&servos[SERVO_THROTTLE].spec, thr);
        servo_set_ticks(&servos[SERVO_GEAR].spec, gear);
        servo_set_ticks(&servos[SERVO_DIFF].spec, diff);
        servo_set_ticks(&servos[SERVO_DIFF_REAR].spec, diff_rear);

        // 5) Keep the loop period
        const uint64_t elapsed_us = k_ticks_to_us_floor64(k_uptime_ticks()) - loop_start_us;
        const int32_t sleep_time = (int32_t)(LOOP_MS * 1000) - (int32_t)elapsed_us;
        if (sleep_time > 0) {
            k_sleep(K_USEC(sleep_time));
        }
    }
}

K_THREAD_DEFINE(control_tid, 2048, control_thread, NULL, NULL, NULL, 3 /* prio */, 0, 0);

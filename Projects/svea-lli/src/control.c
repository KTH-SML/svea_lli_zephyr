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
bool forward_guess = true;     // Guess forward direction
static bool diff_state = true; // persistent diff state (true = engaged)
static inline bool diff_next_state(bool current) {
    if (rc_consume_diff_toggle_event()) {
        bool next = !current;
        g_ros_ctrl.diff = next; // keep ROS shadow in sync
        LOG_INF("Diff toggled to %s", next ? "ENGAGED" : "DISENGAGED");
        return next;
    }
    return current;
}
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
        if (!device_is_ready(servos[i].spec.dev)) {
            LOG_ERR("Servo %d PWM device not ready");
        }
    }
    servos_initialized = true;
}
static inline void servo_set_ticks(const struct pwm_dt_spec *s, uint16_t t_us) {
    uint32_t t_ns = t_us * 1000; // Convert microseconds to nanoseconds

    int err = pwm_set_pulse_dt(s, t_ns);
    if (err) {
        LOG_ERR("PWM set failed: %d", err);
    }
}
/* ───── 3. control thread – readable, explicit conditions ───────────── */
#define LOOP_MS 25 // 40 Hz

// Neutral/safe outputs
#define SERVO_NEUTRAL_US 1500

// Gear positions
#define GEAR_HIGH_US 1100
#define GEAR_LOW_US 1900

// Diff positions (front/rear) when engaged vs. open
#define DIFF_ENGAGED_FRONT_US 1900
#define DIFF_ENGAGED_REAR_US 1100
#define DIFF_OPEN_FRONT_US 1100
#define DIFF_OPEN_REAR_US 1900

// Throttle ramp profile, prevents overcurrent trips but shouldn't be noticable
#define THROTTLE_RAMP_DELTA_US 600  // change to consider (us)
#define THROTTLE_RAMP_TIME_MS 200   // time to make that change (ms)
#define THROTTLE_DECEL_DELTA_US 500 // decel towards neutral
#define THROTTLE_DECEL_TIME_MS 600  // time to make that change (ms)

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
        // Deceleration
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
    g_ros_ctrl.diff = diff_state;

    // Per-loop ramp limits (us per loop)
    const int32_t max_thr_accel = (THROTTLE_RAMP_DELTA_US * LOOP_MS) / THROTTLE_RAMP_TIME_MS;
    const int32_t max_thr_decel = (THROTTLE_DECEL_DELTA_US * LOOP_MS) / THROTTLE_DECEL_TIME_MS;

    for (;;) {
        const uint64_t loop_start_us = k_ticks_to_us_floor64(k_uptime_ticks());

        // 1) Connection + override state (three-way)
        remote_connected = rc_input_connected();
        // TODO: ros_connected
        rc_override_mode_t override_mode = rc_get_override_mode();
        in_override_mode = (override_mode != RC_OVERRIDE_ROS); // for ros publish

        // 2) Decide command sources
        // - Compute booleans first (like gear), then map to pulses together
        uint32_t steer_us = SERVO_NEUTRAL_US;
        uint32_t thr_us = SERVO_NEUTRAL_US;
        bool high_gear = false;
        bool diff_engaged = diff_state; // start from persisted state
        uint32_t gear_us = GEAR_LOW_US;
        uint32_t diff_front_us = DIFF_OPEN_FRONT_US;
        uint32_t diff_rear_us = DIFF_OPEN_REAR_US;
        bool mute_outputs = false;

        if (!remote_connected) {
            // Disconnected: null all primary outputs
            steer_us = SERVO_NEUTRAL_US;
            thr_us = SERVO_NEUTRAL_US;
            high_gear = false;
            diff_engaged = false;
        } else {
            switch (override_mode) {
            case RC_OVERRIDE_REMOTE:
                // Manual override passthrough
                steer_us = rc_get_pulse_us(RC_STEER);
                thr_us = rc_get_pulse_us(RC_THROTTLE);
                high_gear = (rc_get_pulse_us(RC_HIGH_GEAR) > SERVO_NEUTRAL_US);
                // Update ROS shadow for better transition in case ros agent is off
                g_ros_ctrl.steering = (int8_t)(((int32_t)steer_us - SERVO_NEUTRAL_US) * 127 / 500);
                g_ros_ctrl.throttle = (int8_t)(((int32_t)thr_us - SERVO_NEUTRAL_US) * 127 / 500);
                g_ros_ctrl.high_gear = high_gear;
                // Diff next state from latched RC event
                diff_state = diff_next_state(diff_state);
                diff_engaged = diff_state;

                gear_us = high_gear ? GEAR_HIGH_US : GEAR_LOW_US;
                if (diff_engaged) {
                    diff_front_us = DIFF_ENGAGED_FRONT_US;
                    diff_rear_us = DIFF_ENGAGED_REAR_US;
                } else {
                    diff_front_us = DIFF_OPEN_FRONT_US;
                    diff_rear_us = DIFF_OPEN_REAR_US;
                }

                break;
            case RC_OVERRIDE_MUTE:
                // Override engaged, but mute all outputs
                mute_outputs = true;
                // Allow diff toggle while muted so it takes effect when unmuted
                diff_state = diff_next_state(diff_state);
                diff_engaged = diff_state;

                steer_us = 0;
                thr_us = 0;
                gear_us = 0;
                diff_front_us = 0;
                diff_rear_us = 0;

                break;
            case RC_OVERRIDE_ROS:
                // TODO ADD CHECK IF ROS IS ACTIVE, OTHERWISE FALLBACK TO MANUAL
                // ROS control
                steer_us = int8_to_us(g_ros_ctrl.steering);
                thr_us = int8_to_us(g_ros_ctrl.throttle);
                high_gear = g_ros_ctrl.high_gear;

                gear_us = high_gear ? GEAR_HIGH_US : GEAR_LOW_US;

                // Track ROS diff state when in ROS mode
                diff_state = g_ros_ctrl.diff;
                // Also allow CH4 to toggle diff even in ROS mode
                diff_state = diff_next_state(diff_state);
                diff_engaged = diff_state;

                if (diff_engaged) {
                    diff_front_us = DIFF_ENGAGED_FRONT_US;
                    diff_rear_us = DIFF_ENGAGED_REAR_US;
                } else {
                    diff_front_us = DIFF_OPEN_FRONT_US;
                    diff_rear_us = DIFF_OPEN_REAR_US;
                }
                break;
            default:
                // Use defaults
                break;
            }
        }

        // 3) Apply throttle ramping (accel/decel) unless muted
        if (!mute_outputs) {
            int32_t accel = max_thr_accel;
            int32_t decel = max_thr_decel;
            if (gear_us == GEAR_HIGH_US) {
                // In high gear, be gentler on accel
                accel /= 2;
            }
            thr_us = clamp_throttle(thr_us, prev_thr, accel, decel);
            prev_thr = thr_us;
            forward_guess = thr_us > 1450; // simple heuristic
        }

        // 4) Write outputs
        servo_set_ticks(&servos[SERVO_STEERING].spec, steer_us);
        servo_set_ticks(&servos[SERVO_THROTTLE].spec, thr_us);
        servo_set_ticks(&servos[SERVO_GEAR].spec, gear_us);
        servo_set_ticks(&servos[SERVO_DIFF].spec, diff_front_us);
        servo_set_ticks(&servos[SERVO_DIFF_REAR].spec, diff_rear_us);

        // 5) Keep the loop period
        const uint64_t elapsed_us = k_ticks_to_us_floor64(k_uptime_ticks()) - loop_start_us;
        const int32_t sleep_time = (int32_t)(LOOP_MS * 1000) - (int32_t)elapsed_us;
        if (sleep_time > 0) {
            k_sleep(K_USEC(sleep_time));
        }
    }
}

K_THREAD_DEFINE(control_tid, 2048, control_thread, NULL, NULL, NULL, 3 /* prio */, 0, 0);

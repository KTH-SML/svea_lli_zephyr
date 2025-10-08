/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * SVEA Note.
 *
 * Part of the SVEA Low‑Level Interface (Zephyr) application.
 * Author: Nils Kiefer
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
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

// Throttle smoothing parameters for first-order filter
#define THROTTLE_FILTER_ACCEL_TAU_MS 220      // base smoothing time constant (ms)
#define THROTTLE_FILTER_STANDSTILL_TAU_MS 450 // slower response when leaving neutral (ms)
#define THROTTLE_FILTER_DECEL_TAU_MS 120      // quicker response when reducing throttle (ms)
#define THROTTLE_FILTER_SOFT_ZONE_US 50       // +/- range around neutral treated as standstill
#define THROTTLE_FILTER_HIGH_GEAR_SCALE 1.8f  // smoothing multiplier when high gear engaged

// Remove old pulse_to_us, use new int8 mapping
static inline uint32_t int8_to_us(int8_t val) {
    // Map [-127,127] to [1000,2000]us, 0 = 1500us
    return 1500 + ((int32_t)val * 500) / 127;
}

static inline uint32_t int8_to_throttle_us(int8_t val) {
    // Positive values command forward, map to lower pulse widths
    return SERVO_NEUTRAL_US - ((int32_t)val * 500) / 127;
}

static void control_thread(void *, void *, void *) {
    while (!servos_initialized) {
        k_sleep(K_MSEC(10));
    }

    LOG_INF("Control thread started, servos initialized.");

    float filtered_thr = (float)SERVO_NEUTRAL_US;
    g_ros_ctrl.diff = diff_state;

    uint64_t prev_loop_us = k_ticks_to_us_floor64(k_uptime_ticks());

    for (;;) {
        const uint64_t loop_start_us = k_ticks_to_us_floor64(k_uptime_ticks());
        uint64_t dt_us = loop_start_us - prev_loop_us;
        if (dt_us == 0U) {
            dt_us = LOOP_MS * 1000ULL;
        }

        // 1) Connection + override state (three-way)
        remote_connected = rc_input_connected();
        // TODO: ros_connected
        rc_override_mode_t override_mode = rc_get_override_mode();

        // 2) Decide command sources
        // - Compute booleans first (like gear), then map to pulses together
        uint32_t steer_us = SERVO_NEUTRAL_US;
        uint32_t thr_target_us = SERVO_NEUTRAL_US;
        bool high_gear = false;
        bool diff_engaged = diff_state; // start from persisted state
        uint32_t gear_us = GEAR_LOW_US;
        uint32_t diff_front_us = DIFF_OPEN_FRONT_US;
        uint32_t diff_rear_us = DIFF_OPEN_REAR_US;

        if (remote_connected && override_mode != RC_OVERRIDE_MUTE) {
            switch (override_mode) {
            case RC_OVERRIDE_REMOTE:
                // Manual override passthrough
                steer_us = rc_get_pulse_us(RC_STEER);
                thr_target_us = rc_get_pulse_us(RC_THROTTLE);
                high_gear = (rc_get_pulse_us(RC_HIGH_GEAR) > SERVO_NEUTRAL_US);
                // Update ROS shadow for better transition in case ros agent is off
                int32_t steer_delta = (int32_t)steer_us - SERVO_NEUTRAL_US;
                int32_t steer_val = (steer_delta * 127) / 500;
                if (steer_val > 127)
                    steer_val = 127;
                if (steer_val < -127)
                    steer_val = -127;
                g_ros_ctrl.steering = (int8_t)steer_val;

                int32_t thr_delta = (int32_t)SERVO_NEUTRAL_US - (int32_t)thr_target_us;
                int32_t thr_val = (thr_delta * 127) / 500;
                if (thr_val > 127)
                    thr_val = 127;
                if (thr_val < -127)
                    thr_val = -127;
                g_ros_ctrl.throttle = (int8_t)thr_val;
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

            case RC_OVERRIDE_ROS:
                // TODO ADD CHECK IF ROS IS ACTIVE, OTHERWISE FALLBACK TO MANUAL
                // ROS control

                steer_us = int8_to_us(g_ros_ctrl.steering);
                thr_target_us = SERVO_NEUTRAL_US;

                uint32_t now_ms = k_uptime_get_32();
                uint32_t age_ms = now_ms - g_ros_ctrl.timestamp;
                if (age_ms < 150U) {
                    thr_target_us = int8_to_throttle_us(g_ros_ctrl.throttle);
                } else {
                    thr_target_us = SERVO_NEUTRAL_US;
                    filtered_thr = (float)SERVO_NEUTRAL_US;
                }

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

            // 3) Apply throttle smoothing via first-order filter
            int32_t neutral_filtered = abs((int32_t)filtered_thr - (int32_t)SERVO_NEUTRAL_US);
            int32_t neutral_target = abs((int32_t)thr_target_us - (int32_t)SERVO_NEUTRAL_US);
            bool accelerating = neutral_target > neutral_filtered;

            float tau_ms = THROTTLE_FILTER_ACCEL_TAU_MS;
            if (!accelerating) {
                tau_ms = THROTTLE_FILTER_DECEL_TAU_MS;
            } else if (neutral_filtered < THROTTLE_FILTER_SOFT_ZONE_US) {
                tau_ms = THROTTLE_FILTER_STANDSTILL_TAU_MS;
            }

            if (high_gear) {
                tau_ms *= THROTTLE_FILTER_HIGH_GEAR_SCALE;
            }

            float dt_ms = (float)dt_us / 1000.0f;
            if (dt_ms < 0.0f) {
                dt_ms = 0.0f;
            }

            float alpha = dt_ms / (tau_ms + dt_ms);
            if (alpha > 1.0f)
                alpha = 1.0f;
            else if (alpha < 0.0f)
                alpha = 0.0f;

            filtered_thr += alpha * ((float)thr_target_us - filtered_thr);

            if (filtered_thr < 1000.0f)
                filtered_thr = 1000.0f;
            if (filtered_thr > 2000.0f)
                filtered_thr = 2000.0f;

            uint32_t thr_output_us = (uint32_t)(filtered_thr + 0.5f);
            forward_guess = thr_output_us > 1450; // simple heuristic

            // 4) Write outputs
            servo_set_ticks(&servos[SERVO_STEERING].spec, steer_us);
            servo_set_ticks(&servos[SERVO_THROTTLE].spec, thr_output_us);
            servo_set_ticks(&servos[SERVO_GEAR].spec, gear_us);
            servo_set_ticks(&servos[SERVO_DIFF].spec, diff_front_us);
            servo_set_ticks(&servos[SERVO_DIFF_REAR].spec, diff_rear_us);
        } else {
            // No RC connection, safe neutral outputs
            servo_set_ticks(&servos[SERVO_STEERING].spec, 0);
            servo_set_ticks(&servos[SERVO_THROTTLE].spec, 0);
            servo_set_ticks(&servos[SERVO_GEAR].spec, 0);
            servo_set_ticks(&servos[SERVO_DIFF].spec, 0);
            servo_set_ticks(&servos[SERVO_DIFF_REAR].spec, 0);
            filtered_thr = (float)SERVO_NEUTRAL_US;
        }
        prev_loop_us = loop_start_us;
        // 5) Keep the loop period
        const uint64_t elapsed_us = k_ticks_to_us_floor64(k_uptime_ticks()) - loop_start_us;
        const int32_t sleep_time = (int32_t)(LOOP_MS * 1000) - (int32_t)elapsed_us;
        if (sleep_time > 0) {
            k_sleep(K_USEC(sleep_time));
        } else {
            LOG_INF("Control loop overrun by %d us", -sleep_time);
            k_sleep(K_USEC(100)); // yield to lower-prio tasks at least a lil bit
        }
    }
}

K_THREAD_DEFINE(control_tid, 2048, control_thread, NULL, NULL, NULL, 3 /* prio */, 0, 0);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * SVEA Note.
 *
 * Part of the SVEA Low‑Level Interface (Zephyr) application.
 * Author: Nils Kiefer
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* rc_input.c – SBUS via Zephyr input (futaba,sbus) */

#include "rc_input.h"
#include "control.h"
#include "loop_delays.h"
#include "ros_iface.h"
#include <limits.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int8.h>
#include <string.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(rc_input_hw, LOG_LEVEL_INF);

static uint16_t sbus_raw[16];
static uint32_t last_frame_ms;
static atomic_t diff_toggle_ev = ATOMIC_INIT(0);
static bool last_ch4_high = false;

// Servos expect values in microseconds (1000-2000)
// SBUS sends values in range ~172-1811 (typical, may vary slightly
static inline uint32_t map_sbus_to_us(uint16_t v) {
    // Clamp input to valid SBUS range
    if ((int)v < 172)
        v = 172;
    if (v > 1811)
        v = 1811;
    // Linear mapping from [172, 1811] -> [1000, 2000]
    return 1000 + ((uint32_t)(v - 172) * 1000U) / (1811 - 172);
}

static inline int8_t pulse_to_int8(int32_t us) {
    if (us < 1000)
        us = 1000;
    if (us > 2000)
        us = 2000;
    return (int8_t)(((us - 1500) * 127) / 500);
}

static inline bool pulse_to_bool(uint32_t us) {
    return us > 1500;
}

static void sbus_input_cb(struct input_event *evt, void *user_data) {
    ARG_UNUSED(user_data);

    /* Sync-marked events correspond to validated SBUS frames */
    if (evt->sync) {
        last_frame_ms = k_uptime_get_32();
        return;
    }

    if (!IS_ENABLED(CONFIG_INPUT_SBUS_SEND_SYNC)) {
        last_frame_ms = k_uptime_get_32();
    }

    if (evt->type != INPUT_EV_ABS) {
        return;
    }

    /* Map codes from overlay to sbus_raw indices:
     * INPUT_ABS_X  -> ch1 (steering)
     * INPUT_ABS_Y  -> ch2 (throttle)
     * INPUT_ABS_Z  -> ch4 (diff toggle button)
     * INPUT_ABS_RX -> ch5 (override)
     * INPUT_ABS_RY -> ch6 (gear)
     */
    switch (evt->code) {
    case INPUT_ABS_X:
        sbus_raw[0] = evt->value;
        break; /* ch1 */
    case INPUT_ABS_Y:
        sbus_raw[1] = evt->value;
        break; /* ch2 */
    case INPUT_ABS_Z: {
        sbus_raw[3] = evt->value; /* ch4 */

        bool high = sbus_raw[3] > 500U;

        if (high && !last_ch4_high) {
            atomic_or(&diff_toggle_ev, 1);
        }
        last_ch4_high = high;
        break;
    }
    case INPUT_ABS_RX:
        sbus_raw[4] = evt->value;
        break; /* ch5 */
    case INPUT_ABS_RY:
        sbus_raw[5] = evt->value;
        break; /* ch6 */
    default:
        // If we didnt get one of these somethings up and it should not count as a legit frame
        return;
    }

    /* Throttled event print to verify callback activity */
    static uint32_t last_pr_ms;
    uint32_t now = k_uptime_get_32();
    if ((now - last_pr_ms) > 100U) {
        last_pr_ms = now;
        // printk("SBUS evt: type=%u code=%u val=%d\n", evt->type, evt->code, evt->value);
    }
}

INPUT_CALLBACK_DEFINE(NULL, sbus_input_cb, NULL);

/* Debug thread declaration and stack placed before init */
static void rc_debug_thread(void *a, void *b, void *c);
static K_THREAD_STACK_DEFINE(rc_dbg_stack, 1024);
static struct k_thread rc_dbg_thread_data;

static void rc_remote_publish_thread(void *a, void *b, void *c);
static K_THREAD_STACK_DEFINE(rc_remote_pub_stack, 1024);
static struct k_thread rc_remote_pub_thread_data;

static void rc_connected_publish_thread(void *a, void *b, void *c);
static K_THREAD_STACK_DEFINE(rc_connected_pub_stack, 1024);
static struct k_thread rc_connected_pub_thread_data;

void rc_input_init(void) {
    /* Do not reset sbus_raw here; callback may already have populated it
     * before this init runs. Just announce readiness and start debug thread.
     */
    LOG_INF("RC input: futaba,sbus callback registered");
    /* Start periodic debug printer */

    k_thread_create(&rc_dbg_thread_data, rc_dbg_stack, K_THREAD_STACK_SIZEOF(rc_dbg_stack),
                    rc_debug_thread, NULL, NULL, NULL,
                    10, 0, K_NO_WAIT);
    k_thread_name_set(&rc_dbg_thread_data, "rc_dbg");

    k_thread_create(&rc_remote_pub_thread_data, rc_remote_pub_stack, K_THREAD_STACK_SIZEOF(rc_remote_pub_stack),
                    rc_remote_publish_thread, NULL, NULL, NULL,
                    5, 0, K_NO_WAIT);
    k_thread_name_set(&rc_remote_pub_thread_data, "rc_remote_pub");

    k_thread_create(&rc_connected_pub_thread_data, rc_connected_pub_stack, K_THREAD_STACK_SIZEOF(rc_connected_pub_stack),
                    rc_connected_publish_thread, NULL, NULL, NULL,
                    5, 0, K_NO_WAIT);
    k_thread_name_set(&rc_connected_pub_thread_data, "rc_remote_conn");
}

uint32_t rc_get_pulse_us(rc_channel_t ch) {
    switch (ch) {
    case RC_STEER:
        return map_sbus_to_us(sbus_raw[0]); /* ch1 */
    case RC_THROTTLE: {
        /* Invert throttle around center (1500us) and clamp to [1000,2000] */
        uint32_t us = map_sbus_to_us(sbus_raw[1]); /* ch2 */
        if (us < 1000U)
            us = 1000U;
        if (us > 2000U)
            us = 2000U;
        uint32_t inv = 3000U - us; /* 1000<->2000, 1500 stays */
        if (inv < 1000U)
            inv = 1000U;
        if (inv > 2000U)
            inv = 2000U;
        return inv;
    }
    case RC_HIGH_GEAR:
        return map_sbus_to_us(sbus_raw[5]); /* ch6 */
    case RC_OVERRIDE:
        return map_sbus_to_us(sbus_raw[4]); /* ch5 */
    case RC_DIFF_TOGGLE:
        return map_sbus_to_us(sbus_raw[3]); /* ch4 */
    default:
        return 1500;
    }
}

uint32_t rc_get_period_us(rc_channel_t idx) {
    ARG_UNUSED(idx);
    return 0;
}

void rc_input_debug_dump(void) {
    return;
    rc_override_mode_t mode = rc_get_override_mode();
    const char *mstr = (mode == RC_OVERRIDE_ROS) ? "ROS" : (mode == RC_OVERRIDE_MUTE) ? "MUTE"
                                                                                      : "FULL";
    // Compute mapped microseconds and int8-scaled values used by ROS
    uint32_t steer_us = rc_get_pulse_us(RC_STEER);
    uint32_t throttle_us = rc_get_pulse_us(RC_THROTTLE);
    uint32_t diff_us = rc_get_pulse_us(RC_DIFF_TOGGLE);
    uint32_t gear_us = rc_get_pulse_us(RC_HIGH_GEAR);
    uint32_t ovr_us = rc_get_pulse_us(RC_OVERRIDE);

    int8_t steer_i8 = pulse_to_int8(steer_us);
    int8_t throttle_i8 = pulse_to_int8(throttle_us);

    // Print raw values, mapped microseconds, and scaled int8 values
    LOG_INF(
        "SBUS raw ch1=%u ch2=%u ch4=%u ch5=%u ch6=%u us[steer=%u thr=%u diff=%u gear=%u ovr=%u] i8[steer=%d thr=%d] mode=%s age=%u us",
        sbus_raw[0], sbus_raw[1], sbus_raw[3], sbus_raw[4], sbus_raw[5],
        steer_us, throttle_us, diff_us, gear_us, ovr_us,
        (int)steer_i8, (int)throttle_i8, mstr,
        k_uptime_get_32() - last_frame_ms);
}

/* Simple periodic debug printer thread */
static void rc_debug_thread(void *a, void *b, void *c) {
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);
    while (1) {
        if (!ros_initialized) {
            rc_input_debug_dump();
        }
        k_msleep(1000);
    }
}

/* (moved to top of file before rc_input_init) */

bool rc_input_connected(void) {
    /* Diff-toggle channel is bistable: failsafe drives it into the mid band */
    uint16_t ch4_raw = sbus_raw[3];
    bool connected = (ch4_raw < 800U) || (ch4_raw > 1200U);

    static bool last_state = true;
    if (connected != last_state) {
        LOG_INF("RC link %s (ch4 raw=%u)", connected ? "reconnected" : "lost", ch4_raw);
        last_state = connected;
    }

    return connected;
}

// Classify override channel into three discrete modes using wide bands
// Low  -> ROS control
// Mid  -> Override engaged, but mute outputs (PWM=0)
// High -> Full manual override passthrough
rc_override_mode_t rc_get_override_mode(void) {
    uint32_t us = rc_get_pulse_us(RC_OVERRIDE);
    // Narrower bands: <1400=ROS, 1400..1600=MUTE, >1600=FULL
    if (us < 1400U) {
        return RC_OVERRIDE_ROS;
    } else if (us <= 1600U) {
        return RC_OVERRIDE_MUTE;
    } else {
        return RC_OVERRIDE_REMOTE;
    }
}

bool rc_consume_diff_toggle_event(void) {
    /* Atomically fetch-and-clear latch; true if a rising edge was latched */
    return atomic_set(&diff_toggle_ev, 0) != 0;
}

static void rc_remote_publish_thread(void *a, void *b, void *c) {
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    static std_msgs__msg__Int8 steer_msg;
    static std_msgs__msg__Int8 throttle_msg;
    static std_msgs__msg__Bool gear_msg;
    static std_msgs__msg__Bool override_msg;

    bool first_run = true;
    bool last_connected = false;
    uint32_t last_publish_ms = 0U;
    uint32_t min_period_ms = UINT32_MAX;
    uint32_t max_period_ms = 0U;
    uint32_t period_samples = 0U;

    while (1) {
        if (!ros_initialized) {
            k_msleep(10);
            continue;
        }

        bool connected = rc_input_connected();

        if (first_run) {
            last_connected = connected;
            first_run = false;
        }

        if (connected != last_connected) {
            LOG_INF("Remote RC link %s", connected ? "reconnected" : "disconnected");
            last_connected = connected;
        }

        uint32_t now_ms = k_uptime_get_32();
        if (last_publish_ms != 0U) {
            uint32_t period_ms = now_ms - last_publish_ms;
            if (period_ms < min_period_ms)
                min_period_ms = period_ms;
            if (period_ms > max_period_ms)
                max_period_ms = period_ms;
            if (++period_samples >= 50U) {
                LOG_DBG("Remote publish period min=%u ms max=%u ms", min_period_ms, max_period_ms);
                min_period_ms = UINT32_MAX;
                max_period_ms = 0U;
                period_samples = 0U;
            }
        }
        last_publish_ms = now_ms;

        int32_t steer_us = rc_get_pulse_us(RC_STEER);
        int32_t throttle_us = rc_get_pulse_us(RC_THROTTLE);
        uint32_t gear_us = rc_get_pulse_us(RC_HIGH_GEAR);
        bool override_active = (rc_get_override_mode() != RC_OVERRIDE_ROS);

        if (!connected) {
            steer_us = 1500;
            throttle_us = 1500;
            gear_us = 1500;
            override_active = true;
        }

        steer_msg.data = pulse_to_int8(steer_us);
        throttle_msg.data = pulse_to_int8(throttle_us);
        gear_msg.data = pulse_to_bool(gear_us);
        override_msg.data = override_active;

        rcl_ret_t rc;

        // publish speed throttle

        rc = ros_publish_try(&pub_remote_steer, &steer_msg);
        if (rc != RCL_RET_OK) {
            // LOG_WRN("Remote publish steer failed rc=%d", (int)rc);
        }
        k_msleep(50); // small delay to avoid flooding the transport
        rc = ros_publish_try(&pub_remote_throttle, &throttle_msg);
        if (rc != RCL_RET_OK) {
            // LOG_WRN("Remote publish throttle failed rc=%d", (int)rc);
        }
        k_msleep(50); // small delay to avoid flooding the transport
        rc = ros_publish_try(&pub_remote_gear, &gear_msg);
        if (rc != RCL_RET_OK) {
            // LOG_WRN("Remote publish gear failed rc=%d", (int)rc);
        }
        k_msleep(50); // small delay to avoid flooding the transport
        rc = ros_publish_try(&pub_remote_override, &override_msg);
        if (rc != RCL_RET_OK) {
            // LOG_WRN("Remote publish override failed rc=%d", (int)rc);
        }
        k_msleep(50); // small delay to avoid flooding the transport
    }
}

static void rc_connected_publish_thread(void *a, void *b, void *c) {
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    static std_msgs__msg__Bool connected_msg;
    static uint8_t fail_streak = 0;
    uint32_t last_publish_ms = 0U;

#if (LOG_LEVEL >= LOG_LEVEL_DBG)
    uint32_t min_period_ms = UINT32_MAX;
    uint32_t max_period_ms = 0U;
    uint32_t samples = 0U;
#endif

    while (1) {
        if (!ros_initialized) {
            k_msleep(100);
            continue;
        }

        connected_msg.data = remote_connected;

#if (LOG_LEVEL >= LOG_LEVEL_DBG)
        uint32_t now_ms = k_uptime_get_32();

        if (last_publish_ms != 0U) {
            uint32_t period_ms = now_ms - last_publish_ms;
            if (period_ms < min_period_ms)
                min_period_ms = period_ms;
            if (period_ms > max_period_ms)
                max_period_ms = period_ms;
            if (++samples >= 50U) {
                LOG_DBG("Remote connected publish period min=%u ms max=%u ms", min_period_ms, max_period_ms);
                min_period_ms = UINT32_MAX;
                max_period_ms = 0U;
                samples = 0U;
            }
        }

        last_publish_ms = now_ms;
#endif
        rcl_ret_t rc = ros_publish_locked(&pub_remote_connected, &connected_msg);
        if (rc != RCL_RET_OK) {
            if (fail_streak < 255)
                fail_streak++;
            if (fail_streak >= 3) {
                LOG_WRN("Remote connected reliable publish failing (%u), forcing reconnect", fail_streak);
                ros_iface_handle_remote_publish_error();
                fail_streak = 0;
            } else {
                LOG_WRN("Remote connected publish failed rc=%d", (int)rc);
            }
        } else {
            fail_streak = 0;
        }

        k_msleep(RC_INPUT_PUBLISH_LOOP_DELAY_MS);
    }
}

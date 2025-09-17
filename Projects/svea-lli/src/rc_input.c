/* rc_input.c – SBUS via Zephyr input (futaba,sbus) */

#include "rc_input.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/input/input.h>
#include <string.h>

LOG_MODULE_REGISTER(rc_input_hw, LOG_LEVEL_INF);

static uint16_t sbus_raw[16];
static uint32_t last_frame_ms;

// Servos expect values in microseconds (1000-2000)
// SBUS sends values in range ~172-1811 (typical, may vary slightly
static inline uint32_t map_sbus_to_us(uint16_t v) {
    // Clamp input to valid SBUS range
    if ((int)v < 172) v = 172;
    if (v > 1811) v = 1811;
    // Linear mapping from [172, 1811] -> [1000, 2000]
    return 1000 + ((uint32_t)(v - 172) * 1000U) / (1811 - 172);
}

static void sbus_input_cb(struct input_event *evt, void *user_data)
{
    ARG_UNUSED(user_data);

    /* Update last frame time on any sbus event */
    last_frame_ms = k_uptime_get_32();

    if (evt->type != INPUT_EV_ABS) {
        return;
    }

    /* Map codes from overlay to sbus_raw indices:
     * INPUT_ABS_X  -> ch1 (steering)
     * INPUT_ABS_Y  -> ch2 (throttle)
     * INPUT_ABS_RX -> ch5 (override)
     * INPUT_ABS_RY -> ch6 (gear)
     */
    switch (evt->code) {
    case INPUT_ABS_X:  sbus_raw[0] = evt->value; break;  /* ch1 */
    case INPUT_ABS_Y:  sbus_raw[1] = evt->value; break;  /* ch2 */
    case INPUT_ABS_RX: sbus_raw[4] = evt->value; break;  /* ch5 */
    case INPUT_ABS_RY: sbus_raw[5] = evt->value; break;  /* ch6 */
    default: break;
    }

    /* Throttled event print to verify callback activity */
    static uint32_t last_pr_ms;
    uint32_t now = k_uptime_get_32();
    if ((now - last_pr_ms) > 100U) {
        last_pr_ms = now;
        //printk("SBUS evt: type=%u code=%u val=%d\n", evt->type, evt->code, evt->value);
    }
}

INPUT_CALLBACK_DEFINE(NULL, sbus_input_cb, NULL);

/* Debug thread declaration and stack placed before init */
static void rc_debug_thread(void *a, void *b, void *c);
static K_THREAD_STACK_DEFINE(rc_dbg_stack, 1024);
static struct k_thread rc_dbg_thread_data;

void rc_input_init(void) {
    /* Do not reset sbus_raw here; callback may already have populated it
     * before this init runs. Just announce readiness and start debug thread.
     */
    LOG_INF("RC input: futaba,sbus callback registered");
    /* Start periodic debug printer */

    k_thread_create(&rc_dbg_thread_data, rc_dbg_stack, K_THREAD_STACK_SIZEOF(rc_dbg_stack),
                    rc_debug_thread, NULL, NULL, NULL,
                    5, 0, K_NO_WAIT);
    k_thread_name_set(&rc_dbg_thread_data, "rc_dbg");
}

uint32_t rc_get_pulse_us(rc_channel_t ch) {
    switch (ch) {
    case RC_STEER:     return map_sbus_to_us(sbus_raw[0]); /* ch1 */
    case RC_THROTTLE: {
        /* Invert throttle around center (1500us) and clamp to [1000,2000] */
        uint32_t us = map_sbus_to_us(sbus_raw[1]); /* ch2 */
        if (us < 1000U) us = 1000U;
        if (us > 2000U) us = 2000U;
        uint32_t inv = 3000U - us; /* 1000<->2000, 1500 stays */
        if (inv < 1000U) inv = 1000U;
        if (inv > 2000U) inv = 2000U;
        return inv;
    }
    case RC_HIGH_GEAR: return map_sbus_to_us(sbus_raw[5]); /* ch6 */
    case RC_OVERRIDE:  return map_sbus_to_us(sbus_raw[4]); /* ch5 */
    default: return 1500;
    }
}

uint32_t rc_get_period_us(rc_channel_t idx) {
    ARG_UNUSED(idx);
    return 0;
}

uint32_t rc_get_capture_raw(rc_channel_t ch) {
    switch (ch) {
    case RC_STEER:     return sbus_raw[0]; /* ch1 */
    case RC_THROTTLE:  return sbus_raw[1]; /* ch2 */
    case RC_HIGH_GEAR: return sbus_raw[5]; /* ch6 */
    case RC_OVERRIDE:  return sbus_raw[4]; /* ch5 */
    default: return 0;
    }
}

uint32_t rc_get_age_us(rc_channel_t ch) {
    ARG_UNUSED(ch);
    uint32_t now = k_uptime_get_32();
    uint32_t last = last_frame_ms;
    return (now - last) * 1000U;
}

void rc_input_debug_dump(void) {
    uint32_t age_us = rc_get_age_us(RC_STEER);
    LOG_INF("SBUS raw ch1=%u ch2=%u ch5=%u ch6=%u us[steer=%u thr=%u gear=%u ovr=%u] age=%u us\n",
           sbus_raw[0], sbus_raw[1], sbus_raw[4], sbus_raw[5],
           map_sbus_to_us(sbus_raw[0]), map_sbus_to_us(sbus_raw[1]),
           map_sbus_to_us(sbus_raw[5]), map_sbus_to_us(sbus_raw[4]),
           age_us);
}

/* Simple periodic debug printer thread */
static void rc_debug_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
    while (1) {
        rc_input_debug_dump();
        k_msleep(100);
    }
}

/* (moved to top of file before rc_input_init) */

bool rc_input_connected(void)
{
    /* Consider link lost if we haven't seen a frame in 250 ms */
    const uint32_t age_ms = k_uptime_get_32() - last_frame_ms;
    return age_ms <= 250U;
}

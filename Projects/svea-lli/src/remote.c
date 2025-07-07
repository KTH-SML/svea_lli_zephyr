#include "remote.h"
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

LOG_MODULE_REGISTER(remote, LOG_LEVEL_INF);

static RemoteState cur, prev;
static atomic_t alive_mask;
static atomic_t rc_valid_atomic;

// Fixed: K_MSGQ_DEFINE now requires 4 arguments in Zephyr 4.1 (name, msg_size, max_msgs, align)
K_MSGQ_DEFINE(rc_q, sizeof(RemoteState), 4, 4);

static void rc_wdg_expiry(struct k_timer *timer) {
    LOG_WRN("RC watchdog timeout - all channels lost");
    for (int ch = 0; ch < NUM_RC_CHANNELS; ch++) {
        remote_link_lost(ch);
    }
}

// Fixed: K_TIMER_DEFINE with proper expiry function
K_TIMER_DEFINE(rc_wdg, rc_wdg_expiry, NULL);

void remote_report(int ch, uint32_t us) {
    if (ch >= NUM_RC_CHANNELS)
        return;

    atomic_set_bit(&alive_mask, ch);
    atomic_set(&rc_valid_atomic, 1);
    cur.fields[ch] = us;

    // Update named fields for convenience
    switch (ch) {
    case RC_STEER:
        cur.steer = us;
        break;
    case RC_HIGH_GEAR:
        cur.high_gear_us = us;
        break;
    case RC_THROTTLE:
        cur.throttle = us;
        break;
    case RC_OVERRIDE:
        cur.override_us = us;
        break;
    }

    // Only send if frame changed
    if (memcmp(&cur, &prev, sizeof(RemoteState)) != 0) {
        prev = cur;
        k_msgq_put(&rc_q, &cur, K_NO_WAIT);
    }

    // Fixed: k_timer_start now requires 3 arguments (timer, duration, period)
    k_timer_start(&rc_wdg, K_MSEC(120), K_NO_WAIT);
}

void remote_link_lost(int ch) {
    if (ch >= NUM_RC_CHANNELS)
        return;

    atomic_clear_bit(&alive_mask, ch);
    LOG_WRN("RC channel %d link lost", ch);

    if (atomic_get(&alive_mask) == 0) {
        atomic_set(&rc_valid_atomic, 0);
        LOG_WRN("All RC channels lost");
        k_msgq_put(&rc_q, &prev, K_NO_WAIT); // Wake arbiter
    }
}

void remote_init(void) {
    atomic_clear(&alive_mask);
    memset(&cur, 0, sizeof(cur));
    memset(&prev, 0, sizeof(prev));

    LOG_INF("Remote control initialized");
}

// Add getter function in remote.h
bool remote_is_valid(void) {
    return atomic_get(&rc_valid_atomic) != 0;
}

bool remote_all_channels_valid(void) {
    // All channels alive if all bits set
    return atomic_get(&alive_mask) == ((1 << NUM_RC_CHANNELS) - 1);
}
#pragma once

#include <stdint.h>

typedef enum {
    RC_STEER = 0,
    RC_HIGH_GEAR,
    RC_THROTTLE,
    RC_OVERRIDE,
    NUM_RC_CHANNELS
} rc_channel_t;

// Returns the latest captured pulse width (timer ticks) for the channel
uint32_t rc_get_capture_raw(rc_channel_t ch);
uint32_t rc_get_age_us(rc_channel_t ch);
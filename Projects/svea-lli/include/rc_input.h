#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    RC_STEER = 0,
    RC_HIGH_GEAR,
    RC_THROTTLE,
    RC_OVERRIDE,
    RC_DIFF_TOGGLE,
    NUM_RC_CHANNELS
} rc_channel_t;

// Three-way override switch mapping (based on RC channel ~1000/1500/2000us)
typedef enum {
    RC_OVERRIDE_ROS = 0,   // ~1000us: ROS control
    RC_OVERRIDE_MUTE = 1,  // ~1500us: override engaged but outputs muted (PWM=0)
    RC_OVERRIDE_REMOTE = 2 // ~2000us: full manual override (RC passthrough)
} rc_override_mode_t;

/* Initialize SBUS (USART2) RC input */
void rc_input_init(void);

/* Print SBUS debug information (raw channels, mapped us, age). */
void rc_input_debug_dump(void);

// Returns the latest captured pulse width (timer ticks) for the channel
uint32_t rc_get_age_us(rc_channel_t ch);
uint32_t rc_get_pulse_us(rc_channel_t ch);

/* Returns true if fresh SBUS data has been received recently. */
bool rc_input_connected(void);

// Decode three-way override switch position from RC channel
rc_override_mode_t rc_get_override_mode(void);

// Returns true once per CH4 rising edge (>1600us), then clears the latch
bool rc_consume_diff_toggle_event(void);

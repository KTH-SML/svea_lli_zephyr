#pragma once

#include <stdint.h>

typedef enum {
    RC_STEER = 0,
    RC_HIGH_GEAR,
    RC_THROTTLE,
    RC_OVERRIDE,
    NUM_RC_CHANNELS
} rc_channel_t;

/* Initialize SBUS (USART2) RC input */
void rc_input_init(void);

/* Print SBUS debug information (raw channels, mapped us, age). */
void rc_input_debug_dump(void);

// Returns the latest captured pulse width (timer ticks) for the channel
uint32_t rc_get_capture_raw(rc_channel_t ch);
uint32_t rc_get_age_us(rc_channel_t ch);
uint32_t rc_get_pulse_us(rc_channel_t ch);

/* Returns true if fresh SBUS data has been received recently. */
bool rc_input_connected(void);

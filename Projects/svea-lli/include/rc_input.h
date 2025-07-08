#ifndef RC_INPUT_H
#define RC_INPUT_H

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>

#define NUM_RC_CHANNELS 4

typedef enum {
    RC_STEER = 0,
    RC_HIGH_GEAR = 1,
    RC_THROTTLE = 2,
    RC_OVERRIDE = 3
} rc_channel_t;

static const char *const rc_channel_names[NUM_RC_CHANNELS] = {
    [RC_STEER] = "steer",
    [RC_HIGH_GEAR] = "high_gear",
    [RC_THROTTLE] = "throttle",
    [RC_OVERRIDE] = "override",
};

#define RC_CHANNEL_NAME(ch) (rc_channel_names[(ch)])

// Raw capture values (from ISR/callback)
typedef struct {
    volatile uint32_t pulse;
    volatile uint32_t period;
    volatile int status;
    volatile bool fresh;
    uint64_t timestamp; // Timestamp of last update in milliseconds
} rc_capture_raw_t;

// Converted microsecond values (updated by a thread or on-demand)
typedef struct {
    volatile uint64_t pulse_ns;
    volatile uint64_t period_ns;
} rc_capture_ns_t;

// Only provide accessors, not the arrays themselves!
const rc_capture_raw_t *rc_get_capture_raw(rc_channel_t ch);
const rc_capture_ns_t *rc_get_capture_ns(rc_channel_t ch);

void rc_input_init(void);

#endif // RC_INPUT_H
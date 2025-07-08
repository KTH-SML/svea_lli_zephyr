#include "rc_input.h"
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(rc_input, LOG_LEVEL_INF);

struct rc_input_channel {
    const struct device *dev;
    uint32_t channel;
    int rc_ch_id;
    const char *name;
};

static struct rc_input_channel rc_channels[] = {
#if DT_NODE_EXISTS(DT_ALIAS(rc_steer))
    {.dev = DEVICE_DT_GET(DT_ALIAS(rc_steer)), .channel = 1, .name = "rc-steer", .rc_ch_id = RC_STEER},
#endif
#if DT_NODE_EXISTS(DT_ALIAS(rc_high_gear))
    {.dev = DEVICE_DT_GET(DT_ALIAS(rc_high_gear)), .channel = 1, .name = "rc-high-gear", .rc_ch_id = RC_HIGH_GEAR},
#endif
#if DT_NODE_EXISTS(DT_ALIAS(rc_throttle))
    {.dev = DEVICE_DT_GET(DT_ALIAS(rc_throttle)), .channel = 1, .name = "rc-throttle", .rc_ch_id = RC_THROTTLE},
#endif
#if DT_NODE_EXISTS(DT_ALIAS(rc_override))
    {.dev = DEVICE_DT_GET(DT_ALIAS(rc_override)), .channel = 3, .name = "rc-override", .rc_ch_id = RC_OVERRIDE},
#endif
};

static rc_capture_raw_t rc_capture_raw[NUM_RC_CHANNELS];
static rc_capture_ns_t rc_capture_ns[NUM_RC_CHANNELS];

bool rc_initialized = false;

// Forward declaration for per-channel thread function
static void rc_input_channel_poll_thread(void *p1, void *p2, void *p3);

// Thread stack and IDs for each channel
#define RC_INPUT_STACK_SIZE 512
#define RC_INPUT_PRIORITY   1

K_THREAD_STACK_DEFINE(rc_input_stacks[NUM_RC_CHANNELS], RC_INPUT_STACK_SIZE);
struct k_thread rc_input_threads[NUM_RC_CHANNELS];

// Start a polling thread for each RC input channel
void rc_input_init(void) {
    LOG_INF("Initializing RC input polling");
    memset(rc_capture_raw, 0, sizeof(rc_capture_raw));
    memset(rc_capture_ns, 0, sizeof(rc_capture_ns));
    rc_initialized = true;

    for (int i = 0; i < NUM_RC_CHANNELS; ++i) {
        k_thread_create(&rc_input_threads[i], rc_input_stacks[i], RC_INPUT_STACK_SIZE,
                        rc_input_channel_poll_thread, (void *)(intptr_t)i, NULL, NULL,
                        RC_INPUT_PRIORITY, 0, K_NO_WAIT);
    }
}

// Per-channel polling thread
static void rc_input_channel_poll_thread(void *p1, void *p2, void *p3) {
    int idx = (int)(intptr_t)p1;
    const struct rc_input_channel *ch = &rc_channels[idx];

    while (!rc_initialized) {
        k_sleep(K_MSEC(10));
    }
    while (1) {
        uint32_t period = 0, pulse = 0;
        int ret = pwm_capture_cycles(ch->dev, ch->channel,
                                     PWM_CAPTURE_TYPE_BOTH | PWM_CAPTURE_MODE_SINGLE,
                                     &period, &pulse, K_MSEC(50));
        if (ret == 0) {
            rc_capture_raw[idx].period = period;
            rc_capture_raw[idx].pulse = pulse;
            rc_capture_raw[idx].status = 0;
            rc_capture_raw[idx].fresh = true;
            rc_capture_raw[idx].timestamp = k_uptime_get();

            pwm_cycles_to_nsec(ch->dev, ch->channel, pulse, &rc_capture_ns[idx].pulse_ns);
            pwm_cycles_to_nsec(ch->dev, ch->channel, period, &rc_capture_ns[idx].period_ns);
        } else {
            rc_capture_raw[idx].status = ret;
            LOG_WRN("PWM capture timed out on channel %d (%s), ret=%d", idx, ch->name, ret);
        }
        k_sleep(K_MSEC(2));
    }
}

const rc_capture_raw_t *rc_get_capture_raw(rc_channel_t ch) {
    return &rc_capture_raw[ch];
}

const rc_capture_ns_t *rc_get_capture_ns(rc_channel_t ch) {
    return &rc_capture_ns[ch];
}

// Thread to periodically log RC input values
void rc_input_logger_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (1) {
        int64_t now = k_uptime_get();
        for (int i = 0; i < NUM_RC_CHANNELS; ++i) {
            int64_t age = now - rc_capture_raw[i].timestamp;
            LOG_INF("RC ch %d (%s): period=%lu, pulse=%lu, status=%d, fresh=%d, timestamp=%lld, age=%lld ms",
                    i,
                    rc_channels[i].name,
                    (unsigned long)rc_capture_raw[i].period,
                    (unsigned long)rc_capture_raw[i].pulse,
                    rc_capture_raw[i].status,
                    rc_capture_raw[i].fresh,
                    rc_capture_raw[i].timestamp,
                    age);
        }
        k_sleep(K_SECONDS(1));
    }
}

// Start the logger thread at file scope
K_THREAD_DEFINE(rc_input_logger_tid, 1024, rc_input_logger_thread, NULL, NULL, NULL, 5, 0, 0);
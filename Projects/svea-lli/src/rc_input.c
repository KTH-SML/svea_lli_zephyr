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
    {.dev = DEVICE_DT_GET(DT_ALIAS(rc_steer)),
     .channel = 1, // TIM3_CH1
     .name = "rc-steer",
     .rc_ch_id = RC_STEER},
#endif
#if DT_NODE_EXISTS(DT_ALIAS(rc_high_gear))
    {.dev = DEVICE_DT_GET(DT_ALIAS(rc_high_gear)),
     .channel = 1, // TIM9_CH1
     .name = "rc-high-gear",
     .rc_ch_id = RC_HIGH_GEAR},
#endif
#if DT_NODE_EXISTS(DT_ALIAS(rc_throttle))
    {.dev = DEVICE_DT_GET(DT_ALIAS(rc_throttle)),
     .channel = 1, // TIM5_CH1
     .name = "rc-throttle",
     .rc_ch_id = RC_THROTTLE},
#endif
#if DT_NODE_EXISTS(DT_ALIAS(rc_override))
    {.dev = DEVICE_DT_GET(DT_ALIAS(rc_override)),
     .channel = 3, // TIM4_CH3
     .name = "rc-override",
     .rc_ch_id = RC_OVERRIDE},
#endif
};

static rc_capture_raw_t rc_capture_raw[NUM_RC_CHANNELS];

bool rc_initialized = false;

// Callback for PWM capture
static void rc_capture_cb(const struct device *dev, uint32_t channel,
                          uint32_t period, uint32_t pulse, int status, void *user_data) {
    rc_channel_t ch = (rc_channel_t)(uintptr_t)user_data;

    rc_capture_raw[ch].pulse = pulse;
    rc_capture_raw[ch].status = status;
    rc_capture_raw[ch].fresh = true;
    rc_capture_raw[ch].timestamp = k_uptime_get();
}

// Initialization: register callback for each channel
void rc_input_init(void) {
    LOG_INF("Initializing RC input capture");

    memset(rc_capture_raw, 0, sizeof(rc_capture_raw));

    int success_count = 0;

    for (int i = 0; i < NUM_RC_CHANNELS; i++) {
        const struct rc_input_channel *ch = &rc_channels[i];
        if (!ch->dev || !device_is_ready(ch->dev)) {
            LOG_WRN("RC channel %d device not ready or missing", i);
            continue;
        }

        LOG_INF("Configuring RC input %s on channel %d", ch->name, ch->channel);

        int ret = pwm_configure_capture(
            ch->dev, ch->channel,
            PWM_CAPTURE_TYPE_PULSE | PWM_CAPTURE_MODE_CONTINUOUS,
            rc_capture_cb, (void *)(uintptr_t)i);
        if (ret < 0) {
            LOG_ERR("Failed to configure PWM capture for channel %d: %d", i, ret);
            continue;
        }

        ret = pwm_enable_capture(ch->dev, ch->channel);
        if (ret < 0) {
            LOG_ERR("Failed to enable PWM capture for channel %d: %d", i, ret);
            continue;
        }

        success_count++;
        LOG_INF("RC input %s initialized on channel %d", RC_CHANNEL_NAME(i), ch->channel);
    }

    LOG_INF("RC input initialization complete: %d/%d channels ready",
            success_count, NUM_RC_CHANNELS);
    if (success_count == 0) {
        LOG_ERR("No RC channels initialized! System may not respond to RC input.");
    }
}

const rc_capture_raw_t *rc_get_capture_raw(rc_channel_t ch) {
    return &rc_capture_raw[ch];
}

// Logger thread (unchanged except period removed)
void rc_input_logger_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (1) {
        int64_t now = k_uptime_get();
        for (int i = 0; i < NUM_RC_CHANNELS; ++i) {
            int64_t age = now - rc_capture_raw[i].timestamp;
            LOG_INF("RC ch %d (%s): raw_pulse=%lu us, status=%d, fresh=%d, timestamp=%lld, age=%lld ms",
                    i, rc_channels[i].name, (unsigned long)rc_capture_raw[i].pulse,
                    rc_capture_raw[i].status, rc_capture_raw[i].fresh,
                    rc_capture_raw[i].timestamp, age);
        }
        k_sleep(K_SECONDS(1));
    }
}

K_THREAD_DEFINE(rc_input_logger_tid, 1024, rc_input_logger_thread, NULL, NULL, NULL, 5, 0, 0);
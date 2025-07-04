#include "rc_input.h"
#include "remote.h"
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(rc_input, LOG_LEVEL_INF);

// Structure to hold PWM input channel info
struct rc_input_channel {
    const struct device *dev;
    uint32_t channel;
    const char *name;
    int rc_ch_id;
};

// Define RC input channels using direct PWM controller references
static struct rc_input_channel rc_channels[] = {
#if DT_NODE_EXISTS(DT_ALIAS(rc_steer))
    {.dev = DEVICE_DT_GET(DT_ALIAS(rc_steer)),
     .channel = 1, // TIM3_CH1
     .name = "rc-steer",
     .rc_ch_id = RC_STEER},
#endif
#if DT_NODE_EXISTS(DT_ALIAS(rc_gear))
    {.dev = DEVICE_DT_GET(DT_ALIAS(rc_gear)),
     .channel = 1, // TIM9_CH1
     .name = "rc-gear",
     .rc_ch_id = RC_GEAR},
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

// Add array to track initialized channels
static bool rc_channel_initialized[NUM_RC_CHANNELS] = {false};

uint32_t ticks_to_us(uint32_t ticks) {
    // Convert timer ticks to microseconds
    // With prescaler 107, timer frequency is ~1MHz (1 tick ≈ 1 µs)
    return ticks;
}

void common_cb(const struct device *dev, uint32_t channel,
               uint32_t period, uint32_t pulse, int status, void *user_data) {
    int ch = (int)(uintptr_t)user_data;

    // Check for capture errors
    if (status != 0) {
        LOG_WRN("PWM capture error on channel %d, status: %d", ch, status);
        remote_link_lost(ch);
        return;
    }

    // Check for reasonable pulse width (0.5ms to 2.5ms for RC signals)
    if (pulse < 500 || pulse > 2500) {
        LOG_WRN("PWM pulse out of range on channel %d: %d us", ch, pulse);
        return;
    }

    uint32_t us = ticks_to_us(pulse);
    remote_report(ch, us);
}

void rc_input_init(void) {
    LOG_INF("Initializing RC input capture");

    int success_count = 0;

    for (int i = 0; i < ARRAY_SIZE(rc_channels); i++) {
        const struct rc_input_channel *ch = &rc_channels[i];

        if (!device_is_ready(ch->dev)) {
            LOG_ERR("PWM device %s not ready", ch->dev->name);
            continue;
        }

        // Use the correct Zephyr 4.1 PWM capture API
        int ret = pwm_configure_capture(ch->dev, ch->channel,
                                        PWM_CAPTURE_TYPE_PULSE,
                                        common_cb,
                                        (void *)(uintptr_t)ch->rc_ch_id);
        if (ret < 0) {
            LOG_ERR("Failed to configure PWM capture for %s: %d", ch->name, ret);
            rc_channel_initialized[ch->rc_ch_id] = false;
            continue;
        }

        ret = pwm_enable_capture(ch->dev, ch->channel);
        if (ret < 0) {
            LOG_ERR("Failed to enable PWM capture for %s: %d", ch->name, ret);
            rc_channel_initialized[ch->rc_ch_id] = false;
            continue;
        }

        rc_channel_initialized[ch->rc_ch_id] = true;
        success_count++;

        LOG_INF("RC input %s initialized on channel %d", ch->name, ch->channel);
    }

    LOG_INF("RC input initialization complete: %d/%d channels ready",
            success_count, ARRAY_SIZE(rc_channels));

    if (success_count == 0) {
        LOG_ERR("No RC channels initialized! System may not respond to RC input.");
    }
}
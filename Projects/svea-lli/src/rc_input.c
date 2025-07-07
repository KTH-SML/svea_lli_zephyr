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

// Add array to track initialized channels
static bool rc_channel_initialized[NUM_RC_CHANNELS] = {false};

// Add this mapping for channel names
static const char *rc_channel_names[NUM_RC_CHANNELS] = {
    [RC_STEER] = "steer",
    [RC_HIGH_GEAR] = "high_gear",
    [RC_THROTTLE] = "throttle",
    [RC_OVERRIDE] = "override",
};

BUILD_ASSERT(NUM_RC_CHANNELS == ARRAY_SIZE(rc_channel_names),
             "NUM_RC_CHANNELS must match ARRAY_SIZE(rc_channel_names)");

static uint32_t pwm_clock_frequency_hz = 1000000; // Default to 1 MHz based on comment

uint32_t ticks_to_us(uint32_t ticks) {
    // Convert timer ticks to microseconds using the stored PWM clock frequency
    return (uint32_t)(((uint64_t)ticks * 1000000) / pwm_clock_frequency_hz);
}

// Track last pulse for each channel
static uint32_t last_pulse_us[NUM_RC_CHANNELS] = {0};

void common_cb(const struct device *dev, uint32_t channel,
               uint32_t period, uint32_t pulse, int status, void *user_data) {
    int ch = (int)(uintptr_t)user_data;
    const char *ch_name = (ch >= 0 && ch < NUM_RC_CHANNELS) ? rc_channel_names[ch] : "unknown";
    uint32_t us = ticks_to_us(pulse);

    // Only log if pulse changes significantly (more than 5 us)
    if (ch >= 0 && ch < NUM_RC_CHANNELS) {
        if (last_pulse_us[ch] == 0 || (us > last_pulse_us[ch] + 50) || (us + 50 < last_pulse_us[ch])) {
            LOG_DBG("PWM capture on %s (ch %d): period %d us, pulse %d us",
                    ch_name, ch, ticks_to_us(period), us);
            last_pulse_us[ch] = us;
        }
    }

    // Check for capture errors
    if (status != 0) {
        LOG_WRN("PWM capture error on %s (ch %d), status: %d", ch_name, ch, status);
        remote_link_lost(ch);
        return;
    }

    // Check for reasonable pulse width (0.5ms to 2.5ms for RC signals)
    if (pulse < 500 || pulse > 2500) {
        LOG_DBG("PWM pulse very large/small on %s (ch %d): %d us", ch_name, ch, pulse);
    }

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
                                        PWM_CAPTURE_TYPE_BOTH | PWM_CAPTURE_MODE_CONTINUOUS,
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
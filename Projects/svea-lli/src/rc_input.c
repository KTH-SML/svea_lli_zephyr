#include "rc_input.h"
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(rc_input, LOG_LEVEL_INF);

typedef struct {
    const struct device *dev;
    uint32_t channel;
    const char *name;
} rc_input_channel_t;

static const rc_input_channel_t rc_channels[NUM_RC_CHANNELS] = {
    [RC_STEER] = {
        .dev = DEVICE_DT_GET_OR_NULL(DT_ALIAS(rc_steer)),
        .channel = 1,
        .name = "rc-steer"},
    [RC_HIGH_GEAR] = {.dev = DEVICE_DT_GET_OR_NULL(DT_ALIAS(rc_high_gear)), .channel = 1, .name = "rc-high-gear"},
    [RC_THROTTLE] = {.dev = DEVICE_DT_GET_OR_NULL(DT_ALIAS(rc_throttle)), .channel = 1, .name = "rc-throttle"},
    [RC_OVERRIDE] = {.dev = DEVICE_DT_GET_OR_NULL(DT_ALIAS(rc_override)), .channel = 3, .name = "rc-override"}};

volatile rc_capture_ns_t _rc_pulse_ns[NUM_RC_CHANNELS];
volatile rc_capture_ns_t *rc_ns = _rc_pulse_ns;

static void rc_pwm_capture_callback(const struct device *dev, uint32_t channel,
                                    uint32_t period, uint32_t pulse, int status, void *user_data) {
    int idx = (int)(uintptr_t)user_data;
    pwm_cycles_to_nsec(dev, channel, pulse, &_rc_pulse_ns[idx].pulse_ns);
    pwm_cycles_to_nsec(dev, channel, period, &_rc_pulse_ns[idx].period_ns);
}

void rc_input_init(void) {
    memset((void *)rc_ns, 0, sizeof(_rc_pulse_ns));
    for (int i = 0; i < NUM_RC_CHANNELS; ++i) {
        if (!rc_channels[i].dev || !device_is_ready(rc_channels[i].dev)) {
            LOG_WRN("RC input %s device not ready", rc_channels[i].name);
            continue;
        }
        int ret = pwm_configure_capture(
            rc_channels[i].dev,
            rc_channels[i].channel,
            PWM_CAPTURE_TYPE_BOTH | PWM_CAPTURE_MODE_CONTINUOUS,
            rc_pwm_capture_callback,
            (void *)(uintptr_t)i);
        if (ret) {
            LOG_ERR("Failed to configure capture for %s: %d", rc_channels[i].name, ret);
            continue;
        }
        ret = pwm_enable_capture(rc_channels[i].dev, rc_channels[i].channel);
        if (ret) {
            LOG_ERR("Failed to enable capture for %s: %d", rc_channels[i].name, ret);
        } else {
            LOG_INF("Enabled PWM capture for %s", rc_channels[i].name);
        }
    }
}

void rc_input_logger_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    while (1) {
        for (int i = 0; i < NUM_RC_CHANNELS; ++i) {
            LOG_INF("RC ch %d (%s): pulse_ns=%llu, period_ns=%llu",
                    i, rc_channels[i].name,
                    rc_ns[i].pulse_ns, rc_ns[i].period_ns);
        }
        k_sleep(K_SECONDS(1));
    }
}

K_THREAD_DEFINE(rc_input_logger_tid, 1024, rc_input_logger_thread, NULL, NULL, NULL, 5, 0, 0);
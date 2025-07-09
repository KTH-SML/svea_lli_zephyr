#include "rc_input.h"
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
    [RC_STEER] = {.dev = DEVICE_DT_GET_OR_NULL(DT_ALIAS(rc_steer)), .channel = 1, .name = "rc-steer"},
    [RC_HIGH_GEAR] = {.dev = DEVICE_DT_GET_OR_NULL(DT_ALIAS(rc_high_gear)), .channel = 1, .name = "rc-high-gear"},
    [RC_THROTTLE] = {.dev = DEVICE_DT_GET_OR_NULL(DT_ALIAS(rc_throttle)), .channel = 1, .name = "rc-throttle"},
    [RC_OVERRIDE] = {.dev = DEVICE_DT_GET_OR_NULL(DT_ALIAS(rc_override)), .channel = 3, .name = "rc-override"}};

// Store latest values for each channel
volatile uint64_t rc_pulse[NUM_RC_CHANNELS];
volatile uint64_t rc_period[NUM_RC_CHANNELS];

void rc_input_channel_thread(void *p1, void *p2, void *p3) {
    int ch = (int)(intptr_t)p1;
    const rc_input_channel_t *rc = &rc_channels[ch];

    if (!rc->dev) {
        printk("RC input channel %d (%s): device not found!\n", ch, rc->name);
        return; // Exit thread if device is missing
    }
    while (1) {
        uint32_t start = k_uptime_get();
        int ret = pwm_capture_nsec(rc->dev, rc->channel, PWM_CAPTURE_TYPE_PULSE | PWM_CAPTURE_MODE_SINGLE, &rc_period[ch], &rc_pulse[ch], K_MSEC(1000));
        uint32_t end = k_uptime_get();
        uint32_t duration = end - start;
        printk("RC input channel %d (%s): capture took %u ms (ret=%d)\n", ch, rc->name, duration, ret);
        k_sleep(K_MSEC(8));
    }
}

void rc_input_logger_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    while (1) {
        for (int i = 0; i < NUM_RC_CHANNELS; ++i) {
            printk("RC ch %d (%s): pulse=%llu ns, period=%llu ns\n",
                   i, rc_channels[i].name,
                   (unsigned long long)rc_pulse[i],
                   (unsigned long long)rc_period[i]);
        }
        k_sleep(K_SECONDS(1));
    }
}

// Create one thread per channel
#define RC_INPUT_THREAD(idx)                                              \
    K_THREAD_DEFINE(rc_input_thread_##idx, 1024, rc_input_channel_thread, \
                    (void *)(intptr_t)(idx), NULL, NULL, 5, 0, 0);

RC_INPUT_THREAD(0)
RC_INPUT_THREAD(1)
RC_INPUT_THREAD(2)
RC_INPUT_THREAD(3)

// Logger thread (optional)
K_THREAD_DEFINE(rc_input_logger_tid, 1024, rc_input_logger_thread, NULL, NULL, NULL, 5, 0, 0);

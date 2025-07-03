#include "remote.h"
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>

/* ---- DT helpers: zero literals ------------------------------------ */
#define RC_NODE(n) DT_ALIAS(rc_##n)
#define RC_DEV(n) DEVICE_DT_GET(DT_PWMS_CTLR(RC_NODE(n)))
#define RC_CH(n) DT_PWMS_CHANNEL_BY_IDX(RC_NODE(n), 0)

BUILD_ASSERT(DT_NODE_HAS_STATUS(RC_NODE(steer), okay), "overlay missing");

static void cb(const struct device *d, uint32_t chan,
               uint32_t per, uint32_t pulse, int status, void *ud) {
    enum rc_chan idx = (enum rc_chan)(uintptr_t)ud;
    bool ov = status & PWM_CAPTURE_CB_OVERFLOW;
    uint16_t us = pwm_ticks_to_ns(d, pulse) / 1000;
    rc_report(idx, us, ov);
}

static int init_one(const struct device *dev, uint32_t ch, enum rc_chan idx) {
    struct pwm_capture_cfg c = {
        .flags = PWM_CAPTURE_TYPE_BOTH | PWM_CAPTURE_MODE_CONTINUOUS,
        .callback = cb,
        .user_data = (void *)(uintptr_t)idx,
    };
    return pwm_capture_enable(dev, ch, &c);
}

int rc_input_init(void) {
    return init_one(RC_DEV(steer), RC_CH(steer), RC_STEER) ||
           init_one(RC_DEV(gear), RC_CH(gear), RC_GEAR) ||
           init_one(RC_DEV(throttle), RC_CH(throttle), RC_THROTTLE) ||
           init_one(RC_DEV(override), RC_CH(override), RC_OVERRIDE);
}

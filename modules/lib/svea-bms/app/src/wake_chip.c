#include "wake_chip.h"
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(wake_chip, CONFIG_LOG_DEFAULT_LEVEL);

#define TS1_NODE DT_PATH(zephyr_user)
#if DT_NODE_HAS_PROP(TS1_NODE, ts1_wake_gpios)
static const struct gpio_dt_spec ts1 = GPIO_DT_SPEC_GET(TS1_NODE, ts1_wake_gpios);
static bool ts1_is_output;
#endif

int wake_chip_init(void)
{
#if !DT_NODE_HAS_PROP(TS1_NODE, ts1_wake_gpios)
    LOG_DBG("TS1 wake GPIO not defined; wake_chip disabled");
    return -ENOTSUP;
#else
    if (!device_is_ready(ts1.port)) {
        LOG_ERR("TS1 wake port not ready");
        return -ENODEV;
    }
    int ret = gpio_pin_configure_dt(&ts1, GPIO_OUTPUT_LOW); /* default low */
    if (ret) {
        LOG_ERR("gpio_pin_configure_dt failed: %d", ret);
        return ret;
    }
    ts1_is_output = true;
    return 0;
#endif
}

int wake_chip_pulse_ms(uint32_t ms)
{
#if !DT_NODE_HAS_PROP(TS1_NODE, ts1_wake_gpios)
    ARG_UNUSED(ms);
    return -ENOTSUP;
#else
    if (!device_is_ready(ts1.port)) {
        int r = wake_chip_init();
        if (r)
            return r;
    }
    if (!ts1_is_output) {
        int r = gpio_pin_configure_dt(&ts1, GPIO_OUTPUT_LOW);
        if (r)
            return r;
        ts1_is_output = true;
    }
    int ret = gpio_pin_set_dt(&ts1, 1);
    if (ret)
        return ret;

    /* Datasheet says >~10 µs; 10–50 ms is plenty */
    k_msleep(ms);

    return gpio_pin_set_dt(&ts1, 0);
#endif
}

int wake_chip_release(void)
{
#if !DT_NODE_HAS_PROP(TS1_NODE, ts1_wake_gpios)
    return -ENOTSUP;
#else
    if (!device_is_ready(ts1.port)) {
        int r = wake_chip_init();
        if (r)
            return r;
    }
    int ret = gpio_pin_configure_dt(&ts1, GPIO_INPUT);
    if (ret == 0)
        ts1_is_output = false;
    return ret;
#endif
}

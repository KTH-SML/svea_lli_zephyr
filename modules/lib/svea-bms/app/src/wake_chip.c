#include "wake_chip.h"
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(wake_chip, CONFIG_LOG_DEFAULT_LEVEL);

#define TS1_NODE DT_PATH(zephyr_user)
static const struct gpio_dt_spec ts1 = GPIO_DT_SPEC_GET(TS1_NODE, ts1_wake_gpios);
static bool ts1_is_output;

int wake_chip_init(void)
{
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
}

int wake_chip_pulse_ms(uint32_t ms)
{
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
}

int wake_chip_release(void)
{
    if (!device_is_ready(ts1.port)) {
        int r = wake_chip_init();
        if (r)
            return r;
    }
    int ret = gpio_pin_configure_dt(&ts1, GPIO_INPUT);
    if (ret == 0)
        ts1_is_output = false;
    return ret;
}

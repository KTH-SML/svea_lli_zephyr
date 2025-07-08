#include "control.h"
#include "rc_input.h"
// #include "ros_iface.h"
// #include "sensors.h"
// #include "servo.h"
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

// Add watchdog device
static const struct device *wdt_dev;
static int wdt_channel_id;

// Function to feed watchdog
static void feed_watchdog(void) {
    if (wdt_dev && device_is_ready(wdt_dev)) {
        wdt_feed(wdt_dev, wdt_channel_id);
    }
}

// Watchdog thread function
static void watchdog_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    if (wdt_channel_id < 0) {
        LOG_ERR("Watchdog thread started with invalid channel ID. Exiting.");
        return;
    }

    while (1) {
        k_msleep(1000);
        if (wdt_dev && device_is_ready(wdt_dev)) {
            wdt_feed(wdt_dev, wdt_channel_id);
        }
    }
}

// Place this at file scope, not inside main()
K_THREAD_DEFINE(wdt_tid, 1024, watchdog_thread, NULL, NULL, NULL, 1, 0, 0);

int main(void) {
    LOG_INF("SVEA LLI starting");

    // Watchdog setup
    wdt_dev = DEVICE_DT_GET(DT_ALIAS(watchdog0));
    if (!device_is_ready(wdt_dev)) {
        LOG_ERR("Watchdog device not ready");
        return 0;
    }

    struct wdt_timeout_cfg wdt_config = {
        .window.min = 0,
        .window.max = 3000, // 3 seconds
        .callback = NULL,
        .flags = WDT_FLAG_RESET_SOC,
    };

    wdt_channel_id = wdt_install_timeout(wdt_dev, &wdt_config);
    if (wdt_channel_id < 0) {
        LOG_ERR("Watchdog install timeout failed: %d", wdt_channel_id);
        return 0;
    }

    wdt_setup(wdt_dev, WDT_OPT_PAUSE_HALTED_BY_DBG);

    // Initialise modules
    servo_init();

    rc_input_init();
    // sensors_init();
    //  ros_iface_init(); // <-- Fix: use correct function name

    return 0;
}
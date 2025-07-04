#include "control.h"
#include "rc_input.h"
#include "remote.h"
#include "ros_iface.h"
#include "sensors.h"
#include "servo.h"
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

    while (1) {
        feed_watchdog();
        k_sleep(K_MSEC(500));
    }
}

int main(void) {
    LOG_INF("Starting SVEA LLI Control System");

    // Initialize watchdog
    wdt_dev = DEVICE_DT_GET(DT_ALIAS(watchdog0));
    if (!device_is_ready(wdt_dev)) {
        LOG_ERR("Watchdog device not ready");
    } else {
        struct wdt_timeout_cfg wdt_config = {
            .window.min = 0,
            .window.max = 1000, // 1 second timeout
            .callback = NULL,
            .flags = WDT_FLAG_RESET_SOC};

        wdt_channel_id = wdt_install_timeout(wdt_dev, &wdt_config);
        if (wdt_channel_id < 0) {
            LOG_ERR("Watchdog install failed: %d", wdt_channel_id);
        } else {
            wdt_setup(wdt_dev, WDT_OPT_PAUSE_HALTED_BY_DBG);
            LOG_INF("Watchdog initialized with %d ms timeout", wdt_config.window.max);
        }
    }

    // Initialize all subsystems
    remote_init();
    servo_init();
    rc_input_init();
    sensors_init();
    // ros_iface_init();

    // Start control system
    control_start();

    LOG_INF("All systems initialized, entering main loop");

    // Main thread can sleep forever - control happens in other threads
    while (1) {
        k_sleep(K_FOREVER);
    }

    return 0;
}

// Start watchdog thread
K_THREAD_DEFINE(wdt_tid, 1024, watchdog_thread, NULL, NULL, NULL, 0, 0, 0);
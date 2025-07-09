#include "control.h"
#include "rc_input.h"
// #include "ros_iface.h"
// #include "sensors.h"
// #include "servo.h"
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

int main(void) {
    LOG_INF("SVEA LLI starting");
    /* Brief sleep to allow for peripherals to initialize */
    k_msleep(1000);

    // Initialise modules
    servo_init();

    // rc_input_init();
    // sensors_init();
    //  ros_iface_init(); // <-- Fix: use correct function name

    return 0;
}
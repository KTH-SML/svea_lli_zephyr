#include "control.h"
#include "rc_input.h"
#include "remote.h"
#include "ros_iface.h"
#include "sensors.h"
#include "servo.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

int main(void) {
    LOG_INF("Starting SVEA LLI Control System");

    // Initialize all subsystems
    remote_init();
    servo_init();
    rc_input_init();
    sensors_init();
    ros_iface_init();

    // Start control system
    control_start();

    LOG_INF("All systems initialized, entering main loop");

    // Main thread can sleep forever - control happens in other threads
    while (1) {
        k_sleep(K_FOREVER);
    }

    return 0;
}
/* Micro-ROS Zephyr base node.
 * - Uses the custom UART transport implemented in zephyr_transport_*.c
 * - Publishes a short diagnostic line at PUBLISH_HZ.              */

#include <version.h>
#if ZEPHYR_VERSION_CODE >= ZEPHYR_VERSION(3, 1, 0)
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#else
#include <device.h>
#include <zephyr.h>
#endif

#include "threads/control/pwm/pwm.h"
#include "threads/sensor/imu/imu_publisher.h"
#include "threads/util/diag/diag_publisher.h"

#include <microros_transports.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/string.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(void) {
    printf("micro‑ROS Zephyr base node\n");
    pwm_in_init();

    for (;;) {
        rmw_uros_set_custom_transport(MICRO_ROS_FRAMING_REQUIRED,
                                      (void *)&default_params,
                                      zephyr_transport_open,
                                      zephyr_transport_close,
                                      zephyr_transport_write,
                                      zephyr_transport_read);

        rcl_allocator_t allocator = rcl_get_default_allocator();
        rclc_support_t support;
        rcl_node_t node;
        rclc_executor_t executor;

        if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
            k_sleep(K_MSEC(100));
            continue;
        }
        if (rclc_node_init_default(&node, "zephyr_base_node", "", &support) != RCL_RET_OK) {
            rclc_support_fini(&support);
            k_sleep(K_MSEC(100));
            continue;
        }

        // Initialize executor BEFORE starting IMU or servo
        rclc_executor_init(&executor, &support.context, atoi(CONFIG_MICROROS_PUBLISHERS) + atoi(CONFIG_MICROROS_SUBSCRIBERS), &allocator);

        // Start diagnostic publisher thread
        diag_publisher_init(&node);

        // Start IMU publisher thread
        imu_publisher_init(&node, &executor);

        servo_subscribers_init(&node, &executor);

        pwm_in_publishers_init(&node);

        printf("micro‑ROS ready\n");

        while (1) {
            rclc_executor_spin_some(&executor, 0);
            k_sleep(K_MSEC(1));
        }

    cleanup:
        printf("micro‑ROS error – restarting setup…\n");
        rclc_executor_fini(&executor);
        rcl_node_fini(&node);
        rclc_support_fini(&support);
        k_sleep(K_MSEC(100));
    }
    return 0;
}

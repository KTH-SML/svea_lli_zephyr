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

#include "threads/diag/diag_publisher.h"
#include "threads/imu/imu_publisher.h"
#include "threads/servo/servo_sweeper.h"
#include <microros_transports.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/string.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RCCHECK(fn)                                                  \
    do {                                                             \
        rcl_ret_t _rc = (fn);                                        \
        if (_rc != RCL_RET_OK) {                                     \
            printf("rcl error %d at line %d\n", (int)_rc, __LINE__); \
            goto cleanup;                                            \
        }                                                            \
    } while (0)
#define RCSOFTCHECK(fn)                                                   \
    do {                                                                  \
        rcl_ret_t _rc = (fn);                                             \
        if (_rc != RCL_RET_OK) {                                          \
            printf("rcl soft error %d at line %d\n", (int)_rc, __LINE__); \
        }                                                                 \
    } while (0)

int main(void) {
    printf("micro‑ROS Zephyr base node\n");

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
            k_sleep(K_MSEC(1000));
            continue;
        }
        if (rclc_node_init_default(&node, "zephyr_base_node", "", &support) != RCL_RET_OK) {
            rclc_support_fini(&support);
            k_sleep(K_MSEC(1000));
            continue;
        }

        // Start diagnostic publisher thread
        diag_publisher_init(&node);

        // Start IMU publisher thread
        imu_publisher_init(&node, &executor);

        // Start the servo sweeper thread
        k_thread_create(&servo_sweeper_thread_data, servo_sweeper_stack, 1024,
                        servo_sweeper_thread, NULL, NULL, NULL,
                        5, 0, K_NO_WAIT);

        printf("micro‑ROS ready\n");

        while (1) {
            k_sleep(K_MSEC(1000));
        }

    cleanup:
        printf("micro‑ROS error – restarting setup…\n");
        rclc_executor_fini(&executor);
        rcl_node_fini(&node);
        rclc_support_fini(&support);
        k_sleep(K_MSEC(1000));
    }
    return 0;
}

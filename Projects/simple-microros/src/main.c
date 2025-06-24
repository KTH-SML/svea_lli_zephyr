/* Minimal, clean micro‑ROS string publisher for Zephyr.
 * ‑ Uses the custom UART transport implemented in zephyr_transport_*.c
 * ‑ Publishes a short diagnostic line at PUBLISH_HZ.              */

#include <version.h>

#if ZEPHYR_VERSION_CODE >= ZEPHYR_VERSION(3, 1, 0)
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/posix/time.h>
#else
#include <device.h>
#include <devicetree.h>
#include <posix/time.h>
#include <zephyr.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <microros_transports.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/string.h>

#define RCCHECK(fn)                                                  \
    do {                                                             \
        rcl_ret_t _rc = (fn);                                        \
        if (_rc != RCL_RET_OK) {                                     \
            printf("rcl error %d at line %d\n", (int)_rc, __LINE__); \
            return;                                                  \
        }                                                            \
    } while (0)

#define PUBLISH_HZ 1000U

static rcl_publisher_t publisher;
static std_msgs__msg__String msg;

/* Ring‑buffer helper until Zephyr 3.3 provides ring_buf_size_get(). */
static inline uint32_t ring_buf_used(const struct ring_buf *rb) {
    return rb->size - ring_buf_space_get(rb);
}

static void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    ARG_UNUSED(last_call_time);
    static uint32_t cnt;
    if (!timer) {
        return;
    }

    /* Build diagnostic string into a static buffer (msg points to it). */
    static char str[96];
    uint32_t uptime_ms = k_uptime_get_32();
    extern struct ring_buf in_ringbuf;
    uint32_t rb_used = ring_buf_used(&in_ringbuf);

    int n = snprintf(str, sizeof str,
                     "cnt=%lu | uptime=%lu ms | ring_used=%u",
                     (unsigned long)cnt++, (unsigned long)uptime_ms, rb_used);
    if (n < 0) {
        return; /* encoding error */
    }

    msg.data.data = str;
    msg.data.size = (size_t)n;
    msg.data.capacity = sizeof str;

    rcl_ret_t rc = rcl_publish(&publisher, &msg, NULL);
    if (rc != RCL_RET_OK) {
        printf("Publish failed: %d\n", rc);
    }
}

int main(void) {
    for (;;) {
        /* Register custom UART transport. */
        rmw_uros_set_custom_transport(MICRO_ROS_FRAMING_REQUIRED,
                                      (void *)&default_params,
                                      zephyr_transport_open,
                                      zephyr_transport_close,
                                      zephyr_transport_write,
                                      zephyr_transport_read);

        rcl_allocator_t allocator = rcl_get_default_allocator();
        rclc_support_t support;
        if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
            k_sleep(K_MSEC(1000));
            continue;
        }

        rcl_node_t node;
        if (rclc_node_init_default(&node, "zephyr_string_publisher", "", &support) != RCL_RET_OK) {
            rclc_support_fini(&support);
            k_sleep(K_MSEC(1000));
            continue;
        }

        if (rclc_publisher_init_default(&publisher, &node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                                        "zephyr_string_publisher") != RCL_RET_OK) {
            rcl_node_fini(&node);
            rclc_support_fini(&support);
            k_sleep(K_MSEC(1000));
            continue;
        }

        /* 1 kHz timer. */
        const uint32_t period_ms = 1000U / PUBLISH_HZ;
        rcl_timer_t timer;
        if (rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(period_ms), timer_callback) != RCL_RET_OK) {
            rcl_publisher_fini(&publisher, &node);
            rcl_node_fini(&node);
            rclc_support_fini(&support);
            k_sleep(K_MSEC(1000));
            continue;
        }

        rclc_executor_t executor;
        if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK ||
            rclc_executor_add_timer(&executor, &timer) != RCL_RET_OK) {
            rcl_timer_fini(&timer);
            rcl_publisher_fini(&publisher, &node);
            rcl_node_fini(&node);
            rclc_support_fini(&support);
            k_sleep(K_MSEC(1000));
            continue;
        }

        printf("micro‑ROS ready: publishing at %u Hz\n", PUBLISH_HZ);

        while (rclc_executor_spin_some(&executor, 1) == RCL_RET_OK) {
            /* run forever */
        }

        /* Tear down on error and retry in 1 s. */
        printf("micro‑ROS error – restarting setup…\n");
        rclc_executor_fini(&executor);
        rcl_timer_fini(&timer);
        rcl_publisher_fini(&publisher, &node);
        rcl_node_fini(&node);
        rclc_support_fini(&support);
        k_sleep(K_MSEC(1000));
    }
    return 0;
}

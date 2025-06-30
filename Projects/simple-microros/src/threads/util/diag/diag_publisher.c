#include "diag_publisher.h"
#include <stdio.h>
#include <string.h>
#include <zephyr/sys/printk.h>

#ifndef RCCHECK
#define RCCHECK(fn)                                                  \
    do {                                                             \
        rcl_ret_t _rc = (fn);                                        \
        if (_rc != RCL_RET_OK) {                                     \
            printf("rcl error %d at line %d\n", (int)_rc, __LINE__); \
            goto cleanup;                                            \
        }                                                            \
    } while (0)
#endif
#ifndef RCSOFTCHECK
#define RCSOFTCHECK(fn)                                                   \
    do {                                                                  \
        rcl_ret_t _rc = (fn);                                             \
        if (_rc != RCL_RET_OK) {                                          \
            printf("rcl soft error %d at line %d\n", (int)_rc, __LINE__); \
        }                                                                 \
    } while (0)
#endif

K_THREAD_STACK_DEFINE(diag_stack, DIAG_THREAD_STACK_SIZE);
struct k_thread diag_thread_data;

std_msgs__msg__String diag_msg;
rcl_publisher_t diag_publisher;

static void diag_publish_loop(void) {
    static uint32_t cnt;
    static char str[96];
    extern struct ring_buf in_ringbuf;
    uint32_t uptime_ms = k_uptime_get_32();
    uint32_t rb_used = in_ringbuf.size - ring_buf_space_get(&in_ringbuf);

    int n = snprintf(str, sizeof str,
                     "cnt=%lu | uptime=%lu ms | ring_used=%u",
                     (unsigned long)cnt++, (unsigned long)uptime_ms, rb_used);
    if (n < 0)
        return;

    diag_msg.data.data = str;
    diag_msg.data.size = (size_t)n;
    diag_msg.data.capacity = sizeof str;

    rcl_publish(&diag_publisher, &diag_msg, NULL);
}

void diag_thread(void *arg1, void *arg2, void *arg3) {
    while (1) {
        diag_publish_loop();
        k_sleep(K_MSEC(1000 / DIAG_PUBLISH_HZ));
    }
}

void diag_publisher_init(rcl_node_t *node) {
    rcl_publisher_options_t pub_ops = rcl_publisher_get_default_options();
    pub_ops.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    rcl_ret_t rc = rcl_publisher_init(
        &diag_publisher, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        DIAG_PUBLISHER_TOPIC, &pub_ops);
    if (rc != RCL_RET_OK) {
        printk("Diag publisher init failed: %d\n", rc);
        return;
    }
    k_thread_create(&diag_thread_data, diag_stack, DIAG_THREAD_STACK_SIZE,
                    diag_thread, NULL, NULL, NULL,
                    DIAG_THREAD_PRIORITY, 0, K_NO_WAIT);
}
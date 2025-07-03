#include "ros_iface.h"
#include <microros_transports/serial_transport.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>

struct k_poll_signal ros_sig = K_POLL_SIGNAL_INITIALIZER(ros_sig);
volatile bool ros_cmd_ready = false;
uint8_t ros_cmd_duty[4]; /* diff, steer, gear, throttle */

static rcl_publisher_t pub;
static rcl_subscription_t sub;
static rclc_executor_t exec;

static void sub_cb(const void *msg_in) {
    const std_msgs__msg__UInt8MultiArray *m = msg_in;
    if (m->data.size >= 4) {
        ros_cmd_duty[0] = m->data.data[0];
        ros_cmd_duty[1] = m->data.data[1];
        ros_cmd_duty[2] = m->data.data[2];
        ros_cmd_duty[3] = m->data.data[3];
        ros_cmd_ready = true;
        k_poll_signal_raise(&ros_sig, 0);
    }
}

static void spin_fn(void *a, void *b, void *c) {
    while (1) {
        if (rmw_uros_ping_agent(50, 2) != RMW_RET_OK) {
            k_sleep(K_SECONDS(1));
            continue;
        }

        /* --- rebuild entities each reconnect ----------------------- */
        rcl_allocator_t alloc = rcl_get_default_allocator();
        rcl_node_t node;
        rclc_node_init_default(&node, "zephyr", "", &alloc);
        rclc_publisher_init_best_effort(
            &pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray),
            "/lli/remote/frame");

        rclc_subscription_init_default(
            &sub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray),
            "/lli/ctrl/servo");

        rclc_executor_init(&exec, &node, 1, &alloc);
        rclc_executor_add_subscription(&exec, &sub, NULL, sub_cb, ON_NEW_DATA);

        while (rclc_executor_spin_some(&exec, RCL_MS_TO_NS(10)) == RCL_RET_OK)
            ;

        rclc_executor_fini(&exec);
        rcl_subscription_fini(&sub, &node);
        rcl_publisher_fini(&pub, &node);
        rcl_node_fini(&node);
    }
}

int ros_iface_start(void) {
    microros_transport_serial_init("/dev/ttyACM0", 115200);
    static uint8_t st[2048];
    k_thread_create(&(struct k_thread){}, st, sizeof st,
                    spin_fn, NULL, NULL, NULL, 3, 0, K_NO_WAIT);
    return 0;
}

void ros_publish_rc(const struct RcFrame *f) {
    uint8_t data[4] = {f->steer, f->gear, f->throttle, f->override_raw};
    std_msgs__msg__UInt8MultiArray m = {0};
    m.data.data = data;
    m.data.size = m.data.capacity = 4;
    (void)rcl_publish(&pub, &m, NULL);
}

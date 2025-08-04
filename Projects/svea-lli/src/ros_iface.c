#include "ros_iface.h"
#include "control.h"
#include "rc_input.h"

#include <geometry_msgs/msg/twist.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/u_int8.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/posix/time.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int8.h>

#include <rclc/executor.h>
#include <rclc/rclc.h>

#include <microros_transports.h>
#include <rmw_microros/rmw_microros.h>

LOG_MODULE_REGISTER(ros_iface, LOG_LEVEL_INF);
ros_ctrl_t g_ros_ctrl = {0};
#define ROS_STACK_SIZE 4096

bool ros_initialized = false;
enum ros_states state = ROS_WAITING_AGENT;

static K_THREAD_STACK_DEFINE(ros_stack, ROS_STACK_SIZE);
static struct k_thread ros_thread;

static rclc_support_t support;
static rcl_node_t node;
static rclc_executor_t executor;

// Publishers
static rcl_publisher_t pub_steer, pub_throttle, pub_gear, pub_override, pub_connected;
static std_msgs__msg__Int8 msg_steer, msg_throttle;
static std_msgs__msg__Bool msg_gear, msg_override;

// Subscriptions
static rcl_subscription_t sub_steer, sub_throttle, sub_gear, sub_diff;
static std_msgs__msg__Int8 submsg_steer, submsg_throttle;
static std_msgs__msg__Bool submsg_gear, submsg_diff;

// IMU Publisher
rcl_publisher_t imu_pub;

// Encoder Publisher
rcl_publisher_t encoders_pub;

// Time synchronization variables
static int64_t synced_epoch_ns = 0;
static int64_t synced_epoch_ms = 0;
static uint64_t synced_uptime_ms = 0;
static struct k_thread time_sync_thread_data;
K_THREAD_STACK_DEFINE(time_sync_stack, 2048);

// Map pulse [1000,2000] to int8 [-127,127]
static inline int8_t pulse_to_int8(int32_t us) {
    if (us < 1000)
        us = 1000;
    if (us > 2000)
        us = 2000;
    return (int8_t)(((us - 1500) * 127) / 500);
}
// Map pulse to bool (>1500 true)
static inline bool pulse_to_bool(uint32_t us) {
    return us > 1500;
}
// Subscription callbacks
static void steer_cb(const void *msg) {
    int8_t value = ((std_msgs__msg__Int8 *)msg)->data;
    LOG_DBG("Received steering command: %d", value);
    g_ros_ctrl.steering = value;
}

static void throttle_cb(const void *msg) {
    int8_t value = ((std_msgs__msg__Int8 *)msg)->data;
    LOG_DBG("Received throttle command: %d", value);
    g_ros_ctrl.throttle = value;
    g_ros_ctrl.timestamp = k_uptime_get(); // Update timestamp on throttle change, important
}

static void gear_cb(const void *msg) {
    bool value = ((std_msgs__msg__Bool *)msg)->data;
    LOG_DBG("Received high gear command: %s", value ? "true" : "false");
    g_ros_ctrl.high_gear = value;
}

static void diff_cb(const void *msg) {
    bool value = ((std_msgs__msg__Bool *)msg)->data;
    LOG_DBG("Received diff command: %s", value ? "true" : "false");
    g_ros_ctrl.diff = value;
}

static uint64_t epoch_off_ns; /* agent_epoch ‑ local_uptime */
static atomic_t epoch_locked = ATOMIC_INIT(0);
// Sync time thread: updates virtual clock every second
static void time_sync_thread(void *a, void *b, void *c) {

    while (1) {
        if (ROS_AGENT_CONNECTED != state) {
            k_sleep(K_SECONDS(1));
            continue;
        }
        rmw_ret_t rc = rmw_uros_sync_session(200);
        if (rc != RMW_RET_OK) {
            state = ROS_AGENT_DISCONNECTED;
            LOG_WRN("Failed to sync session: %d", rc);
            k_sleep(K_MSEC(500));
            continue;
        }
        if (rmw_uros_epoch_synchronized() &&
            !atomic_test_and_set_bit(&epoch_locked, 0)) {

            uint64_t agent_ns = rmw_uros_epoch_nanos();
            uint64_t up_ns = k_uptime_get() * 1000000ULL;

            /* disable IRQs: atomic on 32‑bit CPU */
            unsigned int key = irq_lock();
            epoch_off_ns = agent_ns - up_ns;
            irq_unlock(key);

            LOG_INF("Time locked to agent, offset = %lld ns",
                    (long long)epoch_off_ns);
        }
        k_sleep(K_SECONDS(1)); /* lower bus traffic */
    }
}

// --- Add: Helper functions for entity management ---

static bool create_ros_entities(void) {
    rcl_ret_t rc;
    rcl_allocator_t allocator = rcl_get_default_allocator();

    rc = rclc_support_init(&support, 0, NULL, &allocator);
    if (rc != RCL_RET_OK)
        return false;

    rc = rclc_node_init_default(&node, "svea_lli_node", "", &support);
    if (rc != RCL_RET_OK)
        goto fail_node;

    rc = rclc_publisher_init_best_effort(&pub_steer, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), "/lli/remote/steering");
    if (rc != RCL_RET_OK)
        goto fail_pub;
    rc = rclc_publisher_init_best_effort(&pub_throttle, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), "/lli/remote/throttle");
    if (rc != RCL_RET_OK)
        goto fail_pub;
    rc = rclc_publisher_init_best_effort(&pub_gear, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/lli/remote/high_gear");
    if (rc != RCL_RET_OK)
        goto fail_pub;
    rc = rclc_publisher_init_best_effort(&pub_override, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/lli/remote/override");
    if (rc != RCL_RET_OK)
        goto fail_pub;
    rc = rclc_publisher_init_best_effort(&pub_connected, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/lli/remote/connected");
    if (rc != RCL_RET_OK)
        goto fail_pub;
    rc = rclc_publisher_init_best_effort(&imu_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/lli/sensor/imu");
    if (rc != RCL_RET_OK)
        goto fail_pub;
    rc = rclc_publisher_init_best_effort(&encoders_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistWithCovarianceStamped), "/lli/sensor/encoders");
    if (rc != RCL_RET_OK)
        goto fail_pub;

    rc = rclc_subscription_init_best_effort(&sub_steer, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), "/lli/ctrl/steering");
    if (rc != RCL_RET_OK)
        goto fail_pub;
    rc = rclc_subscription_init_best_effort(&sub_throttle, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), "/lli/ctrl/throttle");
    if (rc != RCL_RET_OK)
        goto fail_pub;
    rc = rclc_subscription_init_best_effort(&sub_gear, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/lli/ctrl/high_gear");
    if (rc != RCL_RET_OK)
        goto fail_pub;
    rc = rclc_subscription_init_best_effort(&sub_diff, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/lli/ctrl/diff");
    if (rc != RCL_RET_OK)
        goto fail_pub;

    rc = rclc_executor_init(&executor, &support.context, 5, support.allocator);
    if (rc != RCL_RET_OK)
        goto fail_pub;
    rc = rclc_executor_add_subscription(&executor, &sub_steer, &submsg_steer, steer_cb, ON_NEW_DATA);
    if (rc != RCL_RET_OK)
        goto fail_pub;
    rc = rclc_executor_add_subscription(&executor, &sub_throttle, &submsg_throttle, throttle_cb, ON_NEW_DATA);
    if (rc != RCL_RET_OK)
        goto fail_pub;
    rc = rclc_executor_add_subscription(&executor, &sub_gear, &submsg_gear, gear_cb, ON_NEW_DATA);
    if (rc != RCL_RET_OK)
        goto fail_pub;
    rc = rclc_executor_add_subscription(&executor, &sub_diff, &submsg_diff, diff_cb, ON_NEW_DATA);
    if (rc != RCL_RET_OK)
        goto fail_pub;

    return true;

fail_pub:
    rcl_node_fini(&node);
fail_node:
    rclc_support_fini(&support);
    return false;
}

static void destroy_ros_entities(void) {
    /* disable the reliable erase handshake */
    rmw_uros_set_context_entity_destroy_session_timeout(
        rcl_context_get_rmw_context(&support.context), 0);

    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

static void ros_iface_thread(void *a, void *b, void *c) {
    LOG_INF("ros_iface_thread: started");
    k_msleep(1000); // Allow time for other subsystems to initialize

    rmw_uros_set_custom_transport(
        MICRO_ROS_FRAMING_REQUIRED,
        (void *)&default_params,
        zephyr_transport_open,
        zephyr_transport_close,
        zephyr_transport_write,
        zephyr_transport_read);

    ros_initialized = false;

    const uint32_t pub_period_ms = 50;
    uint64_t last_pub_time = 0;

    while (1) {
        ros_initialized = false;
        switch (state) {
        case ROS_WAITING_AGENT:
            if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
                state = ROS_AGENT_AVAILABLE;
                LOG_INF("micro-ROS agent available");
            } else {
                LOG_INF("Waiting for micro-ROS agent...");
                k_msleep(500);
            }
            break;

        case ROS_AGENT_AVAILABLE:
            if (create_ros_entities()) {
                state = ROS_AGENT_CONNECTED;
                LOG_INF("micro-ROS entities created");
                ros_initialized = false;
                last_pub_time = k_uptime_get();
            } else {
                state = ROS_WAITING_AGENT;
                LOG_ERR("Failed to create micro-ROS entities");
                destroy_ros_entities();
                k_msleep(1000);
            }
            break;

        case ROS_AGENT_CONNECTED:
            // Assume connected; spin executor and publish at regular intervals
            ros_initialized = true;
            rclc_executor_spin_some(&executor, 10);
            k_msleep(1);

            uint64_t now = k_uptime_get();
            if ((now - last_pub_time) >= pub_period_ms) {
                last_pub_time = now;
                bool publish_ok = true;
                rcl_ret_t rc;

                // Publish remote controls if available
                if (remote_connected) {
                    msg_steer.data = pulse_to_int8(rc_get_pulse_us(RC_STEER));
                    msg_throttle.data = pulse_to_int8(rc_get_pulse_us(RC_THROTTLE));
                    msg_gear.data = pulse_to_bool(rc_get_pulse_us(RC_HIGH_GEAR));
                    msg_override.data = pulse_to_bool(rc_get_pulse_us(RC_OVERRIDE));

                    rc = rcl_publish(&pub_steer, &msg_steer, NULL);
                    if (rc != RCL_RET_OK)
                        publish_ok = false;
                    rc = rcl_publish(&pub_throttle, &msg_throttle, NULL);
                    if (rc != RCL_RET_OK)
                        publish_ok = false;
                    rc = rcl_publish(&pub_gear, &msg_gear, NULL);
                    if (rc != RCL_RET_OK)
                        publish_ok = false;
                    rc = rcl_publish(&pub_override, &msg_override, NULL);
                    if (rc != RCL_RET_OK)
                        publish_ok = false;
                }

                // Always publish connection status
                std_msgs__msg__Bool msg_conn = {.data = remote_connected};
                rc = rcl_publish(&pub_connected, &msg_conn, NULL);
                if (rc != RCL_RET_OK)
                    publish_ok = false;

                if (!publish_ok) {
                    LOG_ERR("Publish failed, assuming agent disconnected");
                    // destroy_ros_entities();
                    ros_initialized = false;
                    state = ROS_AGENT_DISCONNECTED;
                }
            }
            break;

        case ROS_AGENT_DISCONNECTED:
            LOG_INF("micro-ROS agent disconnected, destroying entities");
            destroy_ros_entities();
            ros_initialized = false;
            state = ROS_WAITING_AGENT;
            LOG_INF("Destroyed micro-ROS entities, waiting for agent");
            k_msleep(1000);
            break;
        }
    }
}

uint64_t ros_iface_epoch_millis(void) {
    unsigned int key = irq_lock(); // atomic copy
    uint64_t off = epoch_off_ns;
    irq_unlock(key);

    return k_uptime_get() + off / 1000000ULL;
}

uint64_t ros_iface_epoch_nanos(void) /* new helper: epoch ns */
{
    unsigned int key = irq_lock(); /* atomic copy       */
    uint64_t off = epoch_off_ns;
    irq_unlock(key);

    return k_uptime_get() * 1000000ULL + off;
}

// Call this in ros_iface_init
void ros_iface_init(void) {
    k_thread_create(&ros_thread, ros_stack, K_THREAD_STACK_SIZEOF(ros_stack),
                    ros_iface_thread, NULL, NULL, NULL,
                    5, 0, K_NO_WAIT);
    k_thread_name_set(&ros_thread, "ros_iface");

    k_thread_create(&time_sync_thread_data, time_sync_stack, K_THREAD_STACK_SIZEOF(time_sync_stack),
                    time_sync_thread, NULL, NULL, NULL,
                    4, 0, K_NO_WAIT);
    k_thread_name_set(&time_sync_thread_data, "time_sync");
}

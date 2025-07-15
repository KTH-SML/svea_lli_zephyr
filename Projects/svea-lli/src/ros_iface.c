#include "ros_iface.h"
#include "control.h"
#include "rc_input.h"

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

static void ros_iface_thread(void *a, void *b, void *c) {
    LOG_INF("ros_iface_thread: started");
    k_msleep(1000); // Allow time for other subsystems to initialize
    // --- ADD THIS BLOCK: Micro-ROS transport setup ---
    rmw_uros_set_custom_transport(
        MICRO_ROS_FRAMING_REQUIRED,
        (void *)&default_params,
        zephyr_transport_open,
        zephyr_transport_close,
        zephyr_transport_write,
        zephyr_transport_read);
    // -------------------------------------------------

    rcl_ret_t rc;

    LOG_INF("ros_iface_thread: initializing rclc_support");
    while (1) {
        rcl_allocator_t allocator = rcl_get_default_allocator();
        rc = rclc_support_init(&support, 0, NULL, &allocator);
        if (rc != RCL_RET_OK) {
            LOG_ERR("rclc_support_init failed: %d", rc);
            k_msleep(1000);
            continue;
        }

        rc = rclc_node_init_default(&node, "svea_lli_node", "", &support);
        if (rc != RCL_RET_OK) {
            LOG_ERR("rclc_node_init_default failed: %d", rc);
            rclc_support_fini(&support);
            k_msleep(1000);
            continue;
        }

        // Publishers
        rc = rclc_publisher_init_best_effort(&pub_steer, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), "/lli/remote/steering");
        if (rc != RCL_RET_OK) {
            LOG_ERR("rclc_publisher_init_best_effort (pub_steer) failed: %d", rc);
            rcl_node_fini(&node);
            rclc_support_fini(&support);
            k_msleep(1000);
            continue;
        }
        rc = rclc_publisher_init_best_effort(&pub_throttle, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), "/lli/remote/throttle");
        if (rc != RCL_RET_OK) {
            LOG_ERR("rclc_publisher_init_best_effort (pub_throttle) failed: %d", rc);
            rcl_node_fini(&node);
            rclc_support_fini(&support);
            k_msleep(1000);
            continue;
        }
        rc = rclc_publisher_init_best_effort(&pub_gear, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/lli/remote/high_gear");
        if (rc != RCL_RET_OK) {
            LOG_ERR("rclc_publisher_init_best_effort (pub_gear) failed: %d", rc);
            rcl_node_fini(&node);
            rclc_support_fini(&support);
            k_msleep(1000);
            continue;
        }
        rc = rclc_publisher_init_best_effort(&pub_override, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/lli/remote/override");
        if (rc != RCL_RET_OK) {
            LOG_ERR("rclc_publisher_init_best_effort (pub_override) failed: %d", rc);
            rcl_node_fini(&node);
            rclc_support_fini(&support);
            k_msleep(1000);
            continue;
        }
        rc = rclc_publisher_init_best_effort(&pub_connected, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/lli/remote/connected");
        if (rc != RCL_RET_OK) {
            LOG_ERR("rclc_publisher_init_best_effort (pub_connected) failed: %d", rc);
            rcl_node_fini(&node);
            rclc_support_fini(&support);
            k_msleep(1000);
            continue;
        }
        // Sensors pubs

        rc = rclc_publisher_init_best_effort(&imu_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/lli/sensor/imu");
        if (rc != RCL_RET_OK) {
            LOG_ERR("rclc_publisher_init_best_effort (pub_imu) failed: %d", rc);
            rcl_node_fini(&node);
            rclc_support_fini(&support);
            k_msleep(1000);
            continue;
        }
        // Subscriptions
        rc = rclc_subscription_init_best_effort(&sub_steer, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), "/lli/ctrl/steering");
        if (rc != RCL_RET_OK) {
            LOG_ERR("rclc_subscription_init_best_effort (sub_steer) failed: %d", rc);
            rcl_node_fini(&node);
            rclc_support_fini(&support);
            k_msleep(1000);
            continue;
        }
        rc = rclc_subscription_init_best_effort(&sub_throttle, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), "/lli/ctrl/throttle");
        if (rc != RCL_RET_OK) {
            LOG_ERR("rclc_subscription_init_best_effort (sub_throttle) failed: %d", rc);
            rcl_node_fini(&node);
            rclc_support_fini(&support);
            k_msleep(1000);
            continue;
        }
        rc = rclc_subscription_init_best_effort(&sub_gear, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/lli/ctrl/high_gear");
        if (rc != RCL_RET_OK) {
            LOG_ERR("rclc_subscription_init_best_effort (sub_gear) failed: %d", rc);
            rcl_node_fini(&node);
            rclc_support_fini(&support);
            k_msleep(1000);
            continue;
        }
        rc = rclc_subscription_init_best_effort(&sub_diff, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/lli/ctrl/diff");
        if (rc != RCL_RET_OK) {
            LOG_ERR("rclc_subscription_init_best_effort (sub_diff) failed: %d", rc);
            rcl_node_fini(&node);
            rclc_support_fini(&support);
            k_msleep(1000);
            continue;
        }

        rc = rclc_executor_init(&executor, &support.context, 5, support.allocator);
        if (rc != RCL_RET_OK) {
            LOG_ERR("rclc_executor_init failed: %d", rc);
            rcl_node_fini(&node);
            rclc_support_fini(&support);
            k_msleep(1000);
            continue;
        }
        rc = rclc_executor_add_subscription(&executor, &sub_steer, &submsg_steer, steer_cb, ON_NEW_DATA);
        if (rc != RCL_RET_OK) {
            LOG_ERR("rclc_executor_add_subscription (sub_steer) failed: %d", rc);
            rcl_node_fini(&node);
            rclc_support_fini(&support);
            k_msleep(1000);
            continue;
        }
        rc = rclc_executor_add_subscription(&executor, &sub_throttle, &submsg_throttle, throttle_cb, ON_NEW_DATA);
        if (rc != RCL_RET_OK) {
            LOG_ERR("rclc_executor_add_subscription (sub_throttle) failed: %d", rc);
            rcl_node_fini(&node);
            rclc_support_fini(&support);
            k_msleep(1000);
            continue;
        }
        rc = rclc_executor_add_subscription(&executor, &sub_gear, &submsg_gear, gear_cb, ON_NEW_DATA);
        if (rc != RCL_RET_OK) {
            LOG_ERR("rclc_executor_add_subscription (sub_gear) failed: %d", rc);
            rcl_node_fini(&node);
            rclc_support_fini(&support);
            k_msleep(1000);
            continue;
        }
        rc = rclc_executor_add_subscription(&executor, &sub_diff, &submsg_diff, diff_cb, ON_NEW_DATA);
        if (rc != RCL_RET_OK) {
            LOG_ERR("rclc_executor_add_subscription (sub_diff) failed: %d", rc);
            rcl_node_fini(&node);
            rclc_support_fini(&support);
            k_msleep(1000);
            continue;
        }

        // If all succeeded, break out of retry loop
        break;
    }

    LOG_INF("ROS iface thread started");
    ros_initialized = true;
    const uint32_t pub_period_ms = 50; // Publish every 50ms
    uint64_t last_pub_time = k_uptime_get();

    while (1) {
        rclc_executor_spin_some(&executor, 10);
        k_msleep(1);

        uint64_t now = k_uptime_get();
        if ((now - last_pub_time) >= pub_period_ms) {
            last_pub_time = now;

            // Only publish remote state if remote is connected
            if (remote_connected) {
                int32_t steer_us = rc_get_pulse_us(RC_STEER);
                int32_t throttle_us = rc_get_pulse_us(RC_THROTTLE);
                uint32_t gear_us = rc_get_pulse_us(RC_HIGH_GEAR);
                uint32_t override_us = rc_get_pulse_us(RC_OVERRIDE);

                msg_steer.data = pulse_to_int8(steer_us);
                msg_throttle.data = pulse_to_int8(throttle_us);
                msg_gear.data = pulse_to_bool(gear_us);
                msg_override.data = pulse_to_bool(override_us);

                rcl_ret_t rc;
                rc = rcl_publish(&pub_steer, &msg_steer, NULL);
                if (rc != RCL_RET_OK)
                    LOG_ERR("pub_steer failed: %d", rc);
                rc = rcl_publish(&pub_throttle, &msg_throttle, NULL);
                if (rc != RCL_RET_OK)
                    LOG_ERR("pub_throttle failed: %d", rc);
                rc = rcl_publish(&pub_gear, &msg_gear, NULL);
                if (rc != RCL_RET_OK)
                    LOG_ERR("pub_gear failed: %d", rc);
                rc = rcl_publish(&pub_override, &msg_override, NULL);
                if (rc != RCL_RET_OK)
                    LOG_ERR("pub_override failed: %d", rc);
            }

            // Always publish remote_connected status
            rcl_ret_t rc = rcl_publish(&pub_connected, &remote_connected, NULL);
            if (rc != RCL_RET_OK)
                LOG_ERR("pub_connected failed: %d", rc);
        }
    }
}

void ros_iface_init(void) {
    k_thread_create(&ros_thread, ros_stack, K_THREAD_STACK_SIZEOF(ros_stack),
                    ros_iface_thread, NULL, NULL, NULL,
                    5, 0, K_NO_WAIT);
}

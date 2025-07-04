#include "ros_iface.h"
#include "servo.h"
#include <microros_transports.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rcutils/allocator.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/u_int8.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ros_iface, LOG_LEVEL_INF);

static inline void ros_transport_init(void) {
    rmw_uros_set_custom_transport(
        true,                    /* MICRO_ROS_FRAMING_REQUIRED           */
        (void *)&default_params, /* provided by the serial transport     */
        zephyr_transport_open,
        zephyr_transport_close,
        zephyr_transport_write,
        zephyr_transport_read);
}

// ROS Topics
#define ROS_TOPIC_STEERING "/lli/ctrl/steering" // uint8
#define ROS_TOPIC_GEAR "/lli/ctrl/gear"         // bool
#define ROS_TOPIC_THROTTLE "/lli/ctrl/throttle" // uint8
#define ROS_TOPIC_DIFF "/lli/ctrl/diff"         // bool

#define ROS_TOPIC_RC_STEERING "/lli/remote/steering"   // uint8
#define ROS_TOPIC_RC_GEAR "/lli/remote/high_gear"      // bool
#define ROS_TOPIC_RC_THROTTLE "/lli/remote/throttle"   // uint8
#define ROS_TOPIC_RC_OVERRIDE "/lli/remote/override"   // bool
#define ROS_TOPIC_RC_CONNECTED "/lli/remote/connected" // bool

// Servo channel structure for ROS control
struct servo_ros_channel {
    rcl_subscription_t sub;
    std_msgs__msg__UInt8 msg;
    const char *topic;
    int servo_id;
    uint8_t prev_value;
};

// RC publisher structure
struct rc_publisher {
    rcl_publisher_t pub;
    std_msgs__msg__UInt8 msg;
    const char *topic;
    int rc_channel;
};

// ROS interface context
static struct {
    rcl_context_t context;
    rcl_node_t node;
    rclc_executor_t executor;
    rclc_support_t support;

    struct servo_ros_channel servo_channels[4];
    struct rc_publisher rc_publishers[4];

    bool initialized;
} ros_ctx = {0};

bool ros_cmd_valid = false;

// Convert uint8 (0-255) to microseconds (1000-2000)
static uint32_t uint8_to_us(uint8_t value) {
    return 1000 + ((uint32_t)value * 1000) / 255;
}

// Convert microseconds (1000-2000) to uint8 (0-255)
static uint8_t us_to_uint8(uint32_t us) {
    if (us < 1000)
        us = 1000;
    if (us > 2000)
        us = 2000;
    return (uint8_t)(((us - 1000) * 255) / 1000);
}

static bool us_to_bool(uint32_t us) {
    return us > 1500;
}

// Servo command callback
static void servo_callback(const void *msg_in, void *context) {
    struct servo_ros_channel *channel = (struct servo_ros_channel *)context;

    if (strcmp(channel->topic, ROS_TOPIC_DIFF) == 0) {
        const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msg_in;
        set_diff_state(msg->data);
        ros_cmd_valid = true;
        return;
    }

    const std_msgs__msg__UInt8 *msg = (const std_msgs__msg__UInt8 *)msg_in;

    LOG_DBG("ROS servo command received for %s: %d", channel->topic, msg->data);

    // Skip if value hasn't changed
    if (msg->data == channel->prev_value) {
        return;
    }

    channel->prev_value = msg->data;

    // Convert uint8 to microseconds and send to servo
    uint32_t us = uint8_to_us(msg->data);
    servo_request(channel->servo_id, us);

    ros_cmd_valid = true;
}

bool rclc_support_init_ok(void) {
    if (ros_ctx.initialized) {
        return true;
    }

    // Get default allocator
    rcl_allocator_t allocator = rcl_get_default_allocator();

    // Initialize micro-ROS support with correct parameters
    rcl_ret_t ret = rclc_support_init(&ros_ctx.support, 0, NULL, &allocator);
    if (ret != RCL_RET_OK) {
        LOG_ERR("Failed to initialize rclc support: %d", ret);
        return false;
    }

    // Create node
    ret = rclc_node_init_default(&ros_ctx.node, "svea_lli_node", "", &ros_ctx.support);
    if (ret != RCL_RET_OK) {
        LOG_ERR("Failed to create node: %d", ret);
        return false;
    }

    // Create executor
    ret = rclc_executor_init(&ros_ctx.executor, &ros_ctx.support.context, 8, &allocator);
    if (ret != RCL_RET_OK) {
        LOG_ERR("Failed to create executor: %d", ret);
        return false;
    }

    LOG_INF("Micro-ROS support initialized successfully");
    ros_ctx.initialized = true;
    return true;
}

static void init_servo_subscribers(void) {
    LOG_INF("Initializing servo subscribers");

    // Initialize servo channel configurations
    ros_ctx.servo_channels[0] = (struct servo_ros_channel){
        .topic = ROS_TOPIC_STEERING,
        .servo_id = 0,
        .prev_value = 127 // Center value
    };

    ros_ctx.servo_channels[1] = (struct servo_ros_channel){
        .topic = ROS_TOPIC_GEAR,
        .servo_id = 1,
        .prev_value = 127};

    ros_ctx.servo_channels[2] = (struct servo_ros_channel){
        .topic = ROS_TOPIC_THROTTLE,
        .servo_id = 2,
        .prev_value = 127};

    ros_ctx.servo_channels[3] = (struct servo_ros_channel){
        .topic = ROS_TOPIC_DIFF,
        .servo_id = 3,
        .prev_value = 127};

    // Initialize subscriptions
    for (int i = 0; i < 4; i++) {
        struct servo_ros_channel *channel = &ros_ctx.servo_channels[i];

        rcl_ret_t ret = rclc_subscription_init_default(
            &channel->sub,
            &ros_ctx.node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
            channel->topic);

        if (ret != RCL_RET_OK) {
            LOG_ERR("Failed to init subscription for %s: %d", channel->topic, ret);
            continue;
        }

        ret = rclc_executor_add_subscription_with_context(
            &ros_ctx.executor,
            &channel->sub,
            &channel->msg,
            servo_callback,
            channel,
            ON_NEW_DATA);

        if (ret != RCL_RET_OK) {
            LOG_ERR("Failed to add subscription to executor for %s: %d", channel->topic, ret);
            continue;
        }

        LOG_INF("Servo subscriber initialized for %s", channel->topic);
    }
}

static void init_rc_publishers(void) {
    LOG_INF("Initializing RC publishers");

    // Initialize RC publisher configurations
    ros_ctx.rc_publishers[0] = (struct rc_publisher){
        .topic = ROS_TOPIC_RC_STEERING,
        .rc_channel = RC_STEER};

    ros_ctx.rc_publishers[1] = (struct rc_publisher){
        .topic = ROS_TOPIC_RC_GEAR,
        .rc_channel = RC_GEAR};

    ros_ctx.rc_publishers[2] = (struct rc_publisher){
        .topic = ROS_TOPIC_RC_THROTTLE,
        .rc_channel = RC_THROTTLE};

    ros_ctx.rc_publishers[3] = (struct rc_publisher){
        .topic = ROS_TOPIC_RC_OVERRIDE,
        .rc_channel = RC_OVERRIDE};

    // Initialize publishers
    for (int i = 0; i < 4; i++) {
        struct rc_publisher *pub = &ros_ctx.rc_publishers[i];

        rcl_ret_t ret = rclc_publisher_init_best_effort(
            &pub->pub,
            &ros_ctx.node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
            pub->topic);

        if (ret != RCL_RET_OK) {
            LOG_ERR("Failed to init publisher for %s: %d", pub->topic, ret);
            continue;
        }

        LOG_INF("RC publisher initialized for %s", pub->topic);
    }
}

void ros_publish_rc(const RemoteState *rc_frame) {
    if (!ros_ctx.initialized) {
        return;
    }

    // Publish each RC channel as uint8 value
    for (int i = 0; i < 4; i++) {
        struct rc_publisher *pub = &ros_ctx.rc_publishers[i];

        if (!pub->pub.impl) {
            continue; // Publisher not initialized
        }

        uint32_t us_value = rc_frame->fields[i];
        uint8_t uint8_value = us_to_uint8(us_value);

        pub->msg.data = uint8_value;

        rcl_ret_t ret = rcl_publish(&pub->pub, &pub->msg, NULL);
        if (ret != RCL_RET_OK) {
            LOG_WRN("Failed to publish RC data for %s: %d", pub->topic, ret);
        }
    }
}

void ros_iface_init(void) {
    LOG_INF("Initializing ROS interface");
    ros_transport_init();
    // Wait for micro-ROS agent
    while (!rclc_support_init_ok()) {
        LOG_INF("Waiting for micro-ROS agent...");
        k_sleep(K_SECONDS(1));
    }

    LOG_INF("Micro-ROS agent connected");

    // Initialize subscribers and publishers
    init_servo_subscribers();
    init_rc_publishers();

    LOG_INF("ROS interface initialization complete");
}

void ros_iface_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    LOG_INF("ROS interface thread started");

    while (!ros_ctx.initialized) {
        LOG_WRN("ROS interface not initialized, waiting...");
        k_sleep(K_SECONDS(1));
    }

    int consecutive_failures = 0;

    while (1) {
        // Spin the executor to handle callbacks
        rcl_ret_t ret = rclc_executor_spin_some(&ros_ctx.executor, RCL_MS_TO_NS(10));

        if (ret != RCL_RET_OK && ret != RCL_RET_TIMEOUT) {
            LOG_WRN("Executor spin failed: %d", ret);
            consecutive_failures++;

            if (consecutive_failures > 10) {
                LOG_ERR("Too many consecutive ROS failures, attempting recovery");
                // Try to reinitialize
                ros_ctx.initialized = false;
                // Clean up existing resources if needed
                // ...
                if (rclc_support_init_ok()) {
                    // Reinitialize subscribers and publishers
                    init_servo_subscribers();
                    init_rc_publishers();
                    LOG_INF("ROS interface recovery complete");
                }
                consecutive_failures = 0;
                k_sleep(K_MSEC(100)); // Longer sleep after recovery attempt
            }
        } else {
            consecutive_failures = 0;
        }

        // Adaptive sleep - sleep longer if we're having issues
        if (consecutive_failures > 0) {
            k_sleep(K_MSEC(50)); // Sleep longer when having issues
        } else {
            k_sleep(K_MSEC(20)); // Normal operation sleep
        }
    }
}

// Start ROS interface thread
K_THREAD_DEFINE(ros_tid, 4096, ros_iface_thread, NULL, NULL, NULL, 2, 0, 0);
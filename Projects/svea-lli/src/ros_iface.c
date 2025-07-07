#include "ros_iface.h"

#include <microros_transports.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rcutils/allocator.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/u_int8.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ros_iface, LOG_LEVEL_INF);

/* ----- Thread definitions ------------------------------------------- */
#define ROS_STACK_SIZE 4096
#define ROS_THREAD_PRIORITY 5

static K_THREAD_STACK_DEFINE(ros_stack, ROS_STACK_SIZE);
static struct k_thread ros_thread;

/* ----- Type definitions --------------------------------------------- */
typedef struct {
    rcl_subscription_t sub;
    void *msg;
    rclc_subscription_callback_t cb;
    const rosidl_message_type_support_t *type_support;
    const char *topic_name;
} sub_t;

typedef struct {
    rcl_publisher_t pub;
    void *msg;
    const rosidl_message_type_support_t *type_support;
    const char *topic_name;
} pub_t;

/* ----- ROS Context -------------------------------------------------- */
static struct {
    rcl_allocator_t allocator;
    rclc_support_t support;
    rcl_node_t node;
    rclc_executor_t executor;
    bool initialized;

    // Subscriptions
    sub_t steering_sub;
    std_msgs__msg__UInt8 steering_msg;

    sub_t gear_sub;
    std_msgs__msg__Bool gear_msg;

    sub_t throttle_sub;
    std_msgs__msg__UInt8 throttle_msg;

    sub_t diff_sub;
    std_msgs__msg__Bool diff_msg;

    // Publishers
    pub_t rc_steering_pub;
    std_msgs__msg__UInt8 rc_steering_msg;

    pub_t rc_gear_pub;
    std_msgs__msg__Bool rc_gear_msg;

    pub_t rc_throttle_pub;
    std_msgs__msg__UInt8 rc_throttle_msg;

    pub_t rc_override_pub;
    std_msgs__msg__Bool rc_override_msg;

    pub_t rc_connected_pub;
    std_msgs__msg__Bool rc_connected_msg;

} g_ros;

/* ----- Global state ------------------------------------------------- */
static ros_command_t g_ros_cmd;
static K_MUTEX_DEFINE(g_ros_cmd_mutex);

/* ----- error helper ------------------------------------------------- */
#define RCCHECK(fn)                                                                     \
    {                                                                                   \
        rcl_ret_t temp_rc = fn;                                                         \
        if ((temp_rc != RCL_RET_OK)) {                                                  \
            LOG_ERR("Failed status on line %d: %d. Aborting.", __LINE__, (int)temp_rc); \
            return;                                                                     \
        }                                                                               \
    }

#define RCSOFTCHECK(fn)                                                                   \
    {                                                                                     \
        rcl_ret_t temp_rc = fn;                                                           \
        if ((temp_rc != RCL_RET_OK)) {                                                    \
            LOG_ERR("Failed status on line %d: %d. Continuing.", __LINE__, (int)temp_rc); \
        }                                                                                 \
    }

/* ----- Conversion helpers ------------------------------------------- */
static inline uint32_t uint8_to_us(uint8_t v) {
    return 1000 + ((uint32_t)v * 1000) / 255;
}

static inline uint8_t us_to_uint8(uint32_t us) {
    if (us < 1000)
        us = 1000;
    if (us > 2000)
        us = 2000;
    return (uint8_t)(((us - 1000) * 255) / 1000);
}

static inline bool us_to_bool(uint32_t us) {
    return us > 1500;
}

/* ----- Topic callbacks ---------------------------------------------- */
static void steering_callback(const void *msg) {
    const std_msgs__msg__UInt8 *m = (const std_msgs__msg__UInt8 *)msg;
    k_mutex_lock(&g_ros_cmd_mutex, K_FOREVER);
    g_ros_cmd.steering_us = uint8_to_us(m->data);
    g_ros_cmd.timestamp = k_uptime_get();
    k_mutex_unlock(&g_ros_cmd_mutex);
    LOG_INF("ROS steering received: %d", m->data);
}

static void gear_callback(const void *msg) {
    const std_msgs__msg__Bool *m = (const std_msgs__msg__Bool *)msg;
    k_mutex_lock(&g_ros_cmd_mutex, K_FOREVER);
    g_ros_cmd.high_gear = m->data;
    g_ros_cmd.timestamp = k_uptime_get();
    k_mutex_unlock(&g_ros_cmd_mutex);
    LOG_INF("ROS gear received: %s", m->data ? "high" : "low");
}

static void throttle_callback(const void *msg) {
    const std_msgs__msg__UInt8 *m = (const std_msgs__msg__UInt8 *)msg;
    k_mutex_lock(&g_ros_cmd_mutex, K_FOREVER);
    g_ros_cmd.throttle_us = uint8_to_us(m->data);
    g_ros_cmd.timestamp = k_uptime_get();
    k_mutex_unlock(&g_ros_cmd_mutex);
    LOG_INF("ROS throttle received: %d", m->data);
}

static void diff_callback(const void *msg) {
    const std_msgs__msg__Bool *m = (const std_msgs__msg__Bool *)msg;
    k_mutex_lock(&g_ros_cmd_mutex, K_FOREVER);
    g_ros_cmd.diff_locked = m->data;
    g_ros_cmd.timestamp = k_uptime_get();
    k_mutex_unlock(&g_ros_cmd_mutex);
    LOG_INF("ROS diff lock received: %s", m->data ? "locked" : "unlocked");
}

/* ----- Private functions -------------------------------------------- */
static void init_subscriptions(void) {
    g_ros.steering_sub = (sub_t){
        .msg = &g_ros.steering_msg,
        .cb = steering_callback,
        .type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
        .topic_name = "/lli/ctrl/steering"};

    g_ros.throttle_sub = (sub_t){
        .msg = &g_ros.throttle_msg,
        .cb = throttle_callback,
        .type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
        .topic_name = "/lli/ctrl/throttle"};

    g_ros.gear_sub = (sub_t){
        .msg = &g_ros.gear_msg,
        .cb = gear_callback,
        .type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        .topic_name = "/lli/ctrl/high_gear"};

    g_ros.diff_sub = (sub_t){
        .msg = &g_ros.diff_msg,
        .cb = diff_callback,
        .type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        .topic_name = "/lli/ctrl/diff"};

    sub_t *subs[] = {&g_ros.steering_sub, &g_ros.throttle_sub, &g_ros.gear_sub, &g_ros.diff_sub};
    const int num_subs = sizeof(subs) / sizeof(subs[0]);

    RCCHECK(rclc_executor_init(&g_ros.executor, &g_ros.support.context, num_subs, &g_ros.allocator));

    for (int i = 0; i < num_subs; i++) {
        RCCHECK(rclc_subscription_init_default(&subs[i]->sub, &g_ros.node, subs[i]->type_support, subs[i]->topic_name));
        RCCHECK(rclc_executor_add_subscription(&g_ros.executor, &subs[i]->sub, subs[i]->msg, subs[i]->cb, ON_NEW_DATA));
    }
}

static void init_publishers(void) {
    g_ros.rc_steering_pub = (pub_t){
        .msg = &g_ros.rc_steering_msg,
        .type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
        .topic_name = "/lli/remote/steering"};

    g_ros.rc_throttle_pub = (pub_t){
        .msg = &g_ros.rc_throttle_msg,
        .type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
        .topic_name = "/lli/remote/throttle"};

    g_ros.rc_gear_pub = (pub_t){
        .msg = &g_ros.rc_gear_msg,
        .type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        .topic_name = "/lli/remote/high_gear"};

    g_ros.rc_override_pub = (pub_t){
        .msg = &g_ros.rc_override_msg,
        .type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        .topic_name = "/lli/remote/override"};

    g_ros.rc_connected_pub = (pub_t){
        .msg = &g_ros.rc_connected_msg,
        .type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        .topic_name = "/lli/remote/connected"};

    pub_t *pubs[] = {&g_ros.rc_steering_pub, &g_ros.rc_throttle_pub, &g_ros.rc_gear_pub, &g_ros.rc_override_pub, &g_ros.rc_connected_pub};
    const int num_pubs = sizeof(pubs) / sizeof(pubs[0]);

    for (int i = 0; i < num_pubs; i++) {
        RCCHECK(rclc_publisher_init_best_effort(&pubs[i]->pub, &g_ros.node, pubs[i]->type_support, pubs[i]->topic_name));
    }
}

static void ros_spin(void *a, void *b, void *c) {
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    while (!g_ros.initialized) {
        k_msleep(50);
    }

    while (1) {
        RCSOFTCHECK(rclc_executor_spin_some(&g_ros.executor, RCL_MS_TO_NS(10)));
        k_msleep(10);
    }
}

/* ----- Public functions --------------------------------------------- */
void ros_get_command(ros_command_t *cmd) {
    k_mutex_lock(&g_ros_cmd_mutex, K_FOREVER);
    *cmd = g_ros_cmd;
    k_mutex_unlock(&g_ros_cmd_mutex);
}

void ros_publish_rc(const RemoteState *rc, bool is_connected) {
    if (!g_ros.initialized) {
        return;
    }

    g_ros.rc_steering_msg.data = us_to_uint8(rc->steer);
    g_ros.rc_gear_msg.data = us_to_bool(rc->gear_us);
    g_ros.rc_throttle_msg.data = us_to_uint8(rc->throttle);
    g_ros.rc_override_msg.data = us_to_bool(rc->override_us);
    g_ros.rc_connected_msg.data = is_connected;

    RCSOFTCHECK(rcl_publish(&g_ros.rc_steering_pub.pub, g_ros.rc_steering_pub.msg, NULL));
    RCSOFTCHECK(rcl_publish(&g_ros.rc_gear_pub.pub, g_ros.rc_gear_pub.msg, NULL));
    RCSOFTCHECK(rcl_publish(&g_ros.rc_throttle_pub.pub, g_ros.rc_throttle_pub.msg, NULL));
    RCSOFTCHECK(rcl_publish(&g_ros.rc_override_pub.pub, g_ros.rc_override_pub.msg, NULL));
    RCSOFTCHECK(rcl_publish(&g_ros.rc_connected_pub.pub, g_ros.rc_connected_pub.msg, NULL));
}

void ros_iface_init(void) {
    // Set up microROS transport
    rmw_uros_set_custom_transport(
        MICRO_ROS_FRAMING_REQUIRED,
        (void *)&default_params,
        zephyr_transport_open,
        zephyr_transport_close,
        zephyr_transport_write,
        zephyr_transport_read);

    // Initialize allocator and support
    g_ros.allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&g_ros.support, 0, NULL, &g_ros.allocator));

    // Create node
    RCCHECK(rclc_node_init_default(&g_ros.node, "svea_lli_node", "", &g_ros.support));

    // Initialize publishers and subscriptions
    init_publishers();
    init_subscriptions();

    // Initialize command state
    k_mutex_lock(&g_ros_cmd_mutex, K_FOREVER);
    g_ros_cmd.steering_us = 1500;
    g_ros_cmd.throttle_us = 1500;
    g_ros_cmd.high_gear = false;
    g_ros_cmd.diff_locked = true;
    g_ros_cmd.timestamp = 0;
    k_mutex_unlock(&g_ros_cmd_mutex);

    // Start executor thread
    k_thread_create(&ros_thread, ros_stack, K_THREAD_STACK_SIZEOF(ros_stack),
                    ros_spin, NULL, NULL, NULL,
                    K_PRIO_COOP(ROS_THREAD_PRIORITY), 0, K_NO_WAIT);

    g_ros.initialized = true;
    LOG_INF("ROS interface initialized successfully");
}
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

#include <geometry_msgs/msg/pose_stamped.h>
#include <microros_transports.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <std_msgs/msg/string.h>

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

#define PUBLISH_HZ 1000U

static rcl_publisher_t publisher;
static std_msgs__msg__String msg;

static rcl_publisher_t pub_imu, pub_gps, pub_pose, pub_batt;
static sensor_msgs__msg__Imu imu_msg;
static sensor_msgs__msg__NavSatFix gps_msg;
static geometry_msgs__msg__PoseStamped pose_msg;
static sensor_msgs__msg__BatteryState batt_msg;

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

    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));

    // Simulate IMU
    imu_msg.header.stamp.sec = cnt;
    imu_msg.header.frame_id.data = "imu";
    imu_msg.header.frame_id.size = 3;
    imu_msg.linear_acceleration.x = (float)(rand() % 100) / 10.0f;
    imu_msg.linear_acceleration.y = (float)(rand() % 100) / 10.0f;
    imu_msg.linear_acceleration.z = (float)(rand() % 100) / 10.0f;
    imu_msg.angular_velocity.x = (float)(rand() % 100) / 10.0f;
    imu_msg.angular_velocity.y = (float)(rand() % 100) / 10.0f;
    imu_msg.angular_velocity.z = (float)(rand() % 100) / 10.0f;

    // Simulate GPS
    gps_msg.header.stamp.sec = cnt;
    gps_msg.header.frame_id.data = "gps";
    gps_msg.header.frame_id.size = 3;
    gps_msg.latitude = 59.0 + ((float)(rand() % 1000) / 10000.0f);
    gps_msg.longitude = 18.0 + ((float)(rand() % 1000) / 10000.0f);
    gps_msg.altitude = 10.0 + ((float)(rand() % 100) / 10.0f);

    // Simulate Pose
    pose_msg.header.stamp.sec = cnt;
    pose_msg.header.frame_id.data = "map";
    pose_msg.header.frame_id.size = 3;
    pose_msg.pose.position.x = (float)(rand() % 100);
    pose_msg.pose.position.y = (float)(rand() % 100);
    pose_msg.pose.position.z = (float)(rand() % 100);
    pose_msg.pose.orientation.w = 1.0;

    // Simulate Battery
    batt_msg.header.stamp.sec = cnt;
    batt_msg.voltage = 12.0f + ((float)(rand() % 100) / 100.0f);
    batt_msg.percentage = ((float)(rand() % 100)) / 100.0f;

    // Publish all
    printf("Publishing all sensor messages, cnt=%lu\n", (unsigned long)cnt);

    RCSOFTCHECK(rcl_publish(&pub_imu, &imu_msg, NULL));
    RCSOFTCHECK(rcl_publish(&pub_gps, &gps_msg, NULL));
    RCSOFTCHECK(rcl_publish(&pub_pose, &pose_msg, NULL));
    RCSOFTCHECK(rcl_publish(&pub_batt, &batt_msg, NULL));

    rcl_reset_error();
}

int main(void) {
    printf("micro‑ROS Zephyr string publisher\n");
    printf("micro-ROS resource config:\n");
#ifdef CONFIG_MICROROS_PUBLISHERS
    printf("  CONFIG_MICROROS_PUBLISHERS: %s\n", CONFIG_MICROROS_PUBLISHERS);
#endif
#ifdef CONFIG_MICROROS_SUBSCRIBERS
    printf("  CONFIG_MICROROS_SUBSCRIBERS: %s\n", CONFIG_MICROROS_SUBSCRIBERS);
#endif
#ifdef CONFIG_MICROROS_CLIENTS
    printf("  CONFIG_MICROROS_CLIENTS: %s\n", CONFIG_MICROROS_CLIENTS);
#endif
#ifdef CONFIG_MICROROS_SERVERS
    printf("  CONFIG_MICROROS_SERVERS: %s\n", CONFIG_MICROROS_SERVERS);
#endif
    printf("Checking type support for expected messages:\n");

    const rosidl_message_type_support_t *ts_string = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);
    const rosidl_message_type_support_t *ts_imu = ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu);
    const rosidl_message_type_support_t *ts_gps = ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix);
    const rosidl_message_type_support_t *ts_pose = ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped);
    const rosidl_message_type_support_t *ts_batt = ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState);

    printf("  std_msgs/msg/String:      %s\n", ts_string ? "OK" : "MISSING");
    printf("  sensor_msgs/msg/Imu:      %s\n", ts_imu ? "OK" : "MISSING");
    printf("  sensor_msgs/msg/NavSatFix:%s\n", ts_gps ? "OK" : "MISSING");
    printf("  geometry_msgs/msg/PoseStamped: %s\n", ts_pose ? "OK" : "MISSING");
    printf("  sensor_msgs/msg/BatteryState:  %s\n", ts_batt ? "OK" : "MISSING");

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
        rcl_node_t node;
        rcl_timer_t timer;
        rclc_executor_t executor;

        if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
            k_sleep(K_MSEC(1000));
            continue;
        }

        if (rclc_node_init_default(&node, "zephyr_string_publisher", "", &support) != RCL_RET_OK) {
            rclc_support_fini(&support);
            k_sleep(K_MSEC(1000));
            continue;
        }

        printf("Initializing publishers...\n");

        bool ok_string = false, ok_imu = false, ok_gps = false, ok_pose = false, ok_batt = false;
        rcl_ret_t rc_string = 0, rc_imu = 0, rc_gps = 0, rc_pose = 0, rc_batt = 0;

        // IMU publisher (Best Effort)
        {
            rcl_publisher_options_t pub_ops = rcl_publisher_get_default_options();
            pub_ops.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
            rc_imu = rcl_publisher_init(
                &pub_imu, &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
                "imu/data", &pub_ops);
            ok_imu = (rc_imu == RCL_RET_OK);
        }

        // String publisher (Best Effort)
        {
            rcl_publisher_options_t pub_ops = rcl_publisher_get_default_options();
            pub_ops.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
            rc_string = rcl_publisher_init(
                &publisher, &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                "zephyr_string_publisher", &pub_ops);
            ok_string = (rc_string == RCL_RET_OK);
        }

        // GPS publisher (Best Effort)
        {
            rcl_publisher_options_t pub_ops = rcl_publisher_get_default_options();
            pub_ops.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
            rc_gps = rcl_publisher_init(
                &pub_gps, &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix),
                "gps/fix", &pub_ops);
            ok_gps = (rc_gps == RCL_RET_OK);
        }

        // Pose publisher (Best Effort)
        {
            rcl_publisher_options_t pub_ops = rcl_publisher_get_default_options();
            pub_ops.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
            rc_pose = rcl_publisher_init(
                &pub_pose, &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped),
                "pose", &pub_ops);
            ok_pose = (rc_pose == RCL_RET_OK);
        }

        // Battery publisher (Best Effort)
        {
            rcl_publisher_options_t pub_ops = rcl_publisher_get_default_options();
            pub_ops.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
            rc_batt = rcl_publisher_init(
                &pub_batt, &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
                "battery", &pub_ops);
            ok_batt = (rc_batt == RCL_RET_OK);
        }

        printf("Publisher initialization summary:\n");
        printf("  std_msgs/String:      %s (rc=%d)\n", ok_string ? "OK" : "FAILED", rc_string);
        printf("  sensor_msgs/Imu:      %s (rc=%d)\n", ok_imu ? "OK" : "FAILED", rc_imu);
        printf("  sensor_msgs/NavSatFix:%s (rc=%d)\n", ok_gps ? "OK" : "FAILED", rc_gps);
        printf("  geometry_msgs/Pose:   %s (rc=%d)\n", ok_pose ? "OK" : "FAILED", rc_pose);
        printf("  sensor_msgs/Battery:  %s (rc=%d)\n", ok_batt ? "OK" : "FAILED", rc_batt);

        if (!ok_string) {
            printf("Critical publisher (std_msgs/String) failed, aborting setup.\n");
            goto cleanup;
        }
        if (!ok_imu || !ok_gps || !ok_pose || !ok_batt) {
            printf("One or more sensor publishers failed, continuing with available publishers.\n");
        }

        /* Timer setup */
        const uint32_t period_ms = 1000U / PUBLISH_HZ;
        RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(period_ms), timer_callback));

        RCCHECK(rclc_executor_init(&executor, &support.context, 10, &allocator));
        RCCHECK(rclc_executor_add_timer(&executor, &timer));

        printf("micro‑ROS ready: publishing at %u Hz\n", PUBLISH_HZ);

        while (rclc_executor_spin_some(&executor, 1) == RCL_RET_OK) {
            /* run forever */
        }

    cleanup:
        printf("micro‑ROS error – restarting setup…\n");
        rclc_executor_fini(&executor);
        rcl_timer_fini(&timer);
        rcl_publisher_fini(&publisher, &node);
        rcl_publisher_fini(&pub_imu, &node);
        rcl_publisher_fini(&pub_gps, &node);
        rcl_publisher_fini(&pub_pose, &node);
        rcl_publisher_fini(&pub_batt, &node);
        rcl_node_fini(&node);
        rclc_support_fini(&support);
        k_sleep(K_MSEC(1000));
    }
    return 0;
}

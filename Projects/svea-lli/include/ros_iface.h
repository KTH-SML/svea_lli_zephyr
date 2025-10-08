/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * SVEA Note.
 *
 * Part of the SVEA Low‑Level Interface (Zephyr) application.
 * Author: Nils Kiefer
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#ifndef ROS_IFACE_H
#define ROS_IFACE_H

#include "rc_input.h"
#include <rcl/rcl.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/u_int8.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t steering_us;
    uint32_t throttle_us;
    bool high_gear;
    bool diff_locked;
    int64_t timestamp;
} ros_command_t;

void ros_iface_init(void);
void ros_get_command(ros_command_t *cmd);
extern bool ros_initialized;

// Sensors
extern rcl_publisher_t pub_remote_steer;
extern rcl_publisher_t pub_remote_throttle;
extern rcl_publisher_t pub_remote_gear;
extern rcl_publisher_t pub_remote_override;
extern rcl_publisher_t pub_remote_connected;
extern rcl_publisher_t imu_pub;
extern rcl_publisher_t encoders_pub;
extern rcl_publisher_t ina3221_pub;
extern rcl_publisher_t battery_pub;

void ros_iface_handle_remote_publish_error(void);

uint64_t ros_iface_epoch_millis(void);
uint64_t ros_iface_epoch_nanos(void);

#ifdef __cplusplus
}
#endif

#endif // ROS_IFACE_H

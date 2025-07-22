#ifndef ROS_IFACE_H
#define ROS_IFACE_H

#include "rc_input.h"
#include <rcl/rcl.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/u_int8.h>
#include <zephyr/kernel.h>

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
extern rcl_publisher_t imu_pub;
extern rcl_publisher_t encoders_pub;

int32_t ros_iface_epoch_millis(void);
int32_t ros_iface_epoch_nanos(void);

#endif // ROS_IFACE_H

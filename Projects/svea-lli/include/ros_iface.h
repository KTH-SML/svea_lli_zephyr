#ifndef ROS_IFACE_H
#define ROS_IFACE_H

#include "remote.h"
#include <zephyr/kernel.h>
#include <std_msgs/msg/u_int8.h>
#include <std_msgs/msg/bool.h>

typedef struct {
    uint32_t steering_us;
    uint32_t throttle_us;
    bool high_gear;
    bool diff_locked;
    int64_t timestamp;
} ros_command_t;

void ros_iface_init(void);
void ros_get_command(ros_command_t *cmd);
void ros_publish_rc(const RemoteState *rc_frame, bool is_connected);

#endif // ROS_IFACE_H

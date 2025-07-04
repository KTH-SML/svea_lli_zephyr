#ifndef ROS_IFACE_H
#define ROS_IFACE_H

#include "remote.h"
#include <zephyr/kernel.h>

// External variables
extern bool ros_cmd_valid;

// Function declarations
void ros_iface_init(void);
void ros_iface_thread(void *p1, void *p2, void *p3);
bool rclc_support_init_ok(void);
void ros_publish_rc(const RemoteState *rc_frame);

#endif // ROS_IFACE_H
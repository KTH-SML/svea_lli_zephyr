#pragma once

#include <rcl/rcl.h>
#include <std_msgs/msg/float32.h>
#include <zephyr/kernel.h>

void pwm_in_publishers_init(rcl_node_t *node);
#pragma once
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <zephyr/kernel.h>

#define SERVO_THREAD_STACK_SIZE 1024
#define SERVO_THREAD_PRIORITY 4

#define SERVO_TOPIC "/lli/servo/steering"
#define SERVO_MSGTYPE std_msgs__msg__Float32

void steering_servo_subscriber_init(rcl_node_t *node, rclc_executor_t *exec);

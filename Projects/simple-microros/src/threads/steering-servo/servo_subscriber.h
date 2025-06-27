#pragma once
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <zephyr/kernel.h>

/* Tune these if you need */
#define SERVO_THREAD_STACK_SIZE 1024
#define SERVO_THREAD_PRIORITY 4
#define SERVO_NODE DT_NODELABEL(steeringservo)

/* ROS topic and message type the servo listens to */
#define SERVO_TOPIC "/lli/servo/steering"
#define SERVO_MSGTYPE std_msgs__msg__Float32

void steering_servo_subscriber_init(rcl_node_t *node, rclc_executor_t *exec);

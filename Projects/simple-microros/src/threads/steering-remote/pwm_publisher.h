#pragma once

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <zephyr/kernel.h>

#define STEERING_PWM_THREAD_STACK_SIZE 1024
#define STEERING_PWM_THREAD_PRIORITY 4
#define STEERING_PWM_TOPIC "/steering/duty"

extern struct k_thread steering_pwm_thread_data;
extern k_thread_stack_t steering_pwm_stack[];

void steering_pwm_publisher_init(rcl_node_t *node);
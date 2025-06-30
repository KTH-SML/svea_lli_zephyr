#pragma once

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <zephyr/kernel.h>

#define STEERING_PWM_THREAD_STACK_SIZE 1024
#define STEERING_PWM_THREAD_PRIORITY 2
#define STEERING_PWM_TOPIC "/steering/duty"

// Duty min max
#define REMOTE_PWM_MIN 100
#define REMOTE_PWM_MAX 200

extern struct k_thread steering_pwm_thread_data;
extern k_thread_stack_t steering_pwm_stack[STEERING_PWM_THREAD_STACK_SIZE];

void steering_pwm_publisher_init(rcl_node_t *node);
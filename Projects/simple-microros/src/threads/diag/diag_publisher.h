#pragma once

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <zephyr/kernel.h>

#define DIAG_THREAD_STACK_SIZE 2048
#define DIAG_THREAD_PRIORITY 3
#define DIAG_PUBLISHER_TOPIC "zephyr_diag"
#define DIAG_PUBLISH_HZ 1U

extern std_msgs__msg__String diag_msg;
extern struct k_thread diag_thread_data;
extern k_thread_stack_t diag_stack[];

void diag_thread(void *arg1, void *arg2, void *arg3);
void diag_publisher_init(rcl_node_t *node);
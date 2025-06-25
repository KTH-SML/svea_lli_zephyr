#pragma once

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>
#include <zephyr/kernel.h>

#define IMU_THREAD_STACK_SIZE 2048
#define IMU_THREAD_PRIORITY 3
#define IMU_PUBLISHER_NAME "imu_publisher"
#define IMU_PUBLISHER_TOPIC "imu/data_raw"
#define IMU_PUBLISH_HZ 1000U

extern sensor_msgs__msg__Imu imu_msg;
extern struct k_thread imu_thread_data;
extern k_thread_stack_t imu_stack[]; // <-- Correct extern declaration

void imu_thread(void *arg1, void *arg2, void *arg3);
void imu_publisher_init(rcl_node_t *node, rclc_executor_t *executor);

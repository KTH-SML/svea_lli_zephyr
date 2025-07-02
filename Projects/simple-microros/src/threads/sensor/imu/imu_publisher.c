#include "imu_publisher.h"
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rmw_microros/time_sync.h>
#include <sensor_msgs/msg/imu.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#ifndef RCCHECK
#define RCCHECK(fn)                                                  \
    do {                                                             \
        rcl_ret_t _rc = (fn);                                        \
        if (_rc != RCL_RET_OK) {                                     \
            printf("rcl error %d at line %d\n", (int)_rc, __LINE__); \
            goto cleanup;                                            \
        }                                                            \
    } while (0)
#endif
#ifndef RCSOFTCHECK
#define RCSOFTCHECK(fn)                                                   \
    do {                                                                  \
        rcl_ret_t _rc = (fn);                                             \
        if (_rc != RCL_RET_OK) {                                          \
            printf("rcl soft error %d at line %d\n", (int)_rc, __LINE__); \
        }                                                                 \
    } while (0)
#endif

K_THREAD_STACK_DEFINE(imu_stack, IMU_THREAD_STACK_SIZE);
struct k_thread imu_thread_data;

sensor_msgs__msg__Imu imu_msg;
rcl_publisher_t pub_imu;
static const struct device *imu_dev = NULL;

void imu_thread(void *arg1, void *arg2, void *arg3) {
    printk("IMU thread: entered\n");
    k_sleep(K_MSEC(1000));
    imu_dev = DEVICE_DT_GET_ONE(st_ism330dlc);
    if (!device_is_ready(imu_dev)) {
        printk("IMU device not ready!\n");
        while (1) {
            k_sleep(K_MSEC(1000));
        }
    }

    struct sensor_value accel[3], gyro[3];

    while (1) {
        int ret = sensor_sample_fetch(imu_dev);
        if (ret == 0) {
            ret |= sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
            ret |= sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_XYZ, gyro);
            if (ret == 0) {
                imu_msg.linear_acceleration.x = sensor_value_to_double(&accel[0]);
                imu_msg.linear_acceleration.y = sensor_value_to_double(&accel[1]);
                imu_msg.linear_acceleration.z = sensor_value_to_double(&accel[2]);
                imu_msg.angular_velocity.x = sensor_value_to_double(&gyro[0]);
                imu_msg.angular_velocity.y = sensor_value_to_double(&gyro[1]);
                imu_msg.angular_velocity.z = sensor_value_to_double(&gyro[2]);

                imu_msg.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
                imu_msg.header.stamp.nanosec = (rmw_uros_epoch_millis() % 1000) * 1000000;
                imu_msg.header.frame_id.data = "imu_link";
                imu_msg.header.frame_id.size = strlen("imu_link");
                imu_msg.header.frame_id.capacity = imu_msg.header.frame_id.size + 1;

                rcl_ret_t rc = rcl_publish(&pub_imu, &imu_msg, NULL);
                if (rc != RCL_RET_OK) {
                    printk("IMU publish error: %d\n", rc);
                }
            }
        }
        k_sleep(K_MSEC(1000 / IMU_PUBLISH_HZ));
    }
}

void imu_publisher_init(rcl_node_t *node, rclc_executor_t *executor) {
    printf("Initializing IMU publisher...\n");
    RCCHECK(rclc_publisher_init_best_effort(
        &pub_imu, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        IMU_PUBLISHER_TOPIC));

    printf("IMU publisher initialized\n");
    // Start the IMU thread only after publisher is ready
    k_thread_create(&imu_thread_data, imu_stack, IMU_THREAD_STACK_SIZE,
                    imu_thread, NULL, NULL, NULL,
                    IMU_THREAD_PRIORITY, 0, K_NO_WAIT);
    printf("IMU thread started\n");

    return;

cleanup:
    printk("IMU publisher init failed\n");
}
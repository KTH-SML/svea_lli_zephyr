#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rmw_microros/time_sync.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>

static rcl_publisher_t pub;

static void thread(void *n, void *b, void *c) {
    const struct device *imu;
    sensor_msgs__msg__Imu msg;

    while (1) {
        imu = DEVICE_DT_GET_ONE(st_ism330dlc);
        if (!device_is_ready(imu)) {
            k_sleep(K_SECONDS(1));
            continue;
        }

        while (1) {
            if (sensor_sample_fetch(imu))
                break;
            sensor_channel_get(imu, SENSOR_CHAN_ACCEL_XYZ, &msg.linear_acceleration);
            sensor_channel_get(imu, SENSOR_CHAN_GYRO_XYZ, &msg.angular_velocity);
            uint64_t now = rmw_uros_epoch_nanos();
            msg.header.stamp.sec = now / 1000000000ULL;
            msg.header.stamp.nanosec = now % 1000000000ULL;
            rcl_publish(&pub, &msg, NULL);
            k_sleep(K_MSEC(10));
        }
    }
}

int sensor_imu_init(rcl_node_t *node) {
    rclc_publisher_init_best_effort(
        &pub, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/lli/sensor/imu/data_raw");

    static uint8_t st[2048];
    k_thread_create(&(struct k_thread){}, st, sizeof st,
                    thread, NULL, NULL, NULL, 5, 0, K_NO_WAIT);
    return 0;
}

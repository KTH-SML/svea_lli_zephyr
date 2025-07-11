#include "sensors.h"
#include "ros_iface.h"
#include <sensor_msgs/msg/imu.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sensors, LOG_LEVEL_INF);

static K_THREAD_STACK_DEFINE(sensors_stack, 2048);
static struct k_thread sensors_thread_data;

static const struct device *imu_dev;
static sensor_msgs__msg__Imu imu_msg;

void sensors_init(void) {
    LOG_INF("Initializing sensors");
    k_thread_create(&sensors_thread_data, sensors_stack, K_THREAD_STACK_SIZEOF(sensors_stack),
                    sensors_thread, NULL, NULL, NULL,
                    6, 0, K_NO_WAIT);
}

void sensors_thread(void *p1, void *p2, void *p3) {
    LOG_INF("Initializing sensors");

    imu_dev = DEVICE_DT_GET(DT_NODELABEL(ism330dlc));

    while (!device_is_ready(imu_dev)) {
        LOG_ERR("IMU device not ready, retrying...");
        k_sleep(K_MSEC(200));
    }

    LOG_INF("IMU device ready");
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    LOG_INF("Sensors thread started");

    while (1) {
        if (sensor_sample_fetch(imu_dev) == 0) {
            struct sensor_value accel[3], gyro[3];

            sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_X, &accel[0]);
            sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Y, &accel[1]);
            sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Z, &accel[2]);
            sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_X, &gyro[0]);
            sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Y, &gyro[1]);
            sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Z, &gyro[2]);

            imu_msg.linear_acceleration.x = sensor_value_to_double(&accel[0]);
            imu_msg.linear_acceleration.y = sensor_value_to_double(&accel[1]);
            imu_msg.linear_acceleration.z = sensor_value_to_double(&accel[2]);
            imu_msg.angular_velocity.x = sensor_value_to_double(&gyro[0]);
            imu_msg.angular_velocity.y = sensor_value_to_double(&gyro[1]);
            imu_msg.angular_velocity.z = sensor_value_to_double(&gyro[2]);

            if (ros_initialized) {
                rcl_ret_t rc = rcl_publish(&imu_pub, &imu_msg, NULL);
                if (rc != RCL_RET_OK) {
                    LOG_ERR("IMU publish failed: %d", rc);
                }
            }
        } else {
            LOG_WRN("Failed to fetch sensor sample");
        }

        // k_sleep(K_MSEC(1));
    }
}

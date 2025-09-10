#include "sensors.h"
#include "ros_iface.h"
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/imu.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/icm42x70.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sensors, LOG_LEVEL_INF);

static K_THREAD_STACK_DEFINE(sensors_stack, 2048);
static struct k_thread sensors_thread_data;

static const struct device *imu_dev;
static sensor_msgs__msg__Imu imu_msg;

#define IMU_NODE DT_ALIAS(imu)
#if DT_NODE_HAS_STATUS(IMU_NODE, okay)
static const struct i2c_dt_spec imu_i2c = I2C_DT_SPEC_GET(IMU_NODE);

static void imu_soft_reset_try(void)
{
    if (!device_is_ready(imu_i2c.bus)) {
        return;
    }

    uint8_t buf;
    /* Force bank 0 */
    buf = 0x00; (void)i2c_burst_write_dt(&imu_i2c, 0x7C, &buf, 1);
    buf = 0x00; (void)i2c_burst_write_dt(&imu_i2c, 0x79, &buf, 1);
    /* Soft reset */
    buf = 0x10; (void)i2c_burst_write_dt(&imu_i2c, 0x02, &buf, 1);
    k_msleep(20);
}
#endif

void sensors_init(void) {
    LOG_INF("Initializing sensors");
    k_sleep(K_MSEC(500)); // Wait a bit for system to stabilize
    k_thread_create(&sensors_thread_data, sensors_stack, K_THREAD_STACK_SIZEOF(sensors_stack),
                    sensors_thread, NULL, NULL, NULL,
                    6, 0, K_NO_WAIT);
}

void sensors_thread(void *p1, void *p2, void *p3) {
    LOG_INF("Initializing sensors");
    backtobeginning:
    k_sleep(K_MSEC(500)); // Wait a bit for system to stabilize

    
    const struct device *dev = DEVICE_DT_GET(DT_ALIAS(imu));
    for (int tries = 0; !device_is_ready(dev) && tries < 999; ++tries) {
        LOG_DBG("IMU device not ready, retrying...");
        k_sleep(K_MSEC(200));
#if DT_NODE_HAS_STATUS(IMU_NODE, okay)
        if (tries == 2 || tries == 5) {
            imu_soft_reset_try();
        }
#endif
    }
    imu_dev = dev;
    if (device_is_ready(imu_dev)) {
        LOG_INF("IMU device ready: %s", imu_dev->name);
        // Configure sensor like in BMI270 sample: set FS and ODR explicitly
        struct sensor_value v;
        int rc;

        // Accelerometer: 16 g, low-noise mode, 100 Hz
        v.val1 = 16; v.val2 = 0;
        rc = sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &v);
        if (rc) { LOG_WRN("Accel FS set failed: %d", rc); }

        v.val1 = ICM42X70_LOW_NOISE_MODE; v.val2 = 0;
        rc = sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_CONFIGURATION, &v);
        if (rc) { LOG_WRN("Accel power mode set failed: %d", rc); }

        v.val1 = 100; v.val2 = 0;
        rc = sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &v);
        if (rc) { LOG_WRN("Accel ODR set failed: %d", rc); }

        // Gyro: 2000 dps, 100 Hz (only valid for 42670P/S)
        v.val1 = 2000; v.val2 = 0;
        rc = sensor_attr_set(imu_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_FULL_SCALE, &v);
        if (rc) { LOG_WRN("Gyro FS set failed: %d", rc); }

        v.val1 = 100; v.val2 = 0;
        rc = sensor_attr_set(imu_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &v);
        if (rc) { LOG_WRN("Gyro ODR set failed: %d", rc); }
    } else {
        LOG_WRN("IMU device not ready");
        goto backtobeginning;
    }

        while(true){
        sensors_imu_quick_check();
        k_sleep(K_MSEC(1000));
    }
    return;

    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    LOG_INF("Sensors thread started");

    while (1) {
        if (imu_dev && sensor_sample_fetch(imu_dev) == 0) {
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

            imu_msg.header.stamp.sec = (int32_t)(ros_iface_epoch_millis() / 1000ULL);
            imu_msg.header.stamp.nanosec = (uint32_t)(ros_iface_epoch_nanos() % 1000000000ULL);

            strncpy(imu_msg.header.frame_id.data, "imu", imu_msg.header.frame_id.capacity);
            imu_msg.header.frame_id.size = strlen("imu");
            imu_msg.header.frame_id.capacity = imu_msg.header.frame_id.size + 1;

            // Example: set diagonal variances, rest zero
            imu_msg.orientation_covariance[0] = -1; // No orientation estimate
            for (int i = 1; i < 9; i++)
                imu_msg.orientation_covariance[i] = 0;

            imu_msg.angular_velocity_covariance[0] = 0.01; // X variance
            imu_msg.angular_velocity_covariance[4] = 0.01; // Y variance
            imu_msg.angular_velocity_covariance[8] = 0.01; // Z variance
            for (int i = 0; i < 9; i++) {
                if (i != 0 && i != 4 && i != 8)
                    imu_msg.angular_velocity_covariance[i] = 0;
            }

            imu_msg.linear_acceleration_covariance[0] = 0.1; // X variance
            imu_msg.linear_acceleration_covariance[4] = 0.1; // Y variance
            imu_msg.linear_acceleration_covariance[8] = 0.1; // Z variance
            for (int i = 0; i < 9; i++) {
                if (i != 0 && i != 4 && i != 8)
                    imu_msg.linear_acceleration_covariance[i] = 0;
            }

            if (ros_initialized) {
                rcl_ret_t rc = rcl_publish(&imu_pub, &imu_msg, NULL);
                if (rc != RCL_RET_OK) {
                    LOG_ERR("IMU publish failed: %d", rc);
                }
            }
        } else {
            LOG_WRN("Failed to fetch sensor sample");
        }

        k_sleep(K_MSEC(5));
    }
}

int sensors_imu_quick_check(void) {
    k_sleep(K_MSEC(1000));
    const struct device *dev = imu_dev;
    if (dev == NULL) {
        dev = DEVICE_DT_GET(DT_ALIAS(imu));
    }
    if (dev == NULL) {
        LOG_ERR("IMU: device not found (alias 'imu' missing or label unset)");
        return -ENODEV;
    }
    if (!device_is_ready(dev)) {
        LOG_ERR("IMU: device '%s' not ready", dev->name);
        return -EBUSY;
    }

    int rc = sensor_sample_fetch(dev);
    if (rc) {
        LOG_ERR("IMU: sample_fetch failed (%d)", rc);
        return rc;
    }

    struct sensor_value acc[3] = {0};
    struct sensor_value gyr[3] = {0};

    rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, acc);
    if (rc == 0) {
        rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyr);
    }
    if (rc) {
        LOG_ERR("IMU: channel_get failed (%d)");
        return rc;
    }

    double ax = sensor_value_to_double(&acc[0]);
    double ay = sensor_value_to_double(&acc[1]);
    double az = sensor_value_to_double(&acc[2]);
    double gx = sensor_value_to_double(&gyr[0]);
    double gy = sensor_value_to_double(&gyr[1]);
    double gz = sensor_value_to_double(&gyr[2]);

    LOG_INF("IMU OK: acc[m/s^2]=[%.3f %.3f %.3f] gyro[rad/s]=[%.3f %.3f %.3f]",
            ax, ay, az, gx, gy, gz);
    return 0;
}

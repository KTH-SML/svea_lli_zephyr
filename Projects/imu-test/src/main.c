/* Minimal IMU test using the Zephyr ICM42670 sensor driver */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

LOG_MODULE_REGISTER(imu_test, LOG_LEVEL_INF);

#define IMU_NODE DT_ALIAS(imu)
#if !DT_NODE_HAS_STATUS(IMU_NODE, okay)
#error "No 'imu' alias found or it is disabled in the devicetree"
#endif

int main(void)
{
    const struct device *imu = DEVICE_DT_GET(IMU_NODE);
    k_sleep(K_MSEC(300));
    printk("IMU sensor driver test starting\n");

    while (!device_is_ready(imu)) {
        LOG_ERR("ICM42670 device not ready");
        device_init(imu);
        k_sleep(K_MSEC(10));
    }

    /* Optional: set ODR to 100 Hz for both accel and gyro */
    struct sensor_value odr;
    odr.val1 = 100; odr.val2 = 0;
    (void)sensor_attr_set(imu, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);
    (void)sensor_attr_set(imu, SENSOR_CHAN_GYRO_XYZ,  SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);

    while (1) {
        int rc = sensor_sample_fetch(imu);
        if (rc) {
            LOG_WRN("sample_fetch: %d", rc);
            k_sleep(K_MSEC(100));
            continue;
        }

        struct sensor_value acc[3], gyr[3];
        rc = sensor_channel_get(imu, SENSOR_CHAN_ACCEL_XYZ, acc);
        if (rc == 0) {
            rc = sensor_channel_get(imu, SENSOR_CHAN_GYRO_XYZ,  gyr);
        }

        if (rc == 0) {
            double ax = sensor_value_to_double(&acc[0]);
            double ay = sensor_value_to_double(&acc[1]);
            double az = sensor_value_to_double(&acc[2]);
            double gx = sensor_value_to_double(&gyr[0]);
            double gy = sensor_value_to_double(&gyr[1]);
            double gz = sensor_value_to_double(&gyr[2]);
            printk("ACC [m/s^2]=[% .3f % .3f % .3f]  GYR [rad/s]=[% .3f % .3f % .3f]\n",
                   ax, ay, az, gx, gy, gz);
        } else {
            LOG_WRN("channel_get: %d", rc);
        }

        k_sleep(K_MSEC(100));
    }
}

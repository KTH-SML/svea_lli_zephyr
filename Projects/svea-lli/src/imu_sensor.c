#include "imu_sensor.h"
#include "ros_iface.h"

#include <errno.h>
#include <limits.h>
#include <math.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/imu.h>
#include <stdbool.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(imu_sensor, LOG_LEVEL_INF);

static K_THREAD_STACK_DEFINE(imu_sensor_stack, 2048);
static struct k_thread imu_thread_data;

static const struct device *imu_dev;
static sensor_msgs__msg__Imu imu_msg;

static void imu_sensor_thread(void *p1, void *p2, void *p3);
static int imu_device_retry_init(const struct device *dev);

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define IMU_ACCEL_NOISE_DENSITY_G (90e-6) /* 90 µg/√Hz from datasheet */
#define IMU_GYRO_NOISE_DENSITY_MDPS (3.8) /* 3.8 mdps/√Hz from datasheet */
#define STANDARD_GRAVITY (9.80665)
/* With ODR set to 100 Hz we have BW ≈ ODR/2 */
#define IMU_EFFECTIVE_BW_HZ (50.0)

static const double acc_noise_rms = (IMU_ACCEL_NOISE_DENSITY_G * STANDARD_GRAVITY) *
                                    sqrt(IMU_EFFECTIVE_BW_HZ);
static const double gyro_noise_rms = ((IMU_GYRO_NOISE_DENSITY_MDPS * 1e-3) * (M_PI / 180.0)) *
                                     sqrt(IMU_EFFECTIVE_BW_HZ);
static const double acc_variance = acc_noise_rms * acc_noise_rms;
static const double gyro_variance = gyro_noise_rms * gyro_noise_rms;

#if DT_NODE_HAS_PROP(DT_PATH(zephyr_user), imu_reset_gpios)
static const struct gpio_dt_spec imu_reset_gpio =
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), imu_reset_gpios);
#define HAS_IMU_RESET_GPIO 1
#else
#define HAS_IMU_RESET_GPIO 0
#endif

static int imu_device_retry_init(const struct device *dev) {
    extern const struct init_entry __init_POST_KERNEL_start[];
    extern const struct init_entry __init_APPLICATION_start[];
    const struct init_entry *entry;

    for (entry = __init_POST_KERNEL_start; entry < __init_APPLICATION_start; ++entry) {
        if ((entry->dev != dev) || (entry->init_fn.dev == NULL)) {
            continue;
        }

        dev->state->initialized = false;
        dev->state->init_res = 0U;

        int rc = entry->init_fn.dev(dev);

        if (rc != 0) {
            unsigned int stored = (unsigned int)((rc < 0) ? MIN(-rc, (int)UINT8_MAX)
                                                          : MIN(rc, (int)UINT8_MAX));
            dev->state->init_res = (uint8_t)stored;
        } else if (IS_ENABLED(CONFIG_PM_DEVICE_RUNTIME)) {
            (void)pm_device_runtime_auto_enable(dev);
        }

        dev->state->initialized = true;
        return rc;
    }

    return -ENOENT;
}

static int device_init_status(const struct device *dev) {
    const struct device_state *state = dev->state;

    if (state == NULL) {
        return -EIO;
    }

    if (!state->initialized) {
        int rc = device_init(dev);
        LOG_DBG("IMU device_init() invoked manually, rc=%d", rc);
        return rc;
    }

    if (state->init_res != 0U) {
        int rc = -((int)state->init_res);
        LOG_DBG("IMU driver cached init_res=%u (errno=%d)", state->init_res, rc);
        return rc;
    }

    return 0;
}

void imu_sensor_start(void) {
    LOG_INF("Starting IMU sensor thread");
    k_thread_create(&imu_thread_data, imu_sensor_stack, K_THREAD_STACK_SIZEOF(imu_sensor_stack),
                    imu_sensor_thread, NULL, NULL, NULL, 6, 0, K_NO_WAIT);
}

static void imu_sensor_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    LOG_INF("Initializing IMU");

#if HAS_IMU_RESET_GPIO
    if (!gpio_is_ready_dt(&imu_reset_gpio)) {
        LOG_WRN("IMU reset GPIO not ready");
    } else {
        int rst_rc = gpio_pin_configure_dt(&imu_reset_gpio, GPIO_OUTPUT_ACTIVE);
        if (rst_rc) {
            LOG_WRN("Failed to configure IMU reset GPIO: %d", rst_rc);
        } else {
            /* Assert reset (active low) briefly before release */
            gpio_pin_set_dt(&imu_reset_gpio, 0);
            k_msleep(5);
            gpio_pin_set_dt(&imu_reset_gpio, 1);
            k_msleep(5);
        }
    }
#endif

    const struct device *dev = DEVICE_DT_GET(DT_ALIAS(imu));
    const int MAX_IMU_INIT_ATTEMPTS = 20;
    int init_attempts = 0;

    while (dev && !device_is_ready(dev) && (init_attempts < MAX_IMU_INIT_ATTEMPTS)) {
        int rc = device_init_status(dev);
        if (rc != 0) {
            LOG_WRN("IMU init attempt %d/%d failed (rc=%d)", init_attempts + 1,
                    MAX_IMU_INIT_ATTEMPTS, rc);
        }

        int retry = imu_device_retry_init(dev);
        init_attempts++;

        if (retry) {
            LOG_WRN("IMU re-init attempt %d/%d failed: %d", init_attempts, MAX_IMU_INIT_ATTEMPTS, retry);
        } else {
            LOG_INF("IMU re-init attempt %d/%d succeeded", init_attempts, MAX_IMU_INIT_ATTEMPTS);
        }

        if (!device_is_ready(dev)) {
            k_msleep(100);
        }
    }

    if (!dev || !device_is_ready(dev)) {
        LOG_ERR("IMU initialization failed after %d attempts, giving up", init_attempts);
        return;
    }

    imu_dev = dev;
    LOG_INF("IMU device loop starting: %s", imu_dev->name);

    struct sensor_value accel[3];
    struct sensor_value gyro[3];

    while (true) {
        int rc = sensor_sample_fetch(imu_dev);
        if (rc) {
            LOG_WRN("IMU sample fetch failed: %d", rc);
            k_sleep(K_MSEC(100));
            continue;
        }

        int accel_rc = sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
        int gyro_rc = sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_XYZ, gyro);

        if (accel_rc || gyro_rc) {
            LOG_WRN("IMU channel read failed: accel=%d, gyro=%d", accel_rc, gyro_rc);
            k_sleep(K_MSEC(100));
            continue;
        }
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

        imu_msg.orientation_covariance[0] = -1;
        for (int i = 1; i < 9; i++) {
            imu_msg.orientation_covariance[i] = 0;
        }

        imu_msg.angular_velocity_covariance[0] = gyro_variance;
        imu_msg.angular_velocity_covariance[4] = gyro_variance;
        imu_msg.angular_velocity_covariance[8] = gyro_variance;
        for (int i = 0; i < 9; i++) {
            if ((i != 0) && (i != 4) && (i != 8)) {
                imu_msg.angular_velocity_covariance[i] = 0;
            }
        }

        imu_msg.linear_acceleration_covariance[0] = acc_variance;
        imu_msg.linear_acceleration_covariance[4] = acc_variance;
        imu_msg.linear_acceleration_covariance[8] = acc_variance;
        for (int i = 0; i < 9; i++) {
            if ((i != 0) && (i != 4) && (i != 8)) {
                imu_msg.linear_acceleration_covariance[i] = 0;
            }
        }

        if (ros_initialized) {
            rcl_ret_t pub_rc = rcl_publish(&imu_pub, &imu_msg, NULL);
            if (pub_rc != RCL_RET_OK) {
                LOG_ERR("IMU publish failed: %d", pub_rc);
            }
        } else {
            static uint64_t last_log_time;
            uint64_t now = k_uptime_get();
            if ((now - last_log_time) >= 5000U) {
                LOG_INF("IMU sample (accel=%.6f/%.6f/%.6f m/s^2, gyro=%.6f/%.6f/%.6f rad/s)",
                        (double)imu_msg.linear_acceleration.x,
                        (double)imu_msg.linear_acceleration.y,
                        (double)imu_msg.linear_acceleration.z,
                        (double)imu_msg.angular_velocity.x,
                        (double)imu_msg.angular_velocity.y,
                        (double)imu_msg.angular_velocity.z);
                last_log_time = now;
            }
        }
        k_sleep(K_MSEC(10)); // yield to lower-prio tasks at least a lil bit
    }
}

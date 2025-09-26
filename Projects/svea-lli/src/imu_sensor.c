#include "imu_sensor.h"
#include "ros_iface.h"

#include <errno.h>
#include <limits.h>
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
static struct sensor_trigger imu_trigger;
static K_SEM_DEFINE(imu_data_sem, 0, 1);

static void imu_sensor_thread(void *p1, void *p2, void *p3);
static int imu_device_retry_init(const struct device *dev);

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
        LOG_INF("IMU device_init() invoked manually, rc=%d", rc);
        return rc;
    }

    if (state->init_res != 0U) {
        int rc = -((int)state->init_res);
        LOG_ERR("IMU driver cached init_res=%u (errno=%d)", state->init_res, rc);
        return rc;
    }

    return 0;
}

static void imu_trigger_handler(const struct device *dev, const struct sensor_trigger *trig) {
    if ((trig->type != SENSOR_TRIG_DATA_READY) || (trig->chan != SENSOR_CHAN_ALL)) {
        return;
    }

    int rc = sensor_sample_fetch_chan(dev, trig->chan);
    if (rc < 0) {
        LOG_ERR("IMU sample fetch in trigger failed: %d", rc);
        return;
    }

    k_sem_give(&imu_data_sem);
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
        LOG_ERR("IMU driver cached init_res=%u (errno=%d)", dev->state->init_res, rc);

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
        LOG_ERR("IMU initialization failed after %d attempts, giving up", MAX_IMU_INIT_ATTEMPTS);
        return;
    }

    imu_dev = dev;
    LOG_INF("IMU device ready: %s", imu_dev->name);

    struct sensor_value v = {
        .val1 = 100,
        .val2 = 0,
    };
    int rc = sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &v);
    if (rc) {
        LOG_WRN("Accel ODR set failed: %d", rc);
    }

    v.val1 = 100;
    v.val2 = 0;
    rc = sensor_attr_set(imu_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &v);
    if (rc) {
        LOG_WRN("Gyro ODR set failed: %d", rc);
    }

    imu_trigger.type = SENSOR_TRIG_DATA_READY;
    imu_trigger.chan = SENSOR_CHAN_ALL;
    rc = sensor_trigger_set(imu_dev, &imu_trigger, imu_trigger_handler);
    if (rc) {
        LOG_ERR("Failed to configure IMU trigger: %d", rc);
    }

    LOG_INF("IMU thread started");

    struct sensor_value accel[3];
    struct sensor_value gyro[3];
    struct sensor_value temperature;

    while (true) {
        k_sem_take(&imu_data_sem, K_FOREVER);

        if (sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_XYZ, accel) ||
            sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_XYZ, gyro) ||
            sensor_channel_get(imu_dev, SENSOR_CHAN_DIE_TEMP, &temperature)) {
            LOG_WRN("Failed to read IMU channels after trigger");
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

        imu_msg.angular_velocity_covariance[0] = 0.01;
        imu_msg.angular_velocity_covariance[4] = 0.01;
        imu_msg.angular_velocity_covariance[8] = 0.01;
        for (int i = 0; i < 9; i++) {
            if ((i != 0) && (i != 4) && (i != 8)) {
                imu_msg.angular_velocity_covariance[i] = 0;
            }
        }

        imu_msg.linear_acceleration_covariance[0] = 0.1;
        imu_msg.linear_acceleration_covariance[4] = 0.1;
        imu_msg.linear_acceleration_covariance[8] = 0.1;
        for (int i = 0; i < 9; i++) {
            if ((i != 0) && (i != 4) && (i != 8)) {
                imu_msg.linear_acceleration_covariance[i] = 0;
            }
        }

        // LOG_INF("[%lld ms] temp %.2f Cel  accel %.6f %.6f %.6f m/s^2  gyro %.6f %.6f %.6f rad/s",
        //         (long long)k_uptime_get(),
        //         sensor_value_to_double(&temperature),
        //         imu_msg.linear_acceleration.x,
        //         imu_msg.linear_acceleration.y,
        //         imu_msg.linear_acceleration.z,
        //         imu_msg.angular_velocity.x,
        //         imu_msg.angular_velocity.y,
        //         imu_msg.angular_velocity.z);

        if (ros_initialized) {
            rcl_ret_t pub_rc = rcl_publish(&imu_pub, &imu_msg, NULL);
            if (pub_rc != RCL_RET_OK) {
                LOG_ERR("IMU publish failed: %d", pub_rc);
            }
        }
    }
}

int imu_sensor_quick_check(void) {
    const struct device *dev = DEVICE_DT_GET(DT_ALIAS(imu));

    if (!device_is_ready(dev)) {
        LOG_ERR("IMU device not ready for quick check");
        return -ENODEV;
    }

    struct sensor_value accel[3];
    struct sensor_value gyro[3];

    int rc = sensor_sample_fetch(dev);
    if (rc) {
        LOG_ERR("IMU quick check fetch failed: %d", rc);
        return rc;
    }

    rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
    if (rc) {
        return rc;
    }

    rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);
    if (rc) {
        return rc;
    }

    // LOG_INF("IMU quick check accel=(%.3f, %.3f, %.3f) gyro=(%.3f, %.3f, %.3f)",
    //         sensor_value_to_double(&accel[0]),
    //         sensor_value_to_double(&accel[1]),
    //         sensor_value_to_double(&accel[2]),
    //         sensor_value_to_double(&gyro[0]),
    //         sensor_value_to_double(&gyro[1]),
    //         sensor_value_to_double(&gyro[2]));

    return 0;
}

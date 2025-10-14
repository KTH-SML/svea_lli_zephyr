#/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
# * SVEA Note.
# *
# * Part of the SVEA Low‑Level Interface (Zephyr) application.
# * Author: Nils Kiefer
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

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

#if DT_NODE_HAS_PROP(DT_PATH(zephyr_user), imu_calib_button_gpios)
static const struct gpio_dt_spec imu_calib_button =
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), imu_calib_button_gpios);
#define IMU_CALIB_BUTTON_PRESENT 1
#else
#define IMU_CALIB_BUTTON_PRESENT 0
#endif

static K_THREAD_STACK_DEFINE(imu_sensor_stack, 2048);
static struct k_thread imu_thread_data;

static sensor_msgs__msg__Imu imu_msg;
static struct gpio_callback imu_calib_button_cb;

static void imu_calib_button_isr(const struct device *port, struct gpio_callback *cb, uint32_t pins) {
    ARG_UNUSED(port);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);
    LOG_INF("IMU calibration button interrupt");
    /* Defer heavy work to thread context */
    extern volatile bool g_imu_calib_request;
    g_imu_calib_request = true;
}

static void imu_sensor_thread(void *p1, void *p2, void *p3);

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define IMU_ACCEL_NOISE_DENSITY_G (90e-6) /* 90 µg/√Hz from datasheet */
#define IMU_GYRO_NOISE_DENSITY_MDPS (3.8) /* 3.8 mdps/√Hz from datasheet */
#define STANDARD_GRAVITY (-9.80665)
/* With ODR set to 100 Hz we have BW ≈ ODR/2 */
#define IMU_EFFECTIVE_BW_HZ (50.0)

// Covariance calculation based on IMU noise density and effective bandwidth
static const double acc_noise_rms = (IMU_ACCEL_NOISE_DENSITY_G * STANDARD_GRAVITY) *
                                    sqrt(IMU_EFFECTIVE_BW_HZ);
static const double gyro_noise_rms = ((IMU_GYRO_NOISE_DENSITY_MDPS * 1e-3) * (M_PI / 180.0)) *
                                     sqrt(IMU_EFFECTIVE_BW_HZ);
static const double acc_variance = acc_noise_rms * acc_noise_rms;
static const double gyro_variance = gyro_noise_rms * gyro_noise_rms;

/* ── calibration state ─────────────────────────────────────────────── */
volatile bool g_imu_calib_request = true; /* set from ISR */
static bool calib_in_progress;
static uint64_t calib_start_ms;
static double sum_ax, sum_ay, sum_az, sum_gx, sum_gy, sum_gz;
static uint32_t calib_count;
static double accel_bias[3]; /* add to raw to get corrected */
static double gyro_bias[3];  /* add to raw to get corrected */

/* ── simple rolling average (compile-time window) ───────────────────── */
#ifndef IMU_SMOOTH_WINDOW
#define IMU_SMOOTH_WINDOW 4
#endif

struct avg_win {
    double buf[IMU_SMOOTH_WINDOW];
    uint8_t idx;
    uint8_t len;
    double sum;
};

static inline void avg_reset(struct avg_win *w) {
    w->idx = 0;
    w->len = 0;
    w->sum = 0.0;
}

static inline void avg_push(struct avg_win *w, double v) {
    if (w->len < IMU_SMOOTH_WINDOW) {
        w->buf[w->idx] = v;
        w->sum += v;
        w->idx = (w->idx + 1) % IMU_SMOOTH_WINDOW;
        w->len++;
    } else {
        double old = w->buf[w->idx];
        w->buf[w->idx] = v;
        w->sum += v - old;
        w->idx = (w->idx + 1) % IMU_SMOOTH_WINDOW;
    }
}

static inline double avg_mean(const struct avg_win *w) {
    return (w->len ? (w->sum / (double)w->len) : 0.0);
}

static struct avg_win acc_win[3];
static struct avg_win gyro_win[3];

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

    const struct device *imu_dev = DEVICE_DT_GET(DT_ALIAS(imu));

    if (!imu_dev || !device_is_ready(imu_dev)) {
        LOG_ERR("IMU initialization failed, giving up");
        return;
    }
#if IMU_CALIB_BUTTON_PRESENT
    LOG_INF("IMU calibration button using %s pin %u",
            imu_calib_button.port->name, imu_calib_button.pin);
    if (!device_is_ready(imu_calib_button.port)) {
        LOG_ERR("IMU calibration button port not ready");
    } else {
        int ret = gpio_pin_configure_dt(&imu_calib_button, GPIO_INPUT);
        if (ret != 0) {
            LOG_ERR("IMU calibration button configure failed: %d", ret);
        } else {
            gpio_init_callback(&imu_calib_button_cb, imu_calib_button_isr,
                               BIT(imu_calib_button.pin));
            ret = gpio_add_callback(imu_calib_button.port, &imu_calib_button_cb);
            if (ret != 0) {
                LOG_ERR("IMU calibration button callback add failed: %d", ret);
            } else {
                ret = gpio_pin_interrupt_configure_dt(&imu_calib_button, GPIO_INT_EDGE_TO_ACTIVE);
                if (ret != 0) {
                    LOG_ERR("IMU calibration button interrupt configure failed: %d", ret);
                } else {
                    LOG_INF("IMU calibration button interrupt configured");
                }
            }
        }
    }
#else
    LOG_WRN("IMU calibration button GPIO not defined in devicetree");
#endif

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

        /* Convert raw to doubles */
        double ax = sensor_value_to_double(&accel[0]);
        double ay = sensor_value_to_double(&accel[1]);
        double az = sensor_value_to_double(&accel[2]);
        double gx = sensor_value_to_double(&gyro[0]);
        double gy = sensor_value_to_double(&gyro[1]);
        double gz = sensor_value_to_double(&gyro[2]);

        /* Handle calibration trigger (1 s averaging window) */
        if (g_imu_calib_request && !calib_in_progress) {
            g_imu_calib_request = false;
            calib_in_progress = true;
            calib_start_ms = k_uptime_get();
            sum_ax = sum_ay = sum_az = 0.0;
            sum_gx = sum_gy = sum_gz = 0.0;
            calib_count = 0;
            LOG_INF("IMU calibration: started 1s averaging");
        }
        if (calib_in_progress) {
            sum_ax += ax;
            sum_ay += ay;
            sum_az += az;
            sum_gx += gx;
            sum_gy += gy;
            sum_gz += gz;
            calib_count++;
            if ((k_uptime_get() - calib_start_ms) >= 1000U) {
                /* Compute means */
                double n = (calib_count > 0) ? (double)calib_count : 1.0;
                double mean_ax = sum_ax / n;
                double mean_ay = sum_ay / n;
                double mean_az = sum_az / n;
                double mean_gx = sum_gx / n;
                double mean_gy = sum_gy / n;
                double mean_gz = sum_gz / n;

                /* Bias so that corrected = raw + bias → targets are (0,0,-g) and (0,0,0) */
                accel_bias[0] = 0.0 - mean_ax;
                accel_bias[1] = 0.0 - mean_ay;
                accel_bias[2] = -STANDARD_GRAVITY - mean_az;
                gyro_bias[0] = 0.0 - mean_gx;
                gyro_bias[1] = 0.0 - mean_gy;
                gyro_bias[2] = 0.0 - mean_gz;

                calib_in_progress = false;
                /* Reset smoothing windows to avoid mixing pre/post-bias */
                for (int i = 0; i < 3; ++i) {
                    avg_reset(&acc_win[i]);
                    avg_reset(&gyro_win[i]);
                }
                LOG_INF("IMU calibration: done (%u samples)\n"
                        "  accel bias = [%.6f, %.6f, %.6f]\n"
                        "  gyro  bias = [%.6f, %.6f, %.6f]",
                        calib_count,
                        accel_bias[0], accel_bias[1], accel_bias[2],
                        gyro_bias[0], gyro_bias[1], gyro_bias[2]);
            }
        }

        /* Apply biases and push into smoothing windows */
        double ax_c = ax + accel_bias[0];
        double ay_c = ay + accel_bias[1];
        double az_c = az + accel_bias[2];
        double gx_c = gx + gyro_bias[0];
        double gy_c = gy + gyro_bias[1];
        double gz_c = gz + gyro_bias[2];

        avg_push(&acc_win[0], ax_c);
        avg_push(&acc_win[1], ay_c);
        avg_push(&acc_win[2], az_c);
        avg_push(&gyro_win[0], gx_c);
        avg_push(&gyro_win[1], gy_c);
        avg_push(&gyro_win[2], gz_c);

        imu_msg.linear_acceleration.x = avg_mean(&acc_win[0]);
        imu_msg.linear_acceleration.y = avg_mean(&acc_win[1]);
        imu_msg.linear_acceleration.z = avg_mean(&acc_win[2]);
        imu_msg.angular_velocity.x = avg_mean(&gyro_win[0]);
        imu_msg.angular_velocity.y = avg_mean(&gyro_win[1]);
        imu_msg.angular_velocity.z = avg_mean(&gyro_win[2]);

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
            rcl_ret_t pub_rc = ros_publish_try(&imu_pub, &imu_msg);
            if (pub_rc != RCL_RET_OK) {
                /* Skip if transport busy to keep rate consistent */
                /* LOG_DBG("IMU publish skipped (busy)"); */
            }
        } else {
            static uint64_t last_log_time;
            uint64_t now = k_uptime_get();
            if ((now - last_log_time) >= 50000U) {
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
        k_sleep(K_MSEC(30)); // yield to lower-prio tasks at least a lil bit
    }
}

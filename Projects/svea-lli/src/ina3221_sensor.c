#/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
# * SVEA Note.
# *
# * Part of the SVEA Low‑Level Interface (Zephyr) application.
# * Author: Nils Kiefer
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "ina3221_sensor.h"
#include "ros_iface.h"

#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <stdbool.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ina3221_sensor, LOG_LEVEL_INF);

#ifndef SENSOR_ATTR_INA3221_SELECTED_CHANNEL
#define SENSOR_ATTR_INA3221_SELECTED_CHANNEL (SENSOR_ATTR_PRIV_START + 1)
#endif

#define INA3221_THREAD_STACK_SIZE 1536
#define INA3221_THREAD_PRIORITY 5
#define INA3221_SAMPLE_PERIOD_MS 200
#define ESC_UNDERVOLT_RECOVER_TIMEOUT 5000U
#define ESC_LED_BLINK_MS 500U

static const struct device *ina_dev;
static K_THREAD_STACK_DEFINE(ina_thread_stack, INA3221_THREAD_STACK_SIZE);
static struct k_thread ina_thread_data;
static struct ina3221_measurement latest_measurement;
static bool latest_valid;
static struct k_mutex measurement_lock;
static std_msgs__msg__Float32MultiArray ina_msg;
static bool ina_msg_initialized;

static int ina3221_set_channel(uint8_t channel) {
    struct sensor_value sel = {
        .val1 = channel + 1, /* driver expects channels 1-3 */
        .val2 = 0,
    };

    return sensor_attr_set(ina_dev, SENSOR_CHAN_ALL,
                           SENSOR_ATTR_INA3221_SELECTED_CHANNEL, &sel);
}

static const char *ina3221_channel_label(uint8_t ch) {
    switch (ch) {
    case 0:
        return "ESC";
    case 1:
        return "12V";
    case 2:
        return "5V";
    default:
        return "UNKNOWN";
    }
}

static int ina3221_read_channel(uint8_t ch, struct ina3221_measurement *sample) {
    int rc = ina3221_set_channel(ch);
    if (rc) {
        return rc;
    }

    rc = sensor_sample_fetch_chan(ina_dev, SENSOR_CHAN_ALL);
    if (rc) {
        return rc;
    }

    rc = sensor_channel_get(ina_dev, SENSOR_CHAN_VOLTAGE, &sample->bus_voltage[ch]);
    if (rc) {
        return rc;
    }

    rc = sensor_channel_get(ina_dev, SENSOR_CHAN_CURRENT, &sample->shunt_current[ch]);
    if (rc) {
        return rc;
    }

    rc = sensor_channel_get(ina_dev, SENSOR_CHAN_POWER, &sample->power[ch]);
    if (rc) {
        return rc;
    }
    return 0;
}

static void ina3221_thread(void *a, void *b, void *c) {
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    struct ina3221_measurement sample;

    static int last_publish_time = 0;
    while (true) {
        int rc = 0;

        memset(&sample, 0, sizeof(sample));

        for (uint8_t ch = 0U; ch < 3U; ++ch) {
            rc = ina3221_read_channel(ch, &sample);
            if (rc) {
                LOG_WRN("INA3221 channel %u (%s) read failed: %d", ch,
                        ina3221_channel_label(ch), rc);
                break;
            }
        }

        if (rc == 0) {
            k_mutex_lock(&measurement_lock, K_FOREVER);
            memcpy(&latest_measurement, &sample, sizeof(sample));
            latest_valid = true;
            k_mutex_unlock(&measurement_lock);

            if (ros_initialized && ina_msg_initialized) {

                /* Data layout: [ESC_V, ESC_I, ESC_P, 12V_V, 12V_I, 12V_P, 5V_V, 5V_I, 5V_P] */
                ina_msg.data.data[0] = sensor_value_to_float(&sample.bus_voltage[0]);
                ina_msg.data.data[1] = sensor_value_to_float(&sample.shunt_current[0]);
                ina_msg.data.data[2] = sensor_value_to_float(&sample.power[0]);
                ina_msg.data.data[3] = sensor_value_to_float(&sample.bus_voltage[1]);
                ina_msg.data.data[4] = sensor_value_to_float(&sample.shunt_current[1]);
                ina_msg.data.data[5] = sensor_value_to_float(&sample.power[1]);
                ina_msg.data.data[6] = sensor_value_to_float(&sample.bus_voltage[2]);
                ina_msg.data.data[7] = sensor_value_to_float(&sample.shunt_current[2]);
                ina_msg.data.data[8] = sensor_value_to_float(&sample.power[2]);

                (void)ros_publish_try(&ina3221_pub, &ina_msg);

            } else {
                // Log to serial if ROS is not connected
                static uint64_t last_log_time;
                uint64_t now = k_uptime_get();
                if ((now - last_log_time) >= 10000U) {
                    LOG_INF("INA3221 sample: ESC V=%.3fV I=%.3fA P=%.3fW | 12V V=%.3fV I=%.3fA P=%.3fW | 5V V=%.3fV I=%.3fA P=%.3fW",
                            sensor_value_to_double(&sample.bus_voltage[0]),
                            sensor_value_to_double(&sample.shunt_current[0]),
                            sensor_value_to_double(&sample.power[0]),
                            sensor_value_to_double(&sample.bus_voltage[1]),
                            sensor_value_to_double(&sample.shunt_current[1]),
                            sensor_value_to_double(&sample.power[1]),
                            sensor_value_to_double(&sample.bus_voltage[2]),
                            sensor_value_to_double(&sample.shunt_current[2]),
                            sensor_value_to_double(&sample.power[2]));
                    last_log_time = now;
                }
            }
        }
        // For esc cutout
        k_sleep(K_MSEC(INA3221_SAMPLE_PERIOD_MS));
    }
}

int ina3221_sensor_init(void) {
    if (latest_valid) {
        return 0;
    }

    ina_dev = DEVICE_DT_GET_OR_NULL(DT_ALIAS(ina3221));
    if (ina_dev == NULL) {
        LOG_ERR("INA3221 device alias not found");
        return -ENODEV;
    }

    if (!device_is_ready(ina_dev)) {
        LOG_ERR("INA3221 device %s not ready", ina_dev->name);
        return -ENODEV;
    }

    k_mutex_init(&measurement_lock);

    if (!ina_msg_initialized) {
        if (!std_msgs__msg__Float32MultiArray__init(&ina_msg)) {
            LOG_ERR("Failed to init INA3221 ROS message");
        } else if (!rosidl_runtime_c__float32__Sequence__init(&ina_msg.data, 9)) {
            LOG_ERR("Failed to init INA3221 data sequence");
        } else {
            memset(ina_msg.data.data, 0, ina_msg.data.size * sizeof(float));
            ina_msg_initialized = true;
        }
    }

    k_thread_create(&ina_thread_data, ina_thread_stack, INA3221_THREAD_STACK_SIZE,
                    ina3221_thread, NULL, NULL, NULL,
                    INA3221_THREAD_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&ina_thread_data, "ina3221");

    LOG_INF("INA3221 polling thread started");

    return 0;
}

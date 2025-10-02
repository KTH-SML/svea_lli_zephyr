// SPDX-License-Identifier: Apache-2.0

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "bms.h"
#include "ros_iface.h"
#include "wake_chip.h"
// bq769x0 low-level access for debug prints
#include "interface.h"
#include "registers.h"

#include <math.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <rosidl_runtime_c/string_functions.h>
#include <sensor_msgs/msg/battery_state.h>

LOG_MODULE_REGISTER(bms_task, CONFIG_LOG_DEFAULT_LEVEL);

// Global BMS instance accessible to other modules if needed
static Bms g_bms;

static float peak_charge_a;
static float peak_discharge_a;

#define BATTERY_PUBLISHER_STACK_SIZE 2048
#define BATTERY_PUBLISHER_PRIORITY 5

static K_THREAD_STACK_DEFINE(battery_thread_stack, BATTERY_PUBLISHER_STACK_SIZE);
static struct k_thread battery_thread_data;
static sensor_msgs__msg__BatteryState battery_msg;
static bool battery_msg_ready;
static const float BATTERY_NAN = NAN;

static uint8_t bms_health_from_flags(uint32_t flags) {
    if (flags == 0U) {
        return sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_GOOD;
    }

    if (flags & ((1UL << BMS_ERR_CHG_OVERTEMP) | (1UL << BMS_ERR_DIS_OVERTEMP) |
                 (1UL << BMS_ERR_FET_OVERTEMP) | (1UL << BMS_ERR_INT_OVERTEMP))) {
        return sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_OVERHEAT;
    }

    if (flags & ((1UL << BMS_ERR_CHG_UNDERTEMP) | (1UL << BMS_ERR_DIS_UNDERTEMP))) {
        return sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_COLD;
    }

    if (flags & (1UL << BMS_ERR_CELL_OVERVOLTAGE)) {
        return sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_OVERVOLTAGE;
    }

    if (flags & (1UL << BMS_ERR_CELL_UNDERVOLTAGE)) {
        return sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_DEAD;
    }

    return sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
}

static uint8_t bms_power_supply_status(const BmsStatus *status, float current_a) {
    if (current_a > 0.1f) {
        return sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_CHARGING;
    }
    if (current_a < -0.1f) {
        return sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_DISCHARGING;
    }
    if (status->full) {
        return sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_FULL;
    }
    return sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_NOT_CHARGING;
}

#if DT_NODE_HAS_PROP(DT_PATH(zephyr_user), soc_button_gpios)
static const struct gpio_dt_spec soc_button =
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), soc_button_gpios);
#define SOC_BUTTON_PRESENT 1
#else
#define SOC_BUTTON_PRESENT 0
#endif

static const struct gpio_dt_spec soc_led_pins[] = {
    GPIO_DT_SPEC_GET(DT_NODELABEL(led1), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led2), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led3), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led4), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led5), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led6), gpios),
};

static struct gpio_callback soc_button_cb;
static struct k_work soc_display_work;
static struct k_work_delayable soc_clear_work;
static bool soc_leds_ready;

static void soc_leds_off(void) {
    for (size_t i = 0; i < ARRAY_SIZE(soc_led_pins); ++i) {
        gpio_pin_set_dt(&soc_led_pins[i], 0);
    }
}

static void soc_clear_work_handler(struct k_work *work) {
    ARG_UNUSED(work);
    soc_leds_off();
}

static void soc_display_work_handler(struct k_work *work) {
    ARG_UNUSED(work);

    float soc = g_bms.status.soc;
    if (soc < 0.0f) {
        soc = 0.0f;
    } else if (soc > 100.0f) {
        soc = 100.0f;
    }

    static const float thresholds[] = {5.f, 25.f, 45.f, 65.f, 85.f, 97.f};
    int leds_on = 0;
    for (size_t i = 0; i < ARRAY_SIZE(thresholds) && i < ARRAY_SIZE(soc_led_pins); ++i) {
        if (soc >= thresholds[i]) {
            leds_on = (int)i + 1;
        }
    }

    for (int i = 0; i < (int)ARRAY_SIZE(soc_led_pins); ++i) {
        gpio_pin_set_dt(&soc_led_pins[i], 0);
    }

    for (int i = 0; i < leds_on; ++i) {
        gpio_pin_set_dt(&soc_led_pins[i], 1);
        k_msleep(60);
    }

    for (int i = leds_on; i < (int)ARRAY_SIZE(soc_led_pins); ++i) {
        gpio_pin_set_dt(&soc_led_pins[i], 0);
    }

    k_work_reschedule(&soc_clear_work, K_SECONDS(1));
}

static void soc_button_isr(const struct device *port, struct gpio_callback *cb, uint32_t pins) {
    ARG_UNUSED(port);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    if (soc_leds_ready) {
        k_work_submit(&soc_display_work);
    }
}

static void soc_indicator_init(void) {
#if SOC_BUTTON_PRESENT
    int err;

    for (size_t i = 0; i < ARRAY_SIZE(soc_led_pins); ++i) {
        if (!device_is_ready(soc_led_pins[i].port)) {
            LOG_WRN("SoC LED %zu port not ready", i);
            continue;
        }
        err = gpio_pin_configure_dt(&soc_led_pins[i], GPIO_OUTPUT_INACTIVE);
        if (err) {
            LOG_WRN("SoC LED %zu configure failed: %d", i, err);
        }
    }

    if (!device_is_ready(soc_button.port)) {
        LOG_WRN("SoC button port not ready");
        return;
    }

    err = gpio_pin_configure_dt(&soc_button, GPIO_INPUT);
    if (err) {
        LOG_WRN("SoC button configure failed: %d", err);
        return;
    }

    err = gpio_pin_interrupt_configure_dt(&soc_button, GPIO_INT_EDGE_TO_ACTIVE);
    if (err) {
        LOG_WRN("SoC button interrupt configure failed: %d", err);
        return;
    }

    gpio_init_callback(&soc_button_cb, soc_button_isr, BIT(soc_button.pin));
    gpio_add_callback(soc_button.port, &soc_button_cb);

    k_work_init(&soc_display_work, soc_display_work_handler);
    k_work_init_delayable(&soc_clear_work, soc_clear_work_handler);

    soc_leds_ready = true;
    LOG_INF("SoC LED indicator ready");
#endif
}

static void print_bms_status(const Bms *bms) {

    return;
    if (bms->status.pack_current > peak_charge_a) {
        peak_charge_a = bms->status.pack_current;
    }
    if (bms->status.pack_current < peak_discharge_a) {
        peak_discharge_a = bms->status.pack_current;
    }

    printf("\n--- BMS Telemetry ---\n");
    static const char *state_names[] = {"OFF", "CHG", "DIS", "NORMAL", "SHUTDOWN"};
    printf("State: %s (%u)  |  SoC: %.1f %%  |  Full: %d  Empty: %d\n",
           state_names[bms->status.state], bms->status.state, bms->status.soc, bms->status.full,
           bms->status.empty);

    printf("Pack Voltage: %.3f V  |  Pack Current: %.3f A\n", bms->status.pack_voltage,
           bms->status.pack_current);
    printf("Peak Currents: Charge %.3f A  Discharge %.3f A\n", peak_charge_a, peak_discharge_a);
    printf("Stack Voltage: %.3f V\n", bms->status.stack_voltage);

    printf("Cell Voltages: ");
    for (int i = 0; i < 5; i++) {
        printf("%.3f V ", bms->status.cell_voltages[i]);
    }
    printf("\nCell Min: %.3f V  |  Avg: %.3f V  |  Max: %.3f V\n", bms->status.cell_voltage_min,
           bms->status.cell_voltage_avg, bms->status.cell_voltage_max);

    printf("Temps: BatMin %.1f °C  BatAvg %.1f °C  BatMax %.1f °C\n", bms->status.bat_temp_min,
           bms->status.bat_temp_avg, bms->status.bat_temp_max);
    printf("       MOSFET %.1f °C  IC %.1f °C  MCU %.1f °C\n", bms->status.mosfet_temp,
           bms->status.ic_temp, bms->status.mcu_temp);

    printf("Balancing status: 0x%08x\n", bms->status.balancing_status);
    printf("Error flags:      0x%08x\n", bms->status.error_flags);

    if (bms->status.error_flags) {
        printf("  Active errors:\n");
        const char *err_desc[] = {
            "Cell undervoltage",                // 0
            "Cell overvoltage",                 // 1
            "Pack short circuit (discharge)",   // 2
            "Pack overcurrent (discharge)",     // 3
            "Pack overcurrent (charge)",        // 4
            "Cell open wire",                   // 5
            "Temp below discharge min",         // 6
            "Temp above discharge max",         // 7
            "Temp below charge min",            // 8
            "Temp above charge max",            // 9
            "Internal temp above limit",        // 10
            "Cell failure (voltage diff)",      // 11
            "Discharge FET off (should be on)", // 12
            "Charge FET off (should be on)",    // 13
            "MOSFET temp above limit"           // 14
        };
        for (int i = 0; i < 15; i++) {
            if (bms->status.error_flags & (1U << i)) {
                printf("    - %s\n", err_desc[i]);
            }
        }
    } else {
        printf("  No errors detected.\n");
    }
    uint8_t sys_stat = bq769x0_read_byte(BQ769X0_SYS_STAT);
    uint8_t protect1 = bq769x0_read_byte(BQ769X0_PROTECT1);
    uint8_t protect2 = bq769x0_read_byte(BQ769X0_PROTECT2);
    LOG_INF("SYS_STAT=0x%02x PROTECT1=0x%02x PROTECT2=0x%02x", sys_stat, protect1, protect2);
}

static bool battery_message_prepare(void) {
    if (battery_msg_ready) {
        return true;
    }

    if (!sensor_msgs__msg__BatteryState__init(&battery_msg)) {
        LOG_ERR("Failed to init BatteryState message");
        return false;
    }

    if (!rosidl_runtime_c__float__Sequence__init(&battery_msg.cell_voltage, BOARD_NUM_CELLS_MAX)) {
        LOG_ERR("Failed to init BatteryState cell_voltage sequence");
        sensor_msgs__msg__BatteryState__fini(&battery_msg);
        return false;
    }

    if (!rosidl_runtime_c__float__Sequence__init(&battery_msg.cell_temperature, BOARD_NUM_THERMISTORS_MAX)) {
        LOG_ERR("Failed to init BatteryState cell_temperature sequence");
        rosidl_runtime_c__float__Sequence__fini(&battery_msg.cell_voltage);
        sensor_msgs__msg__BatteryState__fini(&battery_msg);
        return false;
    }

    if (!rosidl_runtime_c__String__assign(&battery_msg.header.frame_id, "battery")) {
        LOG_WRN("Failed to set BatteryState frame_id");
    }

    (void)rosidl_runtime_c__String__assign(&battery_msg.location, "pack");
    // Leave serial_number empty by default

    battery_msg.design_capacity = BATTERY_NAN;
    battery_msg.capacity = BATTERY_NAN;
    battery_msg.charge = BATTERY_NAN;

    battery_msg_ready = true;
    return true;
}

static void battery_publisher_thread(void *, void *, void *) {
    k_thread_name_set(NULL, "battery_pub");

    uint32_t last_debug_ms = 0U;

    while (true) {
        if (!battery_message_prepare()) {
            k_sleep(K_SECONDS(1));
            continue;
        }

        const BmsStatus *status = &g_bms.status;
        const BmsConfig *conf = &g_bms.conf;

        battery_msg.header.stamp.sec = (int32_t)(ros_iface_epoch_millis() / 1000ULL);
        battery_msg.header.stamp.nanosec = (uint32_t)(ros_iface_epoch_nanos() % 1000000000ULL);

        battery_msg.voltage = status->pack_voltage;
        battery_msg.current = status->pack_current;
        battery_msg.temperature = status->bat_temp_avg;

        float soc_ratio = status->soc / 100.0f;
        if (soc_ratio < 0.0f) {
            soc_ratio = 0.0f;
        } else if (soc_ratio > 1.0f) {
            soc_ratio = 1.0f;
        }

        battery_msg.capacity = conf->nominal_capacity_Ah;
        battery_msg.design_capacity = conf->nominal_capacity_Ah;
        battery_msg.charge = (conf->nominal_capacity_Ah > 0.0f)
                                 ? conf->nominal_capacity_Ah * soc_ratio
                                 : BATTERY_NAN;
        battery_msg.percentage = soc_ratio;

        size_t voltage_count = 0U;
        for (size_t i = 0; i < BOARD_NUM_CELLS_MAX; ++i) {
            float v = status->cell_voltages[i];
            if (isfinite(v) && (v > 0.1f)) {
                battery_msg.cell_voltage.data[voltage_count++] = v;
            }
        }
        for (size_t i = voltage_count; i < battery_msg.cell_voltage.capacity; ++i) {
            battery_msg.cell_voltage.data[i] = BATTERY_NAN;
        }
        battery_msg.cell_voltage.size = voltage_count;

        size_t temp_count = 0U;
        for (size_t i = 0; i < BOARD_NUM_THERMISTORS_MAX; ++i) {
            float temp = status->bat_temps[i];
            if (isfinite(temp)) {
                battery_msg.cell_temperature.data[temp_count++] = temp;
            }
        }
        for (size_t i = temp_count; i < battery_msg.cell_temperature.capacity; ++i) {
            battery_msg.cell_temperature.data[i] = BATTERY_NAN;
        }
        battery_msg.cell_temperature.size = temp_count;

        battery_msg.present = (voltage_count > 0U);

        if (!battery_msg.present) {
            battery_msg.power_supply_status = sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_UNKNOWN;
            battery_msg.power_supply_health = sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_UNKNOWN;
        } else {
            battery_msg.power_supply_status = bms_power_supply_status(status, status->pack_current);
            battery_msg.power_supply_health = bms_health_from_flags(status->error_flags);
        }

        battery_msg.power_supply_technology = sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_LIPO;

        if (ros_initialized) {
            rcl_ret_t rc = rcl_publish(&battery_pub, &battery_msg, NULL);
            if (rc != RCL_RET_OK) {
                LOG_WRN("BatteryState publish failed: %d", rc);
            }
        } else {
            uint32_t now_ms = k_uptime_get_32();
            if ((now_ms - last_debug_ms) >= 1000U) {
                LOG_DBG("Battery: V=%.2f V I=%.2f A SoC=%.1f%% cell(min/avg/max)=%.3f/%.3f/%.3f flags=0x%08x state=%u",
                        (double)battery_msg.voltage,
                        (double)battery_msg.current,
                        (double)status->soc,
                        (double)status->cell_voltage_min,
                        (double)status->cell_voltage_avg,
                        (double)status->cell_voltage_max,
                        status->error_flags,
                        (unsigned int)status->state);
                last_debug_ms = now_ms;
            }
        }

        k_sleep(K_MSEC(500));
    }
}

static int setup_bms(void) {
    /* Proactively wake the BMS IC before first init attempt */
    if (wake_chip_init() == 0) {
        wake_chip_pulse_ms(50);
        wake_chip_release();
    }

    while (bms_init_hardware(&g_bms) != 0) {
        LOG_ERR("BMS hardware init failed, retry in 1s");
        if (wake_chip_init() == 0) {
            wake_chip_pulse_ms(50);
            wake_chip_release();
        }
        k_sleep(K_MSEC(1000));
    }
    LOG_INF("BMS hardware initialized");

    (void)bms_configure(&g_bms);
    bms_update(&g_bms);
    bms_soc_reset(&g_bms, -1);
    return 0;
}

// Initialize status + a reasonable default configuration early
static int bms_init_config_sys(void) {
    bms_init_status(&g_bms);
    // Default: generic LiPo, 8 Ah. Adjust via config later if needed.
    bms_init_config(&g_bms, CELL_TYPE_GENERIC_LIPO, 9);

    k_thread_create(&battery_thread_data, battery_thread_stack, BATTERY_PUBLISHER_STACK_SIZE,
                    battery_publisher_thread, NULL, NULL, NULL,
                    BATTERY_PUBLISHER_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&battery_thread_data, "battery_state");
    return 0;
}

SYS_INIT(bms_init_config_sys, APPLICATION, 0);

static void bms_thread(void *, void *, void *) {
    LOG_INF("BMS thread starting");
    soc_indicator_init();
    int64_t last_print = 0;

    while (true) {
        peak_charge_a = 0.0f;
        peak_discharge_a = 0.0f;
        setup_bms();
        int state = 0;
        while (state != -1) {
            bms_update(&g_bms);
            state = bms_state_machine(&g_bms);
            bms_soc_update(&g_bms);

            int64_t now = k_uptime_get();
            if (now - last_print >= 1000) {
                print_bms_status(&g_bms);
                last_print = now;
            }

            k_sleep(K_MSEC(100));
        }
        LOG_WRN("BMS state machine entered FATAL; reinitializing");
    }
}

// Priority lower than main; stack sized generously for I2C + logging
K_THREAD_DEFINE(bms_tid, 3072, bms_thread, NULL, NULL, NULL,
                3, 0, 0);

extern "C" float bms_get_soc_percent(void) {
    return g_bms.status.soc;
}

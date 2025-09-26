// SPDX-License-Identifier: Apache-2.0

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "bms.h"
#include "wake_chip.h"
// bq769x0 low-level access for debug prints
#include "interface.h"
#include "registers.h"

LOG_MODULE_REGISTER(bms_task, CONFIG_LOG_DEFAULT_LEVEL);

// Global BMS instance accessible to other modules if needed
static Bms g_bms;

static float peak_charge_a;
static float peak_discharge_a;

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

    const float step = 100.0f / ARRAY_SIZE(soc_led_pins);
    int leds_on = (int)((soc + (step - 1.0f)) / step);
    if (leds_on > (int)ARRAY_SIZE(soc_led_pins)) {
        leds_on = ARRAY_SIZE(soc_led_pins);
    }
    if (leds_on == 0 && soc > 0.0f) {
        leds_on = 1;
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
    bms_init_config(&g_bms, CELL_TYPE_GENERIC_LIPO, 8);
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

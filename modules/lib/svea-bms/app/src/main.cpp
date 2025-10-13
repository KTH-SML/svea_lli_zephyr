/*
 * Copyright (c) The Libre Solar Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef UNIT_TEST

#include "bms.h"
#include "button.h"
#include "data_objects.h"
#include "helper.h"
#include "leds.h"
#include "thingset.h"
#include "wake_chip.h"
#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include <stdio.h>

LOG_MODULE_REGISTER(bms_main, CONFIG_LOG_DEFAULT_LEVEL);

Bms bms;
static void print_bms_status(const Bms *bms)
{
    printf("\n--- BMS Telemetry ---\n");
    static const char *state_names[] = { "OFF", "CHG", "DIS", "NORMAL", "SHUTDOWN" };
    printf("State: %s (%u)  |  SoC: %.1f %%  |  Full: %d  Empty: %d\n",
           state_names[bms->status.state], bms->status.state, bms->status.soc, bms->status.full,
           bms->status.empty);

    printf("Pack Voltage: %.3f V  |  Pack Current: %.3f A\n", bms->status.pack_voltage,
           bms->status.pack_current);
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
            "MOSFET temp above limit",          // 14
            "No alert from monitor",            // 15
            "ADC calibration registers bad",    // 16
            "Measured cell voltage implausibly low",  // 17
            "Measured cell voltage implausibly high"  // 18
        };
        for (size_t i = 0; i < ARRAY_SIZE(err_desc); i++) {
            if (bms->status.error_flags & (1U << i)) {
                printf("    - %s\n", err_desc[i]);
            }
        }
    }
    else {
        printf("  No errors detected.\n");
    }
}

int setupBMS(void)
{
    // memset(&bms, 0, sizeof(Bms)); // Clear all fields to zero

    LOG_INF("Hardware: Libre Solar %s (%s)", DT_PROP(DT_PATH(pcb), type),
            DT_PROP(DT_PATH(pcb), version_str));
    LOG_INF("Firmware: %s", FIRMWARE_VERSION_ID);

    while (bms_init_hardware(&bms) != 0) {
        LOG_ERR("BMS hardware initialization failed, retrying in 1s");
        if (wake_chip_init() == 0) {
            wake_chip_pulse_ms(50);
            wake_chip_release(); //  TS1 floating after
        }
        k_sleep(K_MSEC(1));
    }

    bms_configure(&bms);

    bms_update(&bms);
    bms_soc_reset(&bms, -1);
    return 0;
}
int main(void)
{
    // setupBMS();
    //  button_init();

    int64_t t_start = k_uptime_get();
    int64_t last_print = 0;
    int stateMachineStatus = 0;
    while (true) {
        setupBMS();
        while (stateMachineStatus != -1) {
            bms_update(&bms);
            stateMachineStatus = bms_state_machine(&bms);
            bms_soc_update(&bms);
            int64_t now = k_uptime_get();
            if (now - last_print >= 5000) { // every 5 s
                printf("Time: %lld ms\n", now);
                print_bms_status(&bms);
                last_print = now;
            }

            k_sleep(K_MSEC(100));
        }
    }

    return 0;
}

static int init_config(void)
{
    bms_init_status(&bms);
    bms_init_config(&bms, CELL_TYPE_GENERIC_LIPO, 8);

    return 0;
}

/*
 * The default configuration must be initialized before the ThingSet storage backend reads data
 * from EEPROM or flash (with THINGSET_INIT_PRIORITY_STORAGE = 30).
 */
SYS_INIT(init_config, APPLICATION, 0);

#endif /* UNIT_TEST */

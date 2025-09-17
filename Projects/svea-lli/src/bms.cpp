// SPDX-License-Identifier: Apache-2.0

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "bms.h"
#include "wake_chip.h"
// bq769x0 low-level access for debug prints
#include "interface.h"
#include "registers.h"

LOG_MODULE_REGISTER(bms_task, CONFIG_LOG_DEFAULT_LEVEL);

// Global BMS instance accessible to other modules if needed
static Bms g_bms;

static void print_bms_status(const Bms *bms) {
    printf("\n--- BMS Telemetry ---\n");
    static const char *state_names[] = {"OFF", "CHG", "DIS", "NORMAL", "SHUTDOWN"};
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
    // FET debug: read SYS_CTRL2 and show CHG_ON / DSG_ON bits
    uint8_t sys2 = bq769x0_read_byte(BQ769X0_SYS_CTRL2);
    SYS_CTRL2_Type s2;
    s2.byte = sys2;
    printk("FETs: CHG_ON=%d  DSG_ON=%d  SYS_CTRL2=0x%02x\n",
           (int)s2.CHG_ON, (int)s2.DSG_ON, sys2);
    // Hint: to verify on hardware, measure MOSFET Vgs (gate-to-source), not gate-to-ground.
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
    int64_t last_print = 0;

    while (true) {
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
                K_PRIO_PREEMPT(5), 0, 0);

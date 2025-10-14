/*
 * Copyright (c) The Libre Solar Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bms.h"
#include "board.h"
#include "bq769x0/interface.h"
#include "helper.h"
#include "wake_chip.h"
#include <zephyr/kernel.h>

LOG_MODULE_REGISTER(bms, CONFIG_LOG_DEFAULT_LEVEL);

#ifndef BMS_PRECHARGE_MS
#define BMS_PRECHARGE_MS 1000
#endif

/* Hold-off after charge overcurrent to avoid NORMAL<->DIS thrash */
#ifndef BMS_CHG_OC_HOLDOFF_MS
#define BMS_CHG_OC_HOLDOFF_MS 500
#endif

/* When CHG overcurrent is detected, delay re-enabling CHG FET for a short time */
static int64_t chg_oc_resume_at_ms;

static float ocv_lfp[OCV_POINTS] = { 3.392F, 3.314F, 3.309F, 3.308F, 3.304F, 3.296F, 3.283F,
                                     3.275F, 3.271F, 3.268F, 3.265F, 3.264F, 3.262F, 3.252F,
                                     3.240F, 3.226F, 3.213F, 3.190F, 3.177F, 3.132F, 2.833F };

static float ocv_nmc[OCV_POINTS] = { 4.198F, 4.135F, 4.089F, 4.056F, 4.026F, 3.993F, 3.962F,
                                     3.924F, 3.883F, 3.858F, 3.838F, 3.819F, 3.803F, 3.787F,
                                     3.764F, 3.745F, 3.726F, 3.702F, 3.684F, 3.588F, 2.800F };
/* SOC points (descending): 100, 95, 90, …, 5, 0 (%) */
/* SOC points: 100, 95, 90, …, 5, 0 (%) */
static float ocv_lipo[OCV_POINTS] = { 4.200F, 4.120F, 4.080F, 4.050F, 4.020F, 3.990F, 3.960F,
                                      3.930F, 3.900F, 3.870F, 3.840F, 3.815F, 3.790F, 3.765F,
                                      3.740F, 3.710F, 3.680F, 3.650F, 3.600F, 3.450F, 3.200F };

/* Wrapper around discharge switch to apply precharge timing
 * - Enable: turn on precharge briefly before enabling main discharge FET
 * - Disable: enable precharge, turn off main FET, keep precharge for a short time, then disable
 */
static int bms_dis_switch_safe(Bms *bms, bool enable)
{
    static bool dsg_cmd_on;
    if (enable) {
        if (dsg_cmd_on == true) {
            return 0; // already on
        }
        (void)bq769x0_pchg_set(true);
        k_msleep(BMS_PRECHARGE_MS);
        int r = bms_dis_switch(bms, true);
        (void)bq769x0_pchg_set(false);
        if (r == 0) {
            dsg_cmd_on = true;
        }
        return r;
    }
    else {
        int r;
        if (dsg_cmd_on == false) {
            r = bms_dis_switch(bms, false);
            (void)bq769x0_pchg_set(false);
        }
        else {
            (void)bq769x0_pchg_set(true);
            int r = bms_dis_switch(bms, false);
            k_msleep(BMS_PRECHARGE_MS);
            (void)bq769x0_pchg_set(false);
        }
        if (r == 0) {
            dsg_cmd_on = false;
        }
        return r;
    }
}
void bms_init_status(Bms *bms)
{
    bms->status.chg_enable = true;
    bms->status.dis_enable = true;
}

void bms_init_config(Bms *bms, int type, float nominal_capacity)
{
    bms->conf.bal_idle_delay = 300;          // default: 300 sec
    bms->conf.bal_idle_current = 0.05F;      // A
    bms->conf.bal_cell_voltage_diff = 0.01F; // 10 mV

    bms->conf.thermistor_beta = 6000; // typical value for Semitec 103AT-5 thermistor

    bms->conf.nominal_capacity_Ah = nominal_capacity;

    bms->conf.dis_oc_limit = 15.0F;  // A discharge overcurrent
    bms->conf.dis_oc_delay_ms = 320; // ms (tolerates short spikes)

    bms->conf.dis_sc_limit = 50.0F;  // A (instant fault)
    bms->conf.dis_sc_delay_us = 400; // µs

    bms->conf.chg_oc_limit = 6.0f;  // Charge overcurrent
    bms->conf.chg_oc_delay_ms = 10; // ms (tolerates short spikes)
    // IGNORE TEMP LIMITS
    bms->conf.dis_ut_limit = -1000;
    bms->conf.dis_ot_limit = 1000;
    bms->conf.chg_ut_limit = -1000;
    bms->conf.chg_ot_limit = 1000;
    bms->conf.t_limit_hyst = 0;

    bms->conf.shunt_res_mOhm = BOARD_SHUNT_RESISTOR;

    bms->conf.cell_ov_delay_s = 4; // Think its more like seconds, not ms
    bms->conf.cell_uv_delay_s = 8;

    bms->conf.valid_min_voltage = 2.90F;
    bms->conf.valid_max_voltage = 4.30F;

    switch (type) {
        case CELL_TYPE_LFP:
            bms->conf.cell_ov_limit = 3.80F;
            bms->conf.cell_chg_voltage = 3.55F;
            bms->conf.cell_ov_reset = 3.40F;
            bms->conf.bal_cell_voltage_min = 3.30F;
            bms->conf.cell_uv_reset = 3.10F;
            bms->conf.cell_dis_voltage = 2.80F;
            bms->conf.cell_uv_limit = 2.50F; // most cells survive even 2.0V, but we should
                                             // keep some margin for further self-discharge
            bms->conf.ocv = ocv_lfp;
            break;
        case CELL_TYPE_NMC:
            bms->conf.cell_ov_limit = 4.25F;
            bms->conf.cell_chg_voltage = 4.20F;
            bms->conf.cell_ov_reset = 4.05F;
            bms->conf.bal_cell_voltage_min = 3.80F;
            bms->conf.cell_uv_reset = 3.50F;
            bms->conf.cell_dis_voltage = 3.20F;
            bms->conf.cell_uv_limit = 3.00F;
            bms->conf.ocv = ocv_nmc;
            break;
        case CELL_TYPE_NMC_HV:
            bms->conf.cell_ov_limit = 4.35F;
            bms->conf.cell_chg_voltage = 4.30F;
            bms->conf.cell_ov_reset = 4.15F;
            bms->conf.bal_cell_voltage_min = 3.80F;
            bms->conf.cell_uv_reset = 3.50F;
            bms->conf.cell_dis_voltage = 3.20F;
            bms->conf.cell_uv_limit = 3.00F;
            // ToDo: Use typical OCV curve for NMC_HV cells
            bms->conf.ocv = NULL;
            break;
        case CELL_TYPE_LTO:
            bms->conf.cell_ov_limit = 2.85F;
            bms->conf.cell_chg_voltage = 2.80F;
            bms->conf.cell_ov_reset = 2.70F;
            bms->conf.bal_cell_voltage_min = 2.50F;
            bms->conf.cell_uv_reset = 2.10F;
            bms->conf.cell_dis_voltage = 2.00F;
            bms->conf.cell_uv_limit = 1.90F;
            // ToDo: Use typical OCV curve for LTO cells
            bms->conf.ocv = NULL;
            break;
        case CELL_TYPE_GENERIC_LIPO:
            bms->conf.cell_ov_limit = 4.25F;        // absolute max
            bms->conf.cell_chg_voltage = 4.20F;     // normal full charge
            bms->conf.cell_ov_reset = 4.18F;        // hysteresis after OV
            bms->conf.bal_cell_voltage_min = 4.10F; // balance near top
            bms->conf.cell_dis_voltage = 3.23F;     // soft discharge stop
            bms->conf.cell_uv_limit = 3.20F;        // absolute minimum
            bms->conf.cell_uv_reset = 3.25F;        // margin for recovery
            bms->conf.ocv = ocv_lipo;

            // Limits when battery is considered invalid and should not be used
            bms->conf.valid_min_voltage = 2.90F;
            bms->conf.valid_max_voltage = 4.30F;

            break;
        case CELL_TYPE_CUSTOM:
            break;
    }
    LOG_INF("BMS config: shunt=%.3f mOhm, capacity=%.1f Ah, cell_type=%d",
            (double)bms->conf.shunt_res_mOhm, (double)bms->conf.nominal_capacity_Ah, type);

    LOG_DBG("Balancing: idle_delay=%d s, idle_current=%.2f A, diff=%.3f V, min=%.3f V",
            bms->conf.bal_idle_delay, (double)bms->conf.bal_idle_current,
            (double)bms->conf.bal_cell_voltage_diff, (double)bms->conf.bal_cell_voltage_min);

    LOG_DBG("Current limits: dis_oc=%.2f A/%d ms, chg_oc=%.2f A/%d ms, dis_sc=%.2f A/%d us",
            (double)bms->conf.dis_oc_limit, bms->conf.dis_oc_delay_ms,
            (double)bms->conf.chg_oc_limit, bms->conf.chg_oc_delay_ms,
            (double)bms->conf.dis_sc_limit, bms->conf.dis_sc_delay_us);

    LOG_DBG(
        "Voltage limits: chg=%.3f V, dis=%.3f V, ov=%.3f/%.3f V, uv=%.3f/%.3f V, delays=%d/%d s",
        (double)bms->conf.cell_chg_voltage, (double)bms->conf.cell_dis_voltage,
        (double)bms->conf.cell_ov_limit, (double)bms->conf.cell_ov_reset,
        (double)bms->conf.cell_uv_limit, (double)bms->conf.cell_uv_reset, bms->conf.cell_ov_delay_s,
        bms->conf.cell_uv_delay_s);

    LOG_DBG("Temperature limits: dis_ut=%.1f °C, dis_ot=%.1f °C, chg_ut=%.1f °C, chg_ot=%.1f °C, "
            "hyst=%.1f °C",
            (double)bms->conf.dis_ut_limit, (double)bms->conf.dis_ot_limit,
            (double)bms->conf.chg_ut_limit, (double)bms->conf.chg_ot_limit,
            (double)bms->conf.t_limit_hyst);
}

__weak int bms_state_machine(Bms *bms)
{
    bms_handle_errors(bms);

    switch (bms->status.state) {
            // TODO CLEANUP AND RESET BMS OBJECT

        case BMS_STATE_OFF:
            if (bms_startup_inhibit()) {
                return 0;
            }
            if (bms_dis_allowed(bms)) {
                /* Precharge before enabling discharge path (reduces inrush) */
                bms_dis_switch_safe(bms, true);
                bms->status.state = BMS_STATE_DIS;
                LOG_INF("OFF -> DIS (error flags: 0x%08x)", bms->status.error_flags);
            }
            else {
                bms_dis_switch_safe(bms, false);
            }
            if (bms_chg_allowed(bms)) {
                bms_chg_switch(bms, true);
                bms->status.state = BMS_STATE_CHG;
                LOG_INF("OFF -> CHG (error flags: 0x%08x)", bms->status.error_flags);
            }
            else {
                bms_chg_switch(bms, false);
            }
            break;

        case BMS_STATE_CHG:
            if (!bms_chg_allowed(bms)) {
                bms_chg_switch(bms, false);
                bms_dis_switch_safe(bms, false); // if on because of ideal diode control
                bms->status.state = BMS_STATE_OFF;
                LOG_INF("CHG -> OFF (error flags: 0x%08x)", bms->status.error_flags);
            }
            else if (bms_dis_allowed(bms)) {
                /* Precharge before enabling discharge */
                bms_dis_switch_safe(bms, true);
                bms->status.state = BMS_STATE_NORMAL;
                LOG_INF("CHG -> NORMAL (error flags: 0x%08x)", bms->status.error_flags);
            }

            break;
        case BMS_STATE_DIS:
            /* If a CHG overcurrent is active, start/extend hold-off timer */
            if (bms->status.error_flags & (1U << BMS_ERR_CHG_OVERCURRENT)) {
                chg_oc_resume_at_ms = k_uptime_get() + BMS_CHG_OC_HOLDOFF_MS;
            }
            if (!bms_dis_allowed(bms)) {
                bms_dis_switch_safe(bms, false);
                bms_chg_switch(bms, false); // if on because of ideal diode control
                bms->status.state = BMS_STATE_OFF;
                LOG_INF("DIS -> OFF (error flags: 0x%08x)", bms->status.error_flags);
            }
            else if (bms_chg_allowed(bms)) {
                /* Only re-enable charge after hold-off expired */
                if (k_uptime_get() >= chg_oc_resume_at_ms) {
                    bms_chg_switch(bms, true);
                    bms->status.state = BMS_STATE_NORMAL;
                    LOG_INF("DIS -> NORMAL (error flags: 0x%08x)", bms->status.error_flags);
                }
            }

            break;
        case BMS_STATE_NORMAL:
            if (!bms_dis_allowed(bms)) {
                bms_dis_switch_safe(bms, false);
                bms->status.state = BMS_STATE_CHG;
                LOG_INF("NORMAL -> CHG (error flags: 0x%08x)", bms->status.error_flags);
            }
            else if (!bms_chg_allowed(bms)) {
                bms_chg_switch(bms, false);
                bms->status.state = BMS_STATE_DIS;
                LOG_INF("NORMAL -> DIS (error flags: 0x%08x)", bms->status.error_flags);
            }
            else {
                /* Both directions allowed: ensure both FETs are on */
                bms_dis_switch_safe(bms, true);
                bms_chg_switch(bms, true);
            }

            break;
    }
    return 0;
}
bool bms_chg_error(uint32_t error_flags)
{
    return (error_flags & (1U << BMS_ERR_CELL_OVERVOLTAGE))
           || (error_flags & (1U << BMS_ERR_CHG_OVERCURRENT))
           || (error_flags & (1U << BMS_ERR_OPEN_WIRE))
           || (error_flags & (1U << BMS_ERR_CHG_UNDERTEMP))
           || (error_flags & (1U << BMS_ERR_CHG_OVERTEMP))
           || (error_flags & (1U << BMS_ERR_INT_OVERTEMP))
           || (error_flags & (1U << BMS_ERR_CELL_FAILURE))
           || (error_flags & (1U << BMS_ERR_CHG_OFF)) //|| (error_flags & (1U << BMS_ERR_NO_ALERT))
           //|| (error_flags & (1U << BMS_ERR_GIBBERISH_REGISTERS))
           || (error_flags & (1U << BMS_ERR_MEAS_VOLTAGE_TOO_LOW))
           || (error_flags & (1U << BMS_ERR_MEAS_VOLTAGE_TOO_HIGH));
}

bool bms_dis_error(uint32_t error_flags)
{
    return (error_flags & (1U << BMS_ERR_CELL_UNDERVOLTAGE))
           || (error_flags & (1U << BMS_ERR_SHORT_CIRCUIT))
           || (error_flags & (1U << BMS_ERR_DIS_OVERCURRENT))
           || (error_flags & (1U << BMS_ERR_OPEN_WIRE))
           || (error_flags & (1U << BMS_ERR_DIS_UNDERTEMP))
           || (error_flags & (1U << BMS_ERR_DIS_OVERTEMP))
           || (error_flags & (1U << BMS_ERR_INT_OVERTEMP))
           || (error_flags & (1U << BMS_ERR_CELL_FAILURE))
           || (error_flags & (1U << BMS_ERR_DIS_OFF)) //|| (error_flags & (1U << BMS_ERR_NO_ALERT))
           //|| (error_flags & (1U << BMS_ERR_GIBBERISH_REGISTERS))
           || (error_flags & (1U << BMS_ERR_MEAS_VOLTAGE_TOO_LOW))
           || (error_flags & (1U << BMS_ERR_MEAS_VOLTAGE_TOO_HIGH));
}

bool bms_chg_allowed(Bms *bms)
{
    return !bms_chg_error(bms->status.error_flags & ~(1U << BMS_ERR_CHG_OFF)) && !bms->status.full
           && bms->status.chg_enable;
}

bool bms_dis_allowed(Bms *bms)
{
    return !bms_dis_error(bms->status.error_flags & ~(1U << BMS_ERR_DIS_OFF)) && !bms->status.empty
           && bms->status.dis_enable;
}

bool bms_balancing_allowed(Bms *bms)
{
    int idle_sec = uptime() - bms->status.no_idle_timestamp;
    float voltage_diff = bms->status.cell_voltage_max - bms->status.cell_voltage_min;

    return idle_sec >= bms->conf.bal_idle_delay
           && bms->status.cell_voltage_max > bms->conf.bal_cell_voltage_min
           && voltage_diff > bms->conf.bal_cell_voltage_diff;
}

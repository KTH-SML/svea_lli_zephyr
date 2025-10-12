/*
 * Copyright (c) The Libre Solar Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "board.h"
#include "helper.h"

#include "interface.h"
#include "registers.h"

#include <string.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/crc.h>

LOG_MODULE_REGISTER(bq769x0_if, CONFIG_LOG_DEFAULT_LEVEL);

#define BQ769X0_NODE DT_INST(0, ti_bq769x0)

int adc_gain;   // factory-calibrated, read out from chip (uV/LSB)
int adc_offset; // factory-calibrated, read out from chip (mV)

// indicates if a new current reading or an error is available from BMS IC
static bool alert_interrupt_flag;
static time_t alert_interrupt_timestamp;
static bool alert_supported;

#ifndef UNIT_TEST

static const struct device *i2c_dev = DEVICE_DT_GET(DT_PARENT(BQ769X0_NODE));
static int i2c_address = DT_REG_ADDR(BQ769X0_NODE);
static bool crc_enabled; // determined automatically

#if DT_NODE_HAS_PROP(BQ769X0_NODE, alert_gpios)
static const struct gpio_dt_spec alert = GPIO_DT_SPEC_GET(BQ769X0_NODE, alert_gpios);
static struct gpio_callback alert_cb;
#endif

/* Optional precharge control GPIO (e.g., BQ76200 PCHG) */
static const struct gpio_dt_spec pchg = GPIO_DT_SPEC_GET_OR(BQ769X0_NODE, pchg_gpios, {0});
static bool pchg_is_output;

// The bq769x0 drives the ALERT pin high if the SYS_STAT register contains
// a new value (either new CC reading or an error)
static void bq769x0_alert_isr(const struct device *port, struct gpio_callback *cb,
                              gpio_port_pins_t pins)
{
    alert_interrupt_timestamp = uptime();
    alert_interrupt_flag = true;
}
#else
static void bq769x0_alert_isr(const struct device *port, struct gpio_callback *cb,
                              gpio_port_pins_t pins)
{
    ARG_UNUSED(port);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);
}
#endif

void bq769x0_write_byte(uint8_t reg_addr, uint8_t data)
{
    uint8_t buf[4] = { i2c_address << 1, // slave address incl. R/W bit required for CRC calculation
                       reg_addr, data };

    if (crc_enabled) {
        buf[3] = crc8_ccitt(0, buf, 3);
        i2c_write(i2c_dev, buf + 1, 3, i2c_address);
    }
    else {
        i2c_write(i2c_dev, buf + 1, 2, i2c_address);
    }
}

uint8_t bq769x0_read_byte(uint8_t reg_addr)
{
    uint8_t buf[3];

    i2c_write(i2c_dev, &reg_addr, 1, i2c_address);

    if (crc_enabled) {
        // CRC is calculated over the slave address (incl. R/W bit) and data
        buf[0] = (i2c_address << 1) | 1U;
        do {
            i2c_read(i2c_dev, buf + 1, 2, i2c_address);
        } while (crc8_ccitt(0, buf, 2) != buf[2]);

        return buf[1];
    }
    else {
        i2c_read(i2c_dev, buf, 1, i2c_address);
        return buf[0];
    }
}

int32_t bq769x0_read_word(uint8_t reg_addr)
{
    uint8_t buf[5];

    // write starting register
    i2c_write(i2c_dev, &reg_addr, 1, i2c_address);

    if (crc_enabled) {
        // CRC is calculated over the slave address (incl. R/W bit) and data
        buf[0] = (i2c_address << 1) | 1U;
        i2c_read(i2c_dev, buf + 1, 4, i2c_address);

        if (crc8_ccitt(0, buf, 2) != buf[2]) {
            return -1;
        }

        // CRC of subsequent bytes only considering data
        if (crc8_ccitt(0, buf + 3, 1) != buf[4]) {
            return -1;
        }

        return buf[1] << 8 | buf[3];
    }
    else {
        i2c_read(i2c_dev, buf, 2, i2c_address);
        return buf[0] << 8 | buf[1];
    }
}

// automatically find out address and CRC setting
static int determine_address_and_crc(void)
{
    /* Try the two common 7-bit addresses without CRC first.
     * Avoid CRC probing here to prevent potential infinite loops when the
     * device does not respond and crc8_ccitt never matches. */

    i2c_address = 0x08;
    crc_enabled = true;
    bq769x0_write_byte(BQ769X0_CC_CFG, 0x19);
    if (bq769x0_read_byte(BQ769X0_CC_CFG) == 0x19) {
        return 0;
    }

    i2c_address = 0x18;
    crc_enabled = true;
    bq769x0_write_byte(BQ769X0_CC_CFG, 0x19);
    if (bq769x0_read_byte(BQ769X0_CC_CFG) == 0x19) {
        return 0;
    }

    return -EIO;
}

int bq769x0_init()
{
#if DT_NODE_HAS_PROP(BQ769X0_NODE, alert_gpios)
    alert_supported = true;
    if (!device_is_ready(alert.port)) {
        return -ENODEV;
    }

    gpio_pin_configure_dt(&alert, GPIO_INPUT);
    gpio_init_callback(&alert_cb, bq769x0_alert_isr, BIT(alert.pin));
    gpio_add_callback(alert.port, &alert_cb);
#else
    alert_supported = false;
    LOG_WRN("BQ769x0 ALERT GPIO not defined; falling back to polling");
#endif

    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C device not ready");
        return -ENODEV;
    }

    alert_interrupt_flag = true; // init with true to check and clear errors at start-up

    /* Configure precharge pin if available */
    if (pchg.port) {
        int r = gpio_pin_configure_dt(&pchg, GPIO_OUTPUT_INACTIVE);
        if (r == 0) {
            pchg_is_output = true;
        } else {
            LOG_WRN("PCHG gpio configure failed: %d", r);
        }
    }

    int err = determine_address_and_crc();
    if (!err) {
        LOG_INF("bq769x0 detected at 0x%02x (CRC %s)", i2c_address,
                crc_enabled ? "enabled" : "disabled");
        // initial settings for bq769x0
        bq769x0_write_byte(BQ769X0_SYS_CTRL1, 0b00011000); // switch external thermistor and ADC on
        bq769x0_write_byte(BQ769X0_SYS_CTRL2, 0b01000000); // switch CC_EN on

        // get ADC offset and gain
        adc_offset = (signed int)bq769x0_read_byte(BQ769X0_ADCOFFSET); // 2's complement
        adc_gain = 365
                   + (((bq769x0_read_byte(BQ769X0_ADCGAIN1) & 0b00001100) << 1)
                      | ((bq769x0_read_byte(BQ769X0_ADCGAIN2) & 0b11100000) >> 5)); // uV/LSB
    }
    else {
        LOG_ERR("BMS communication error");
        return err;
    }

    return 0;
}

#endif // UNIT_TEST

bool bq769x0_alert_flag()
{
    if (!alert_supported) {
        return true;
    }
    return alert_interrupt_flag;
}

void bq769x0_alert_flag_reset()
{
    if (!alert_supported) {
        return;
    }
    alert_interrupt_flag = false;
}

time_t bq769x0_alert_timestamp()
{
    return alert_interrupt_timestamp;
}

int bq769x0_pchg_set(bool enable)
{
    if (!pchg.port) {
        return -ENODEV;
    }
    if (!pchg_is_output) {
        int r = gpio_pin_configure_dt(&pchg, GPIO_OUTPUT_INACTIVE);
        if (r) {
            return r;
        }
        pchg_is_output = true;
    }
    return gpio_pin_set_dt(&pchg, enable ? 1 : 0);
}

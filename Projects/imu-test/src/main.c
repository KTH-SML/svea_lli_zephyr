/* Minimal I2C IMU test: read WHO_AM_I from ICM42670 on I2C2 @ 0x68 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

LOG_MODULE_REGISTER(imu_test, LOG_LEVEL_INF);

#define IMU_NODE DT_ALIAS(imu)

#if !DT_NODE_HAS_STATUS(IMU_NODE, okay)
#error "No 'imu' alias found or it is disabled in the devicetree"
#endif

static const struct i2c_dt_spec imu = I2C_DT_SPEC_GET(IMU_NODE);

static int imu_read_reg(uint8_t reg, uint8_t *val)
{
    return i2c_burst_read_dt(&imu, reg, val, 1);
}

static int imu_write_reg(uint8_t reg, uint8_t val)
{
    return i2c_burst_write_dt(&imu, reg, &val, 1);
}

int main(void)
{
    k_sleep(K_MSEC(1000));
    printf("IMU I2C test starting\n");
    k_sleep(K_MSEC(100));
    if (!device_is_ready(imu.bus)) {
        LOG_ERR("I2C bus not ready");
        return 0;
    }

    /* Try WHO_AM_I a few times with short gaps */
    uint8_t who = 0;
    int rc = -1;
    for (int i = 0; i < 5; i++) {
        rc = imu_read_reg(0x75, &who);
        if (rc == 0) {
            LOG_INF("WHO_AM_I=0x%02x (try %d)", who, i + 1);
            break;
        }
        k_msleep(1);
    }

    if (rc) {
        LOG_ERR("WHO_AM_I read failed: %d", rc);
        return 0;
    }

    /* Optional: soft reset sequence to observe behavior */
    /* Select BLK_SEL_W/R = 0 (safe no-op) */
    (void)imu_write_reg(0x7C, 0x00); /* BLK_SEL_W */
    (void)imu_write_reg(0x79, 0x00); /* BLK_SEL_R */

    /* Trigger soft reset at SIGNAL_PATH_RESET (0x02@MREG -> 0x10002)
     * UI register access for reset is via 0x02 through banked path; for a
     * quick bus test we skip reset to avoid dropping the bus mid-flow.
     */

    /* Read a few UI registers to validate ongoing comms */
    uint8_t intf_cfg1 = 0;
    rc = imu_read_reg(0x36, &intf_cfg1); /* INTF_CONFIG1 */
    if (rc == 0) {
        LOG_INF("INTF_CONFIG1=0x%02x", intf_cfg1);
    } else {
        LOG_WRN("INTF_CONFIG1 read failed: %d", rc);
    }

    while (1) {
        /* Periodically read INT_STATUS (0x3A) as a liveness check */
        uint8_t int_status = 0;
        rc = imu_read_reg(0x3A, &int_status);
        if (rc == 0) {
            LOG_INF("INT_STATUS=0x%02x", int_status);
        } else {
            LOG_WRN("INT_STATUS read failed: %d", rc);
        }
        k_sleep(K_MSEC(500));
    }
}

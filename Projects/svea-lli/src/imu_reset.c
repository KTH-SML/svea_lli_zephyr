// Strap MB4 CS high (force I2C mode), clear I2C2 bus if needed, then IMU soft-reset before driver probe
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(imu_reset, LOG_LEVEL_INF);

// Board wiring: I2C2 on PB10 (SCL), PB11 (SDA)
#define GPIOB_NODE DT_NODELABEL(gpiob)
#define GPIOD_NODE DT_NODELABEL(gpiod)
#define IMU_NODE DT_ALIAS(imu)

// Attempt to free a stuck I2C bus by toggling SCL up to 9 pulses
static int i2c2_bus_clear(const struct device *gpio_port)
{
    if (!device_is_ready(gpio_port)) {
        return -ENODEV;
    }

    // Configure SCL (PB10) and SDA (PB11) as open-drain GPIOs
    (void)gpio_pin_configure(gpio_port, 10, GPIO_OPEN_DRAIN | GPIO_INPUT);
    (void)gpio_pin_configure(gpio_port, 11, GPIO_OPEN_DRAIN | GPIO_INPUT);

    // If SDA is low, clock SCL until it releases (max 9 pulses)
    for (int i = 0; i < 9; i++) {
        int sda = gpio_pin_get(gpio_port, 11);
        if (sda > 0) {
            break;
        }
        // Drive SCL low then release high
        (void)gpio_pin_configure(gpio_port, 10, GPIO_OPEN_DRAIN | GPIO_OUTPUT);
        (void)gpio_pin_set(gpio_port, 10, 0);
        k_busy_wait(5);
        (void)gpio_pin_set(gpio_port, 10, 1);
        (void)gpio_pin_configure(gpio_port, 10, GPIO_OPEN_DRAIN | GPIO_INPUT);
        k_busy_wait(5);
    }

    // Generate a STOP: SDA high while SCL high
    (void)gpio_pin_configure(gpio_port, 10, GPIO_OPEN_DRAIN | GPIO_INPUT);
    (void)gpio_pin_configure(gpio_port, 11, GPIO_OPEN_DRAIN | GPIO_OUTPUT);
    (void)gpio_pin_set(gpio_port, 11, 1);
    k_busy_wait(5);
    (void)gpio_pin_configure(gpio_port, 11, GPIO_OPEN_DRAIN | GPIO_INPUT);

    return 0;
}

static int i2c2_bus_clear_pre_kernel(const struct device *unused)
{
    ARG_UNUSED(unused);
    // 1) Ensure MB4 CS (PD15) is high so ICM42670 latches I2C mode on any warm reset
    const struct device *port_d = DEVICE_DT_GET(GPIOD_NODE);
    if (device_is_ready(port_d)) {
        (void)gpio_pin_configure(port_d, 15, GPIO_OUTPUT);
        (void)gpio_pin_set(port_d, 15, 1);
    }

    // 2) Optionally clear the I2C2 bus if SDA is stuck low
    const struct device *gpio_port = DEVICE_DT_GET(GPIOB_NODE);
    if (!device_is_ready(gpio_port)) {
        return 0;
    }
    (void)i2c2_bus_clear(gpio_port);
    return 0;
}

// Run as early as possible to free the bus before I2C2 config
SYS_INIT(i2c2_bus_clear_pre_kernel, PRE_KERNEL_1, 0);

static int imu_soft_reset_pre_driver(const struct device *unused)
{
    ARG_UNUSED(unused);

#if DT_NODE_HAS_STATUS(IMU_NODE, okay)
    const struct i2c_dt_spec imu = I2C_DT_SPEC_GET(IMU_NODE);
    if (!device_is_ready(imu.bus)) {
        LOG_DBG("IMU I2C bus not ready for pre-reset");
        return 0;
    }
    // If the controller implements bus recovery, try it
    (void)i2c_recover_bus(imu.bus);
    uint8_t v;
    // Force bank 0
    v = 0x00; (void)i2c_burst_write_dt(&imu, 0x7C, &v, 1);
    v = 0x00; (void)i2c_burst_write_dt(&imu, 0x79, &v, 1);
    // Soft reset
    v = 0x10; (void)i2c_burst_write_dt(&imu, 0x02, &v, 1);
    // Allow device to reboot before driver probe
    k_msleep(50);
    LOG_DBG("IMU soft reset issued before driver init");
#endif
    return 0;
}

// Run before sensor drivers (which usually init at CONFIG_SENSOR_INIT_PRIORITY ~90)
SYS_INIT(imu_soft_reset_pre_driver, POST_KERNEL, 60);

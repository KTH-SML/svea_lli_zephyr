/*
 * SPDX-License-Identifier: Apache-2.0
 * Updated 2025-06-30 for Zephyr 4.x capture API
 */
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#define GPIO_PORT DT_NODELABEL(gpioe)
#define GPIO_PIN 10

static const struct device *gpio_dev;
static struct gpio_callback gpio_cb_data;

static volatile uint32_t last_rising = 0;
static volatile uint32_t period = 0;
static volatile uint32_t pulse = 0;

void pwm_gpio_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    uint32_t now = k_cycle_get_32();
    int val = gpio_pin_get(dev, GPIO_PIN);

    if (val) {
        // Rising edge
        period = now - last_rising;
        last_rising = now;
    } else {
        // Falling edge
        pulse = now - last_rising;
    }
}

void main(void) {
    gpio_dev = DEVICE_DT_GET(GPIO_PORT);
    if (!device_is_ready(gpio_dev)) {
        printf("GPIO device not ready!\n");
        return;
    }

    gpio_pin_configure(gpio_dev, GPIO_PIN, GPIO_INPUT | GPIO_PULL_DOWN);
    gpio_pin_interrupt_configure(gpio_dev, GPIO_PIN, GPIO_INT_EDGE_BOTH);

    gpio_init_callback(&gpio_cb_data, pwm_gpio_isr, BIT(GPIO_PIN));
    gpio_add_callback(gpio_dev, &gpio_cb_data);

    printf("PWM edge capture on PE10 (pin %d)\n", GPIO_PIN);

    while (1) {
        uint32_t local_period = period;
        uint32_t local_pulse = pulse;
        uint32_t freq = 0, duty = 0;
        uint32_t us_period = 0, us_pulse = 0;

        if (local_period > 0) {
            freq = sys_clock_hw_cycles_per_sec() / local_period;
            us_period = (local_period * 1000000U) / sys_clock_hw_cycles_per_sec();
            us_pulse = (local_pulse * 1000000U) / sys_clock_hw_cycles_per_sec();
            duty = (local_pulse * 1000U) / local_period;
        }

        printf("period: %u cyc (%u us), pulse: %u cyc (%u us), freq: %u Hz, duty: %u/1000\n",
               local_period, us_period, local_pulse, us_pulse, freq, duty);
        k_msleep(100);
    }
}

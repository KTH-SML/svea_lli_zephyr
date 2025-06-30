/*
 * Single-channel PWM capture demo — Zephyr 4.x API
 * Capture pin: PE11 (TIM1_CH2)
 */

#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define PWM_NODE DT_NODELABEL(pwm_in)
static const struct device *const pwm_dev = DEVICE_DT_GET(PWM_NODE);
#define PWM_CHANNEL 2 // TIM1_CH2 (PE11)

static void cb(const struct device *dev, uint32_t chan,
               uint32_t per_cyc, uint32_t pul_cyc,
               int status, void *ud) {
    static uint32_t last_duty = 0;

    if (status) {
        printk("capture error %d\n", status);
        return;
    }

    uint64_t period, pulse;
    pwm_cycles_to_usec(dev, chan, per_cyc, &period);
    pwm_cycles_to_usec(dev, chan, pul_cyc, &pulse);
    uint32_t duty = (period) ? (pulse * 1000) / period : 0;

    // Only print if duty is in expected range and changed
    if (duty != last_duty && duty >= 100 && duty <= 200) {
        last_duty = duty;
        printf("duty = %u/1000\n", duty);
    }
}

int main(void) {
    printk("TIM1 CH2 PWM capture example — input on PE11\n");

    if (!device_is_ready(pwm_dev)) {
        printk("device not ready\n");
        return 0;
    }

    int ret = pwm_configure_capture(pwm_dev, PWM_CHANNEL,
                                    PWM_CAPTURE_TYPE_BOTH | PWM_CAPTURE_MODE_CONTINUOUS,
                                    cb, NULL);
    if (ret) {
        printk("config failed (%d)\n", ret);
        return 0;
    }

    ret = pwm_enable_capture(pwm_dev, PWM_CHANNEL);
    if (ret) {
        printk("enable failed (%d)\n", ret);
        return 0;
    }

    while (1) {
        k_sleep(K_FOREVER); // Nothing to do, all work is in the callback
    }
}

/*
 * Single-channel PWM capture demo — Zephyr 4.x API
 * Capture pin: PA0 (TIM2_CH1)
 */

#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define PWM_NODE DT_NODELABEL(pwm_in) /* sub-node defined above */
static const struct device *const pwm_dev = DEVICE_DT_GET(PWM_NODE);

static volatile uint64_t period_us, pulse_us;
static volatile uint32_t duty_permille;
static K_SEM_DEFINE(done, 0, 1);

static void cb(const struct device *dev, uint32_t chan,
               uint32_t per_cyc, uint32_t pul_cyc,
               int status, void *ud) {
    if (status) {
        printk("capture error %d\n", status);
        return;
    }

    pwm_cycles_to_usec(dev, chan, per_cyc, &period_us);
    pwm_cycles_to_usec(dev, chan, pul_cyc, &pulse_us);
    duty_permille = (period_us) ? (pulse_us * 1000) / period_us : 0;

    k_sem_give(&done); /* wake the print loop */
}

int main(void) {
    printk("TIM2 CH1 PWM capture example — input on PA0\n");

    if (!device_is_ready(pwm_dev)) {
        printk("device not ready\n");
        return 0;
    }

    /* configure the channel once … */
    int ret = pwm_configure_capture(pwm_dev, 2, /* channel 2 */
                                    PWM_CAPTURE_TYPE_BOTH | PWM_CAPTURE_MODE_CONTINUOUS,
                                    cb, NULL);
    if (ret) {
        printk("config failed (%d)\n", ret);
        return 0;
    }

    /* … then enable it */
    ret = pwm_enable_capture(pwm_dev, 2);
    if (ret) {
        printk("enable failed (%d)\n", ret);
        return 0;
    }

    while (1) {
        k_sem_take(&done, K_FOREVER);
        printk("pulse = %llu us  period = %llu us  duty = %u/1000\n",
               pulse_us, period_us, duty_permille);
    }
}

#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include "servo_sweeper.h"

static const struct pwm_dt_spec servo = PWM_DT_SPEC_GET(DT_NODELABEL(servo));
static const uint32_t min_pulse = DT_PROP(DT_NODELABEL(servo), min_pulse);
static const uint32_t max_pulse = DT_PROP(DT_NODELABEL(servo), max_pulse);

#define STEP PWM_USEC(20)

enum direction {
    DOWN,
    UP,
};

void servo_sweeper_thread(void *arg1, void *arg2, void *arg3) {
    uint32_t pulse_width = min_pulse;
    enum direction dir = UP;
    int ret;

    printk("Servomotor control\n");

    if (!pwm_is_ready_dt(&servo)) {
        printk("Error: PWM device %s is not ready\n", servo.dev->name);
        return;
    }

    while (1) {
        ret = pwm_set_pulse_dt(&servo, pulse_width);
        if (ret < 0) {
            printk("Error %d: failed to set pulse width\n", ret);
            return;
        }

        if (dir == DOWN) {
            if (pulse_width <= min_pulse) {
                dir = UP;
                pulse_width = min_pulse;
            } else {
                pulse_width -= STEP;
            }
        } else {
            pulse_width += STEP;

            if (pulse_width >= max_pulse) {
                dir = DOWN;
                pulse_width = max_pulse;
            }
        }

        k_sleep(K_MSEC(10));
    }
}

K_THREAD_STACK_DEFINE(servo_sweeper_stack, 1024);
struct k_thread servo_sweeper_thread_data;
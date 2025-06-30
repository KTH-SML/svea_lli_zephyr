#include "pwm_publisher.h"
#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#ifndef RCCHECK
#define RCCHECK(fn)                                                  \
    do {                                                             \
        rcl_ret_t _rc = (fn);                                        \
        if (_rc != RCL_RET_OK) {                                     \
            printf("rcl error %d at line %d\n", (int)_rc, __LINE__); \
            goto cleanup;                                            \
        }                                                            \
    } while (0)
#endif

K_THREAD_STACK_DEFINE(steering_pwm_stack, STEERING_PWM_THREAD_STACK_SIZE);
struct k_thread steering_pwm_thread_data;

static rcl_publisher_t steering_pwm_pub;
static std_msgs__msg__Int32 duty_msg;

#define PWM_NODE DT_NODELABEL(pwm_in)
#define PWM_CHANNEL 2

static const struct device *const pwm_dev = DEVICE_DT_GET(PWM_NODE);

static void pwm_cb(const struct device *dev, uint32_t chan,
                   uint32_t per_cyc, uint32_t pul_cyc,
                   int status, void *ud) {
    static int32_t last_duty = -1;
    if (status) {
        printf("PWM callback: capture error %d\n", status);
        return;
    }

    uint64_t period = 0, pulse = 0;
    pwm_cycles_to_usec(dev, chan, per_cyc, &period);
    pwm_cycles_to_usec(dev, chan, pul_cyc, &pulse);
    int32_t duty = (period) ? (pulse * 1000) / period : -1;

    if (duty != last_duty && duty >= 100 && duty <= 200) {
        last_duty = duty;
        printf("PWM callback: duty changed and in range: %d, publishing to ROS\n", duty);
        duty_msg.data = duty;
        rcl_ret_t rc = rcl_publish(&steering_pwm_pub, &duty_msg, NULL);
        if (rc != RCL_RET_OK) {
            printf("PWM publish error: %d\n", rc);
        } else {
            printf("PWM callback: published duty %d\n", duty);
        }
    }
}

static void steering_pwm_thread(void *arg1, void *arg2, void *arg3) {
    printf("PWM thread: entered\n");

    if (!device_is_ready(pwm_dev)) {
        printk("PWM device not ready!\n");
        goto cleanup;
    }

    printf("Configuring PWM capture...\n");
    int ret = pwm_configure_capture(pwm_dev, PWM_CHANNEL,
                                    PWM_CAPTURE_TYPE_BOTH | PWM_CAPTURE_MODE_CONTINUOUS,
                                    pwm_cb, NULL);
    if (ret) {
        printk("PWM capture config failed (%d)\n", ret);
        goto cleanup;
    }
    printf("Enabling PWM capture...\n");
    ret = pwm_enable_capture(pwm_dev, PWM_CHANNEL);
    if (ret) {
        printk("PWM capture enable failed (%d)\n", ret);
        goto cleanup;
    }
    printf("PWM capture enabled, entering main loop.\n");

    while (1) {
        k_sleep(K_FOREVER); // All work is done in the callback
    }

cleanup:
    printk("PWM publisher thread exiting due to error\n");
    while (1) {
        k_sleep(K_MSEC(1000));
    }
}

void steering_pwm_publisher_init(rcl_node_t *node) {
    printf("Initializing steering PWM publisher...\n");
    rcl_ret_t rc = rclc_publisher_init_best_effort(
        &steering_pwm_pub, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        STEERING_PWM_TOPIC);
    if (rc != RCL_RET_OK) {
        printf("PWM publisher init failed: %d\n", rc);
        return;
    }
    printf("PWM publisher initialized\n");
    k_thread_create(&steering_pwm_thread_data, steering_pwm_stack, STEERING_PWM_THREAD_STACK_SIZE,
                    steering_pwm_thread, NULL, NULL, NULL,
                    STEERING_PWM_THREAD_PRIORITY, 0, K_NO_WAIT);
    printf("PWM thread started\n");
}
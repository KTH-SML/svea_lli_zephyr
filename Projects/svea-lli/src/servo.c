#include "servo.h"
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

LOG_MODULE_REGISTER(servo, LOG_LEVEL_INF);

struct servo_data {
    const struct device *dev;
    uint32_t channel;
    uint32_t period_cycles;
    atomic_t target;
    struct k_work work;
    int id;
};

static struct servo_data srv[SERVO_COUNT];

uint32_t us_to_cycles(uint32_t us, uint32_t period_cycles) {
    // Convert microseconds to PWM cycles
    // period_cycles corresponds to 20ms period
    return (us * period_cycles) / 20000;
}

static void apply_servo(struct k_work *work) {
    struct servo_data *servo = CONTAINER_OF(work, struct servo_data, work);
    uint32_t us = atomic_get(&servo->target);
    uint32_t cycles = us_to_cycles(us, servo->period_cycles);

    // Use the correct Zephyr 4.1 API with flags parameter
    int ret = pwm_set_cycles(servo->dev, servo->channel, servo->period_cycles, cycles, 0);
    if (ret < 0) {
        LOG_ERR("Failed to set PWM for servo %d: %d", servo->id, ret);
    }
}

void servo_request(int id, uint32_t us) {
    if (id >= SERVO_COUNT || id < 0) {
        LOG_ERR("Invalid servo ID: %d", id);
        return;
    }

    // Bound check the PWM value to prevent hardware damage
    if (us < 1000) {
        LOG_WRN("Servo %d: Limiting low pulse width from %u to 1000us", id, us);
        us = 1000;
    } else if (us > 2100) {
        LOG_WRN("Servo %d: Limiting high pulse width from %u to 2100us", id, us);
        us = 2100;
    }

    if (us != atomic_get(&srv[id].target)) {
        atomic_set(&srv[id].target, us);
        if (k_work_submit(&srv[id].work) < 0) {
            LOG_ERR("Failed to submit work for servo %d", id);
        }
    }
}

void servo_init(void) {
    LOG_INF("Initializing servo control");

    // Servo 0: Steering
#if DT_NODE_EXISTS(DT_ALIAS(servo0))
    srv[0].dev = DEVICE_DT_GET(DT_PWMS_CTLR(DT_ALIAS(servo0)));
    srv[0].channel = DT_PWMS_CHANNEL(DT_ALIAS(servo0));
    srv[0].period_cycles = DT_PWMS_PERIOD(DT_ALIAS(servo0));
    srv[0].id = 0;

    if (device_is_ready(srv[0].dev)) {
        atomic_set(&srv[0].target, 1500);
        k_work_init(&srv[0].work, apply_servo);
        servo_request(0, 1500);
        LOG_INF("Servo 0 (steering) initialized on channel %d", srv[0].channel);
    } else {
        LOG_ERR("PWM device for servo 0 not ready");
    }
#endif

    // Servo 1: Gear
#if DT_NODE_EXISTS(DT_ALIAS(servo1))
    srv[1].dev = DEVICE_DT_GET(DT_PWMS_CTLR(DT_ALIAS(servo1)));
    srv[1].channel = DT_PWMS_CHANNEL(DT_ALIAS(servo1));
    srv[1].period_cycles = DT_PWMS_PERIOD(DT_ALIAS(servo1));
    srv[1].id = 1;

    if (device_is_ready(srv[1].dev)) {
        atomic_set(&srv[1].target, 1500);
        k_work_init(&srv[1].work, apply_servo);
        servo_request(1, 1500);
        LOG_INF("Servo 1 (gear) initialized on channel %d", srv[1].channel);
    } else {
        LOG_ERR("PWM device for servo 1 not ready");
    }
#endif

    // Servo 2: Throttle
#if DT_NODE_EXISTS(DT_ALIAS(servo2))
    srv[2].dev = DEVICE_DT_GET(DT_PWMS_CTLR(DT_ALIAS(servo2)));
    srv[2].channel = DT_PWMS_CHANNEL(DT_ALIAS(servo2));
    srv[2].period_cycles = DT_PWMS_PERIOD(DT_ALIAS(servo2));
    srv[2].id = 2;

    if (device_is_ready(srv[2].dev)) {
        atomic_set(&srv[2].target, 1500);
        k_work_init(&srv[2].work, apply_servo);
        servo_request(2, 1500);
        LOG_INF("Servo 2 (throttle) initialized on channel %d", srv[2].channel);
    } else {
        LOG_ERR("PWM device for servo 2 not ready");
    }
#endif

    // Servo 3: Diff
#if DT_NODE_EXISTS(DT_ALIAS(servo3))
    srv[3].dev = DEVICE_DT_GET(DT_PWMS_CTLR(DT_ALIAS(servo3)));
    srv[3].channel = DT_PWMS_CHANNEL(DT_ALIAS(servo3));
    srv[3].period_cycles = DT_PWMS_PERIOD(DT_ALIAS(servo3));
    srv[3].id = 3;

    if (device_is_ready(srv[3].dev)) {
        atomic_set(&srv[3].target, 1500);
        k_work_init(&srv[3].work, apply_servo);
        servo_request(3, 1500);
        LOG_INF("Servo 3 (diff) initialized on channel %d", srv[3].channel);
    } else {
        LOG_ERR("PWM device for servo 3 not ready");
    }
#endif
}
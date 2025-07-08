#include "control.h"
#include "rc_input.h"
#include <stdlib.h>             // For abs()
#include <zephyr/drivers/pwm.h> // Needed for pwm_cycles_to_usec
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(control, LOG_LEVEL_INF);

#define ROS_CMD_TIMEOUT_MS 500
#define RC_WATCHDOG_TIMEOUT_MS 150 // Increased from 120 to give more leeway

bool servos_initialized = false; // Flag to indicate if servos are initialized
bool remote_connected = false;   // Flag to indicate if remote is connected
// Servo array
servo_t servos[SERVO_COUNT] = {
#if DT_NODE_EXISTS(DT_NODELABEL(steeringservo))
    [SERVO_STEERING] = {.spec = PWM_DT_SPEC_GET(DT_NODELABEL(steeringservo))},
#endif
#if DT_NODE_EXISTS(DT_NODELABEL(gearservo))
    [SERVO_GEAR] = {.spec = PWM_DT_SPEC_GET(DT_NODELABEL(gearservo))},
#endif
#if DT_NODE_EXISTS(DT_NODELABEL(throttleesc))
    [SERVO_THROTTLE] = {.spec = PWM_DT_SPEC_GET(DT_NODELABEL(throttleesc))},
#endif
#if DT_NODE_EXISTS(DT_NODELABEL(diffservo))
    [SERVO_DIFF] = {.spec = PWM_DT_SPEC_GET(DT_NODELABEL(diffservo))},
#endif
#if DT_NODE_EXISTS(DT_NODELABEL(diffservorear))
    [SERVO_DIFF_REAR] = {.spec = PWM_DT_SPEC_GET(DT_NODELABEL(diffservorear))},
#endif
};

// Function to turn off all servos at highest priority if the remote is disconnected
static void rc_override_watchdog_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (1) {
        uint64_t RC_OVERRIDE_NS = rc_get_capture_ns(RC_OVERRIDE)->pulse_ns;
        int64_t override_age = k_uptime_get() - rc_get_capture_raw(RC_OVERRIDE)->timestamp;

        if (rc_get_capture_ns(RC_OVERRIDE)->pulse_ns < 50000 || k_uptime_get() - rc_get_capture_raw(RC_OVERRIDE)->timestamp > 50) {
            LOG_WRN("RC override pulse out of range (%llu ns, age=%lld ms), turning off all servos", RC_OVERRIDE_NS, override_age);
            turn_off_all_servos();
            k_sleep(K_MSEC(50));
            remote_connected = false; // Set remote connected to false
            continue;
        } else {
            remote_connected = true;
        }
        k_sleep(K_MSEC(25));
    }
}

K_THREAD_DEFINE(rc_override_watchdog_tid, 1024, rc_override_watchdog_thread, NULL, NULL, NULL, K_PRIO_PREEMPT(3), 0, 0);

static void control_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (!servos_initialized) {
        LOG_DBG("Waiting for servos to initialize...");
        k_sleep(K_MSEC(100)); // Wait a bit before checking again
    }

    LOG_INF("Control thread started");

    const struct device *rc_pwm_dev = DEVICE_DT_GET(DT_ALIAS(rc_steer));
    while (!device_is_ready(rc_pwm_dev)) {
        LOG_ERR("RC PWM device not ready!");
        k_sleep(K_MSEC(100));
    }
    LOG_INF("RC PWM device ready: %s", rc_pwm_dev->name);

    while (1) {
        if (!remote_connected) {
            k_sleep(K_MSEC(1));
            continue;
        }
        if (rc_get_capture_ns(RC_OVERRIDE)->pulse_ns > 1500000) { // 1500 us = 1_500_000 ns
            const rc_capture_ns_t *steer = rc_get_capture_ns(RC_STEER);
            LOG_DBG("RC steer: period=%llu, pulse=%llu", steer->period_ns, steer->pulse_ns);
            pwm_set(servos[SERVO_STEERING].spec.dev, servos[SERVO_STEERING].spec.channel,
                    steer->period_ns, steer->pulse_ns, 0);

            const rc_capture_ns_t *throttle = rc_get_capture_ns(RC_THROTTLE);
            LOG_DBG("RC throttle: period=%llu, pulse=%llu", throttle->period_ns, throttle->pulse_ns);
            pwm_set(servos[SERVO_THROTTLE].spec.dev, servos[SERVO_THROTTLE].spec.channel,
                    throttle->period_ns, throttle->pulse_ns, 0);

            const rc_capture_ns_t *high_gear = rc_get_capture_ns(RC_HIGH_GEAR);
            LOG_DBG("RC high gear: period=%llu, pulse=%llu", high_gear->period_ns, high_gear->pulse_ns);
            pwm_set(servos[SERVO_GEAR].spec.dev, servos[SERVO_GEAR].spec.channel,
                    high_gear->period_ns, high_gear->pulse_ns, 0);

            // Unable to set diff
        }

        k_sleep(K_MSEC(10));
    }
}

// Lower priority than override watchdog
K_THREAD_DEFINE(control_tid, 4096, control_thread, NULL, NULL, NULL, K_PRIO_PREEMPT(5), 0, 0);

void servo_init(void) {
    LOG_INF("Initializing servos...");

    // Steering
#if DT_NODE_EXISTS(DT_NODELABEL(steeringservo))
    if (!device_is_ready(servos[SERVO_STEERING].spec.dev)) {
        LOG_ERR("Steering servo device not ready");
    } else {
        LOG_INF("Steering servo ready on %s ch%u", servos[SERVO_STEERING].spec.dev->name, servos[SERVO_STEERING].spec.channel);
    }
#else
    LOG_WRN("Steering servo node not found in devicetree");
#endif

    // Gear
#if DT_NODE_EXISTS(DT_NODELABEL(gearservo))
    LOG_DBG("Configuring gear servo...");
    if (!device_is_ready(servos[SERVO_GEAR].spec.dev)) {
        LOG_ERR("Gear servo device not ready");
    } else {
        LOG_INF("Gear servo ready on %s ch%u", servos[SERVO_GEAR].spec.dev->name, servos[SERVO_GEAR].spec.channel);
    }
#else
    LOG_WRN("Gear servo node not found in devicetree");
#endif

    // Throttle
#if DT_NODE_EXISTS(DT_NODELABEL(throttleesc))
    LOG_DBG("Configuring throttle ESC...");
    if (!device_is_ready(servos[SERVO_THROTTLE].spec.dev)) {
        LOG_ERR("Throttle ESC device not ready");
    } else {
        LOG_INF("Throttle ESC ready on %s ch%u", servos[SERVO_THROTTLE].spec.dev->name, servos[SERVO_THROTTLE].spec.channel);
    }
#else
    LOG_WRN("Throttle ESC node not found in devicetree");
#endif

    // Diff
#if DT_NODE_EXISTS(DT_NODELABEL(diffservo))
    LOG_DBG("Configuring diff servo...");
    if (!device_is_ready(servos[SERVO_DIFF].spec.dev)) {
        LOG_ERR("Diff servo device not ready");
    } else {
        LOG_INF("Diff servo ready on %s ch%u", servos[SERVO_DIFF].spec.dev->name, servos[SERVO_DIFF].spec.channel);
    }
#else
    LOG_WRN("Diff servo node not found in devicetree");
#endif

    // Diff Rear
#if DT_NODE_EXISTS(DT_NODELABEL(diffservorear))
    LOG_DBG("Configuring diff rear servo...");
    if (!device_is_ready(servos[SERVO_DIFF_REAR].spec.dev)) {
        LOG_ERR("Diff rear servo device not ready");
    } else {
        LOG_INF("Diff rear servo ready on %s ch%u", servos[SERVO_DIFF_REAR].spec.dev->name, servos[SERVO_DIFF_REAR].spec.channel);
    }
#else
    LOG_WRN("Diff rear servo node not found in devicetree");
#endif

    LOG_INF("Turning off all servos at startup...");
    turn_off_all_servos(); // Ensure all servos are off initially
    servos_initialized = true;
    LOG_INF("Servos initialized");
}

void turn_off_all_servos(void) {
    // LOG_WRN("Turning off all servos!");
    for (int i = 0; i < SERVO_COUNT; ++i) {
        // LOG_INF("Turning off servo %d", i);
        pwm_set_cycles(servos[SERVO_THROTTLE].spec.dev, servos[SERVO_THROTTLE].spec.channel,
                       0, 0, 0);
    }
}

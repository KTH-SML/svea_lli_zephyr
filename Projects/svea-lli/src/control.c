#include "control.h"
#include "rc_input.h"
#include <stdlib.h>             // For abs()
#include <zephyr/drivers/pwm.h> // Needed for pwm_cycles_to_usec
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(control, LOG_LEVEL_INF);

#define ROS_CMD_TIMEOUT_MS 500
#define RC_WATCHDOG_TIMEOUT_MS 150 // Increased from 120 to give more leeway
#define PWM_PERIOD_NS 10000000     // 20 ms typical for RC servos

#define SERVO_THREAD_STACK_SIZE 1024
#define SERVO_THREAD_PRIORITY 5

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

static void steering_servo_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    while (!servos_initialized) {
        k_sleep(K_MSEC(10));
    }
    while (1) {
        pwm_set_pulse_dt(&servos[SERVO_STEERING].spec, rc_pulse[RC_STEER]);
        k_sleep(K_MSEC(20));
    }
}

static void throttle_servo_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    while (!servos_initialized) {
        k_sleep(K_MSEC(10));
    }
    while (1) {
        pwm_set_pulse_dt(&servos[SERVO_THROTTLE].spec, rc_pulse[RC_THROTTLE]);
        k_sleep(K_MSEC(20));
    }
}

static void gear_servo_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    while (!servos_initialized) {
        k_sleep(K_MSEC(10));
    }
    while (1) {
        pwm_set_pulse_dt(&servos[SERVO_GEAR].spec, rc_pulse[RC_HIGH_GEAR]);
        k_sleep(K_MSEC(500));
    }
}

// Optionally add more threads for other servos if needed

// K_THREAD_DEFINE(steering_servo_tid, SERVO_THREAD_STACK_SIZE, steering_servo_thread, NULL, NULL, NULL, SERVO_THREAD_PRIORITY, 0, 0);
// K_THREAD_DEFINE(throttle_servo_tid, SERVO_THREAD_STACK_SIZE, throttle_servo_thread, NULL, NULL, NULL, SERVO_THREAD_PRIORITY, 0, 0);
// K_THREAD_DEFINE(gear_servo_tid, SERVO_THREAD_STACK_SIZE, gear_servo_thread, NULL, NULL, NULL, SERVO_THREAD_PRIORITY, 0, 0);

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
    for (int i = 0; i < SERVO_COUNT; ++i) {
        pwm_set_cycles(servos[i].spec.dev, servos[i].spec.channel, 0, 0, 0);
    }
}

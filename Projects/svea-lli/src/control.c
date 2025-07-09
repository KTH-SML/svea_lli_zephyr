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

#define PWM_PERIOD_TICKS 10000 /* 10 ms period, prescaler = 1 µs/tick */
#define LOOP_MS 20             /* run once per RC frame (50 Hz)       */

static void control_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (!servos_initialized) {
        k_sleep(K_MSEC(10));
    }
    LOG_INF("Control thread started");

    while (1) {
        /* ---- steering -------------------------------------------------- */
        uint32_t steer_ticks = rc_get_pulse_us(RC_STEER); /* µs == ticks */
        pwm_set_cycles(servos[SERVO_STEERING].spec.dev,
                       servos[SERVO_STEERING].spec.channel,
                       servos[SERVO_STEERING].spec.period, // Use period from DT
                       steer_ticks,
                       0);

        /* ---- throttle -------------------------------------------------- */
        uint32_t throttle_ticks = rc_get_pulse_us(RC_THROTTLE);
        pwm_set_cycles(servos[SERVO_THROTTLE].spec.dev,
                       servos[SERVO_THROTTLE].spec.channel,
                       servos[SERVO_THROTTLE].spec.period, // Use period from DT
                       throttle_ticks,
                       0);

        /* ---- high gear ------------------------------------------------- */
        uint32_t gear_ticks = rc_get_pulse_us(RC_HIGH_GEAR);
        pwm_set_cycles(servos[SERVO_GEAR].spec.dev,
                       servos[SERVO_GEAR].spec.channel,
                       servos[SERVO_GEAR].spec.period, // Use period from DT
                       gear_ticks,
                       0);

        /* ---- diff (front) --------------------------------------------- */
#ifdef SERVO_DIFF
        uint32_t diff_ticks = rc_get_pulse_us(RC_STEER); /* example source */
        pwm_set_cycles(servos[SERVO_DIFF].spec.dev,
                       servos[SERVO_DIFF].spec.channel,
                       PWM_PERIOD_TICKS,
                       diff_ticks,
                       0);
#endif

        /* ---- diff (rear) ---------------------------------------------- */
#ifdef SERVO_DIFF_REAR
        uint32_t diff_rear_ticks = rc_get_pulse_us(RC_STEER);
        pwm_set_cycles(servos[SERVO_DIFF_REAR].spec.dev,
                       servos[SERVO_DIFF_REAR].spec.channel,
                       PWM_PERIOD_TICKS,
                       diff_rear_ticks,
                       0);
#endif

        /* optional debug print */
        LOG_DBG("steer=%uµs throttle=%uµs gear=%uµs",
                steer_ticks, throttle_ticks, gear_ticks);

        k_sleep(K_MSEC(LOOP_MS));
    }
}

// Lower priority than override watchdog
K_THREAD_DEFINE(control_tid, 4096, control_thread, NULL, NULL, NULL, 5, 0, 0);

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

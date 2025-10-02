/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * SVEA Note.
 *
 * Part of the SVEA Low‑Level Interface (Zephyr) application.
 * Author: Nils Kiefer
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#ifndef CONTROL_H
#define CONTROL_H

#include <zephyr/drivers/pwm.h>

typedef struct {
    struct pwm_dt_spec spec;
} servo_t;

enum {
    SERVO_STEERING = 0,
    SERVO_GEAR,
    SERVO_THROTTLE,
    SERVO_DIFF,
    SERVO_DIFF_REAR,
    SERVO_COUNT
};

typedef struct {
    int8_t steering;
    int8_t throttle;
    bool high_gear;
    bool diff;
    uint32_t timestamp;
} ros_ctrl_t;

extern ros_ctrl_t g_ros_ctrl;
extern servo_t servos[SERVO_COUNT];
extern bool remote_connected;
extern bool forward_guess;

void servo_init(void);

#endif // CONTROL_H

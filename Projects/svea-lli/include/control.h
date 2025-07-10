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
    uint8_t steering;
    uint8_t throttle;
    bool high_gear;
    bool diff;
} ros_ctrl_t;

extern ros_ctrl_t g_ros_ctrl;
extern servo_t servos[SERVO_COUNT];
extern bool remote_connected;

void servo_init(void);
void turn_off_all_servos(void);

#endif // CONTROL_H

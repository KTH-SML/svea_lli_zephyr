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

extern servo_t servos[SERVO_COUNT];

void servo_init(void);
void turn_off_all_servos(void);

#endif // CONTROL_H

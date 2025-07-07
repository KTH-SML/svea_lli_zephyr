#ifndef SERVO_H
#define SERVO_H

#include <zephyr/kernel.h>

#define SERVO_COUNT 5

void servo_init(void);
void servo_request(int id, uint32_t us);
void set_diff_state(bool activated);
void center_all_servos(void);

uint32_t us_to_cycles(uint32_t us, uint32_t period_cycles);

#endif // SERVO_H
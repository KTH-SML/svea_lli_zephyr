#pragma once
#include "remote.h"
#include <zephyr/kernel.h>

int servo_init(void);
void servo_set(enum servo_id id, uint8_t duty); /* 100-200 */
void servo_safe_centre(void);

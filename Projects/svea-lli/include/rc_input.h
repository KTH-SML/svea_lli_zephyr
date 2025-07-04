#ifndef RC_INPUT_H
#define RC_INPUT_H

#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>

void rc_input_init(void);
void common_cb(const struct device *dev, uint32_t channel,
               uint32_t period, uint32_t pulse, int status, void *user_data);
uint32_t ticks_to_us(uint32_t ticks);

#endif // RC_INPUT_H
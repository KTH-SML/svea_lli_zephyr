#ifndef WHEEL_ENC_H_
#define WHEEL_ENC_H_

#include <stdint.h>

/* Initialise both encoders (left / right) */
void wheel_enc_init(void);

/* Runtime-changeable debounce (µs) */
void wheel_enc_set_debounce_us(uint32_t left_us, uint32_t right_us);

/* Reset tick counter and timestamp (ms since boot) */
void wheel_enc_reset_left(void);
void wheel_enc_reset_right(void);

/* Query accumulated data */
uint32_t wheel_left_ticks(void);
uint32_t wheel_right_ticks(void);
uint64_t wheel_left_start_ms(void);
uint64_t wheel_right_start_ms(void);

#endif /* WHEEL_ENC_H_ */

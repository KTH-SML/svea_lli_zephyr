#ifndef WHEEL_ENC_H_
#define WHEEL_ENC_H_

#include <stdint.h>

/* one-time setup ---------------------------------------------------------- */
void wheel_enc_init(void);

/* rolling-window tick count ---------------------------------------------- */
uint16_t wheel_left_ticks(void);
uint16_t wheel_right_ticks(void);

#endif /* WHEEL_ENC_H_ */

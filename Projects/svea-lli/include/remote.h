#pragma once
#include <zephyr/kernel.h>

/* ---------- logical indices ----------------------------------------- */
enum rc_chan { RC_STEER,
               RC_GEAR,
               RC_THROTTLE,
               RC_OVERRIDE,
               RC_CNT };
enum servo_id { SERVO_STEER,
                SERVO_GEAR,
                SERVO_THROTTLE,
                SERVO_DIFF_FRONT,
                SERVO_DIFF_REAR,
                SERVO_CNT };

/* UInt8 duty: 100-200 ±10  (raw PWM 1000-2000 µs) -------------------- */
static inline uint8_t us_to_u8(uint16_t us) { return (us - 1000) / 10; }
static inline uint16_t u8_to_us(uint8_t d) { return 1000 + (uint16_t)d * 10; }

struct RcFrame {
    uint8_t steer, gear, throttle;
    uint8_t override_raw; /* still uint8; >150 ⇒ TRUE */
};

/* API used by lower / upper layers ----------------------------------- */
void rc_report(enum rc_chan ch, uint16_t pulse_us, bool overflow);
bool rc_wait(struct RcFrame *dst, k_timeout_t t);

extern volatile bool rc_valid;        /* whole link health */
extern volatile bool override_active; /* current override flag */

#include "remote.h"
#include <string.h>

K_MSGQ_DEFINE(rc_q, sizeof(struct RcFrame), 4, 4);

static struct RcFrame cur, prev;
volatile bool rc_valid = false;
volatile bool override_active = false;
static atomic_t alive_mask;
static void wdg_fn(struct k_timer *t);
K_TIMER_DEFINE(rc_wdg, wdg_fn, NULL);

void rc_report(enum rc_chan ch, uint16_t us, bool overflow) {
    if (overflow) {
        atomic_clear_bit(&alive_mask, ch);
        goto check_alive;
    }

    atomic_set_bit(&alive_mask, ch);
    uint8_t duty = us_to_u8(us);
    switch (ch) {
    case RC_STEER:
        cur.steer = duty;
        break;
    case RC_GEAR:
        cur.gear = duty;
        break;
    case RC_THROTTLE:
        cur.throttle = duty;
        break;
    case RC_OVERRIDE:
        cur.override_raw = duty;
    default:
        break;
    }

check_alive:
    rc_valid = atomic_get(&alive_mask);
    override_active = cur.override_raw > 150; /* >1.5 ms */

    if (memcmp(&cur, &prev, sizeof cur)) {
        prev = cur;
        k_msgq_put(&rc_q, &cur, K_NO_WAIT);
    }

    if (!overflow)
        k_timer_start(&rc_wdg, K_MSEC(120), K_NO_WAIT);
}

static void wdg_fn(struct k_timer *t) {
    rc_valid = false;
    k_msgq_put(&rc_q, &prev, K_NO_WAIT);
}

bool rc_wait(struct RcFrame *dst, k_timeout_t t) { return k_msgq_get(&rc_q, dst, t) == 0; }

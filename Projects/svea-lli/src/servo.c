/* servo.c – PWM output driver for four RC servos  ---------------------- */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include "servo.h" /* your public header – IDs, SERVO_COUNT */

LOG_MODULE_REGISTER(servo, LOG_LEVEL_INF);

/* --------------------------------------------------------------------- */
/* per-servo runtime data                                                */

struct servo_data {
    struct pwm_dt_spec spec; /* dev, channel and period taken from DTS */
    atomic_t target_us;
    struct k_work work;
    uint8_t id;
};

static struct servo_data srv[SERVO_COUNT];

/* --------------------------------------------------------------------- */
/* helpers                                                               */

static inline bool ready(const struct servo_data *s) {
    return device_is_ready(s->spec.dev);
}

static void apply_servo(struct k_work *work) {
    struct servo_data *s = CONTAINER_OF(work, struct servo_data, work);
    uint32_t us = atomic_get(&s->target_us);

    /* convert µs → ns with the stock helper macro */
    int ret = pwm_set_pulse_dt(&s->spec, PWM_USEC(us));
    if (ret) {
        LOG_ERR("servo %u: pwm_set_pulse_dt() -> %d", s->id, ret);
    }
}

/* --------------------------------------------------------------------- */
/* public API                                                            */

void servo_request(int id, uint32_t pulse_us) {
    if (id < 0 || id >= SERVO_COUNT) {
        LOG_ERR("servo_request: bad id %d", id);
        return;
    }

    /* clamp for safety */
    // if (pulse_us < 1000)
    //     pulse_us = 1000;
    // if (pulse_us > 2100)
    //     pulse_us = 2100;

    if (pulse_us != atomic_get(&srv[id].target_us)) {
        atomic_set(&srv[id].target_us, pulse_us);
        k_work_submit(&srv[id].work);
    }
}

/* --------------------------------------------------------------------- */
/* initialisation                                                        */

#define SERVO_INIT(idx, alias_node)                              \
    do {                                                         \
        srv[idx].spec = PWM_DT_SPEC_GET(DT_ALIAS(alias_node));   \
        srv[idx].id = idx;                                       \
        if (!ready(&srv[idx])) {                                 \
            LOG_ERR(#alias_node " PWM device not ready");        \
            break;                                               \
        }                                                        \
        k_work_init(&srv[idx].work, apply_servo);                \
        atomic_set(&srv[idx].target_us, 1500);                   \
        k_work_submit(&srv[idx].work);                           \
        LOG_INF(#alias_node " ready on %s ch%u",                 \
                srv[idx].spec.dev->name, srv[idx].spec.channel); \
    } while (0)

void servo_init(void) {
    LOG_INF("initialising servos…");

#if DT_NODE_EXISTS(DT_ALIAS(servo0))
    srv[0] = (struct servo_data){
        .spec = PWM_DT_SPEC_GET(DT_ALIAS(servo0)),
        .id = 0,
    };
    if (!ready(&srv[0])) {
        LOG_ERR("servo0 PWM device not ready");
    } else {
        k_work_init(&srv[0].work, apply_servo);
        atomic_set(&srv[0].target_us, 1500);
        k_work_submit(&srv[0].work);
        LOG_INF("servo0 ready on %s ch%u", srv[0].spec.dev->name, srv[0].spec.channel);
    }
#endif

#if DT_NODE_EXISTS(DT_ALIAS(servo1))
    srv[1] = (struct servo_data){
        .spec = PWM_DT_SPEC_GET(DT_ALIAS(servo1)),
        .id = 1,
    };
    if (!ready(&srv[1])) {
        LOG_ERR("servo1 PWM device not ready");
    } else {
        k_work_init(&srv[1].work, apply_servo);
        atomic_set(&srv[1].target_us, 1500);
        k_work_submit(&srv[1].work);
        LOG_INF("servo1 ready on %s ch%u", srv[1].spec.dev->name, srv[1].spec.channel);
    }
#endif

#if DT_NODE_EXISTS(DT_ALIAS(servo2))
    srv[2] = (struct servo_data){
        .spec = PWM_DT_SPEC_GET(DT_ALIAS(servo2)),
        .id = 2,
    };
    if (!ready(&srv[2])) {
        LOG_ERR("servo2 PWM device not ready");
    } else {
        k_work_init(&srv[2].work, apply_servo);
        atomic_set(&srv[2].target_us, 1500);
        k_work_submit(&srv[2].work);
        LOG_INF("servo2 ready on %s ch%u", srv[2].spec.dev->name, srv[2].spec.channel);
    }
#endif

#if DT_NODE_EXISTS(DT_ALIAS(servo3))
    srv[3] = (struct servo_data){
        .spec = PWM_DT_SPEC_GET(DT_ALIAS(servo3)),
        .id = 3,
    };
    if (!ready(&srv[3])) {
        LOG_ERR("servo3 PWM device not ready");
    } else {
        k_work_init(&srv[3].work, apply_servo);
        atomic_set(&srv[3].target_us, 1500);
        k_work_submit(&srv[3].work);
        LOG_INF("servo3 ready on %s ch%u", srv[3].spec.dev->name, srv[3].spec.channel);
    }
#endif
}

/* --------------------------------------------------------------------- */

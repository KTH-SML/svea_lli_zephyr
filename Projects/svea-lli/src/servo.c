#include "servo.h"
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>

/* ---- pull everything from overlay aliases ------------------------- */
#define SERVO_NODE(i) DT_ALIAS(servo##i)
#define SERVO_DEV(i) DEVICE_DT_GET(DT_PWMS_CTLR(SERVO_NODE(i)))
#define SERVO_CH(i) DT_PWMS_CHANNEL_BY_IDX(SERVO_NODE(i), 0)
#define SERVO_PER(i) DT_PWMS_PERIOD_BY_IDX(SERVO_NODE(i), 0)

static struct {
    const struct device *dev;
    uint32_t ch, per;
    atomic_t duty; /* 100-200               */
    struct k_work work;
} s[SERVO_CNT];

static void apply(struct k_work *w) {
    int i = (int)w->user_data;
    uint32_t us = u8_to_us(atomic_get(&s[i].duty));
    pwm_set_cycles(s[i].dev, s[i].ch, s[i].per,
                   pwm_us_to_cycles(s[i].dev, us), 0);
}

int servo_init(void) {
    for (int i = 0; i < SERVO_CNT; ++i) {
        s[i].dev = SERVO_DEV(i);
        s[i].ch = SERVO_CH(i);
        s[i].per = SERVO_PER(i);
        k_work_init(&s[i].work, apply);
        s[i].work.user_data = (void *)i;
        atomic_set(&s[i].duty, 150); /* centre */
    }
    return 0;
}

void servo_set(enum servo_id id, uint8_t duty) {
    atomic_set(&s[id].duty, duty);
    k_work_submit(&s[id].work);
}

void servo_safe_centre(void) {
    for (int i = 0; i < SERVO_CNT; ++i)
        servo_set(i, 150);
}

/* src/control.c  –  fast, non-blocking servo output -------------------- */
#include "control.h"
#include "rc_input.h" /* rc_get_pulse_us() prototype        */

#include <stdlib.h>       // <-- Add this line
#include <stm32_ll_tim.h> /* low-level TIM helpers               */
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <stm32_ll_tim.h> /* already included, but make sure */

LOG_MODULE_REGISTER(control, LOG_LEVEL_INF);

/* ───── 1. servo bookkeeping (unchanged) ──────────────────────────────── */
bool servos_initialized = false;
bool remote_connected = false;

servo_t servos[SERVO_COUNT] = {
    [SERVO_STEERING] = {.spec = PWM_DT_SPEC_GET(DT_NODELABEL(steeringservo))},
    [SERVO_GEAR] = {.spec = PWM_DT_SPEC_GET(DT_NODELABEL(gearservo))},
    [SERVO_THROTTLE] = {.spec = PWM_DT_SPEC_GET(DT_NODELABEL(throttleesc))},
    [SERVO_DIFF] = {.spec = PWM_DT_SPEC_GET(DT_NODELABEL(diffservo))},
    [SERVO_DIFF_REAR] = {.spec = PWM_DT_SPEC_GET(DT_NODELABEL(diffservorear))},
};

/* grab TIM base the Zephyr PWM driver created */
struct pwm_stm32_cfg_hack {
    TIM_TypeDef *timer;
};
static inline TIM_TypeDef *pwm_to_tim(const struct pwm_dt_spec *s) {
    return ((const struct pwm_stm32_cfg_hack *)s->dev->config)->timer;
}

static void arm_servo(const struct pwm_dt_spec *s) {
    TIM_TypeDef *T =
        ((const struct pwm_stm32_cfg_hack *)s->dev->config)->timer;

    /*  enable channel output */
    switch (s->channel) {
    case 1:
        LL_TIM_CC_EnableChannel(T, LL_TIM_CHANNEL_CH1);
        break;
    case 2:
        LL_TIM_CC_EnableChannel(T, LL_TIM_CHANNEL_CH2);
        break;
    case 3:
        LL_TIM_CC_EnableChannel(T, LL_TIM_CHANNEL_CH3);
        break;
    case 4:
        LL_TIM_CC_EnableChannel(T, LL_TIM_CHANNEL_CH4);
        break;
    }

    /* advanced timers (TIM1, TIM8) need MOE */
    if (T == TIM1 || T == TIM8) {
        LL_TIM_EnableAllOutputs(T);
    }
}

#define SERVO_PERIOD_TICKS 10000U /* 10 ms @ 1 µs/tick               */
#define SERVO_MID_TICKS 1500U     /* safe neutral position           */

/* disable preload once ⇒ later writes hit CCR immediately */
static inline void servo_raw_init_once(const struct pwm_dt_spec *s) {
    static bool done[SERVO_COUNT];
    int idx = s - &servos[0].spec; /* relies on same order       */
    if (done[idx])
        return;
    TIM_TypeDef *T = pwm_to_tim(s);
    switch (s->channel) {
    case 1:
        LL_TIM_OC_DisablePreload(T, LL_TIM_CHANNEL_CH1);
        break;
    case 2:
        LL_TIM_OC_DisablePreload(T, LL_TIM_CHANNEL_CH2);
        break;
    case 3:
        LL_TIM_OC_DisablePreload(T, LL_TIM_CHANNEL_CH3);
        break;
    case 4:
        LL_TIM_OC_DisablePreload(T, LL_TIM_CHANNEL_CH4);
        break;
    }
    done[idx] = true;
}

void servo_init(void) {
    /* Let the Zephyr driver set up every timer & pin exactly once */
    for (int i = 0; i < SERVO_COUNT; ++i) {
        pwm_set_cycles(servos[i].spec.dev,
                       servos[i].spec.channel,
                       servos[i].spec.period,
                       SERVO_MID_TICKS, /* or 0 if you prefer low     */
                       0);
    }

    /* Now kill preload so future raw writes land immediately */
    for (int i = 0; i < SERVO_COUNT; ++i) {
        servo_raw_init_once(&servos[i].spec);
    }

    servos_initialized = true;
}
/* write compare value (ticks = 1 µs) */
static inline void servo_set_ticks(const struct pwm_dt_spec *s, uint32_t t) {
    // TIM_TypeDef *T = pwm_to_tim(s);
    // switch (s->channel) {
    // case 1:
    //     LL_TIM_OC_SetCompareCH1(T, t);
    //     break;
    // case 2:
    //     LL_TIM_OC_SetCompareCH2(T, t);
    //     break;
    // case 3:
    //     LL_TIM_OC_SetCompareCH3(T, t);
    //     break;
    // case 4:
    //     LL_TIM_OC_SetCompareCH4(T, t);
    //     break;
    // }
    pwm_set_cycles(s->dev, s->channel, s->period, t, 0);
}

/* ───── 3. control thread – 20 ms loop, non-blocking ─────────────────── */
#define LOOP_MS 2 // Run faster (every 10ms)
#define AVG_WINDOW_MS 31
#define AVG_WINDOW (AVG_WINDOW_MS / LOOP_MS)

typedef struct {
    uint32_t buf[AVG_WINDOW];
    uint8_t idx;
    uint32_t sum;
} rolling_avg_t;

static void rolling_avg_init(rolling_avg_t *a, uint32_t init) {
    for (int i = 0; i < AVG_WINDOW; ++i)
        a->buf[i] = init;
    a->sum = init * AVG_WINDOW;
    a->idx = 0;
}

static uint32_t rolling_avg_update(rolling_avg_t *a, uint32_t val) {
    a->sum -= a->buf[a->idx];
    a->sum += val;
    a->buf[a->idx] = val;
    a->idx = (a->idx + 1) % AVG_WINDOW;
    return a->sum / AVG_WINDOW;
}

#define MAX_SERVO_RATE_US_PER_S 4000 // max allowed change per second (µs)
#define MAX_SERVO_DELTA_PER_SAMPLE ((MAX_SERVO_RATE_US_PER_S * LOOP_MS) / 1000)

static uint32_t filter_spike(uint32_t prev, uint32_t curr) {
    if (curr == 0)
        return prev;
    int32_t delta = (int32_t)curr - (int32_t)prev;
    if (delta > MAX_SERVO_DELTA_PER_SAMPLE)
        return prev + MAX_SERVO_DELTA_PER_SAMPLE;
    if (delta < -MAX_SERVO_DELTA_PER_SAMPLE)
        return prev - MAX_SERVO_DELTA_PER_SAMPLE;
    return curr;
}

static void control_thread(void *, void *, void *) {
    while (!servos_initialized) {
        k_sleep(K_MSEC(10));
    }

    LOG_INF("Control thread started, servos initialized.");

    // Initialize rolling averages with mid values
    rolling_avg_t steer_avg, thr_avg, gear_avg;
    rolling_avg_init(&steer_avg, SERVO_MID_TICKS);
    rolling_avg_init(&thr_avg, SERVO_MID_TICKS);
    rolling_avg_init(&gear_avg, SERVO_MID_TICKS);

    static uint32_t prev_steer = SERVO_MID_TICKS, prev_thr = SERVO_MID_TICKS, prev_gear = SERVO_MID_TICKS;
    servo_raw_init_once(&servos[SERVO_STEERING].spec);
    servo_raw_init_once(&servos[SERVO_THROTTLE].spec);
    servo_raw_init_once(&servos[SERVO_GEAR].spec);
    servo_raw_init_once(&servos[SERVO_DIFF].spec);
    servo_raw_init_once(&servos[SERVO_DIFF_REAR].spec);

    for (;;) {
        uint32_t steer_raw = rc_get_pulse_us(RC_STEER);
        uint32_t thr_raw = rc_get_pulse_us(RC_THROTTLE);
        uint32_t gear_raw = rc_get_pulse_us(RC_HIGH_GEAR);

        uint32_t steer_avg_val = steer_avg.sum / AVG_WINDOW;
        uint32_t thr_avg_val = thr_avg.sum / AVG_WINDOW;
        uint32_t gear_avg_val = gear_avg.sum / AVG_WINDOW;

        // Uncomment and use filter_spike to define steer, thr, gear
        uint32_t steer = filter_spike(prev_steer, steer_raw);
        uint32_t thr = filter_spike(prev_thr, thr_raw);
        uint32_t gear = filter_spike(prev_gear, gear_raw);

        prev_steer = steer;
        prev_thr = thr;
        prev_gear = gear;

        uint32_t steer_f = rolling_avg_update(&steer_avg, steer);
        uint32_t thr_f = rolling_avg_update(&thr_avg, thr);
        uint32_t gear_f = rolling_avg_update(&gear_avg, gear);

        LOG_DBG("RC pulses: steer=%u, throttle=%u, gear=%u | avg: steer=%u, throttle=%u, gear=%u",
                steer, thr, gear, steer_f, thr_f, gear_f);

        servo_set_ticks(&servos[SERVO_STEERING].spec, steer_f);
        servo_set_ticks(&servos[SERVO_THROTTLE].spec, thr_f);
        servo_set_ticks(&servos[SERVO_GEAR].spec, gear_f);

        LOG_DBG("Servo outputs set: steer=%u, throttle=%u, gear=%u", steer_f, thr_f, gear_f);

        k_sleep(K_MSEC(LOOP_MS));
    }
}

K_THREAD_DEFINE(control_tid, 2048, control_thread, NULL, NULL, NULL,
                2 /* prio */, 0, 0);

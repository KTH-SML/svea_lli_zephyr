/* rc_input.c – pure-hardware capture, no ISR, no overlay tweaks */

#include "rc_input.h"
#include <stm32_ll_bus.h>
#include <stm32_ll_tim.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(rc_input_hw, LOG_LEVEL_INF);

/* ───────── helpers to fetch the TIM base pointer from a pwm device ───────── */

struct pwm_stm32_cfg {
    TIM_TypeDef *timer;
}; /* forward decl. */

static inline TIM_TypeDef *pwm_to_tim(const struct device *pwm_dev) {
    return ((const struct pwm_stm32_cfg *)pwm_dev->config)->timer;
}

/* ───────── table that matches your /aliases ───────── */

#define RC_STEER 0
#define RC_HIGH_GEAR 1
#define RC_THROTTLE 2
#define RC_OVERRIDE 3
#define NUM_RC_CH 4

static const struct device *const pwm_dev[NUM_RC_CH] = {
    DEVICE_DT_GET(DT_ALIAS(rc_steer)),
    DEVICE_DT_GET(DT_ALIAS(rc_high_gear)),
    DEVICE_DT_GET(DT_ALIAS(rc_throttle)),
    DEVICE_DT_GET(DT_ALIAS(rc_override)),
};

/* ───────── hardware re-configuration ───────── */

static void tim_config_pwm_input(TIM_TypeDef *T) {
    /* 1 µs tick: APB1 108 MHz / (108-1) = 1 MHz */
    LL_TIM_DisableCounter(T);
    // LL_TIM_SetPrescaler(T, 107); set in overlay
    LL_TIM_SetAutoReload(T, 0xFFFF);

    /* CH1 = rising direct, CH2 = falling indirect */
    LL_TIM_IC_SetActiveInput(T, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
    LL_TIM_IC_SetActiveInput(T, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_INDIRECTTI);
    LL_TIM_IC_SetPolarity(T, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);
    LL_TIM_IC_SetPolarity(T, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_FALLING);
    LL_TIM_IC_SetPrescaler(T, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
    LL_TIM_IC_SetPrescaler(T, LL_TIM_CHANNEL_CH2, LL_TIM_ICPSC_DIV1);

    /* reset counter on every rising edge */
    LL_TIM_SetTriggerInput(T, LL_TIM_TS_TI1FP1);
    LL_TIM_SetSlaveMode(T, LL_TIM_SLAVEMODE_RESET);

    /* disable any IRQ the Zephyr driver might have left on */
    LL_TIM_DisableIT_CC1(T);
    LL_TIM_DisableIT_CC2(T);
    LL_TIM_ClearFlag_CC1(T);
    LL_TIM_ClearFlag_CC2(T);

    LL_TIM_CC_EnableChannel(T, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2);
    LL_TIM_EnableCounter(T);
}

/* ───────── public API ───────── */

static TIM_TypeDef *tim[NUM_RC_CH]; /* saved after init */

uint32_t rc_get_age_us(rc_channel_t ch) {
    if (ch < 0 || ch >= NUM_RC_CH || tim[ch] == NULL) {
        return UINT32_MAX; /* invalid channel */
    }
    return LL_TIM_GetCounter(tim[ch]); /* direct read of CNT */
}

void rc_input_init(void) {
    for (int i = 0; i < NUM_RC_CH; i++) {
        if (!device_is_ready(pwm_dev[i])) {
            LOG_ERR("RC dev %d not ready", i);
            continue;
        }
        tim[i] = pwm_to_tim(pwm_dev[i]);
        tim_config_pwm_input(tim[i]);
        LOG_INF("RC timer %d set to PWM-input", i);
    }
}

uint32_t rc_get_pulse_us(rc_channel_t idx) {
    if (idx < 0 || idx >= NUM_RC_CH || tim[idx] == NULL) {
        return 0;
    }

    uint32_t pulse = LL_TIM_IC_GetCaptureCH2(tim[idx]);
    // Clamp to [1000, 2000] if in plausible range, else return 0
    if (pulse >= 1000 && pulse <= 2000) {
        return pulse;
    }
    if (pulse > 900 && pulse < 1000) {
        return 1000;
    }
    if (pulse > 2000 && pulse < 2100) {
        return 2000;
    }
    return 0; // Out of plausible
}

uint32_t rc_get_period_us(rc_channel_t idx) {
    return idx < NUM_RC_CH ? LL_TIM_IC_GetCaptureCH1(tim[idx]) : 0;
}

uint32_t rc_get_capture_raw(rc_channel_t ch) {
    // Defensive: handle out-of-bounds and uninitialized timer
    if (ch < 0 || ch >= NUM_RC_CH || tim[ch] == NULL) {
        return 0;
    }
    // By convention, CH2 is pulse width (high time), CH1 is period
    // Convert from microseconds to nanoseconds (multiply by 1000)
    return LL_TIM_IC_GetCaptureCH2(tim[ch]);
}

/* ───────── optional logger thread ───────── */

static void rc_log(void *, void *, void *) {
    rc_input_init();
    while (1) {
        LOG_INF("steer %u us  throttle %u us  gear %u us  override %u us  override_age %u us",
                rc_get_capture_raw(RC_STEER),
                rc_get_capture_raw(RC_THROTTLE),
                rc_get_capture_raw(RC_HIGH_GEAR),
                rc_get_capture_raw(RC_OVERRIDE),
                rc_get_age_us(RC_OVERRIDE));
        k_msleep(500);
    }
}
K_THREAD_DEFINE(rc_log_tid, 1024, rc_log, NULL, NULL, NULL, 7, 0, 0);

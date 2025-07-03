#include "pwm.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>

/* --------------------
 *  Globals & settings
 * ------------------ */
struct pwm_in_channel pwm_inputs[PWM_INPUTS_MAX];
bool override_mode = false; /* Robotics control by default */
bool rc_remote_disconnected = false;

#define OVERRIDE_THRESHOLD_US 1500u /* diff‑channel pulse ≥1.5 ms ⇒ RC override */
static bool last_override_state = false;

/* Direct pointers to current RC values */
float remote_steering_norm_value = 0.f;
float remote_gear_norm_value = 0.f;
float remote_diff_norm_value = 0.f;
float remote_esc_norm_value = 0.f;

/* Device tree specs ---------------------------------------------------- */
const struct pwm_dt_spec steering_pwm = PWM_DT_SPEC_GET(DT_NODELABEL(steeringservo));
const struct pwm_dt_spec gear_pwm = PWM_DT_SPEC_GET(DT_NODELABEL(gearservo));
const struct pwm_dt_spec diff_pwm = PWM_DT_SPEC_GET(DT_NODELABEL(diffservo));
const struct pwm_dt_spec throttle_pwm = PWM_DT_SPEC_GET(DT_NODELABEL(throttleesc));

const struct device *const pwm3_dev = DEVICE_DT_GET(PWM3_NODE);
const struct device *const pwm4_dev = DEVICE_DT_GET(PWM4_NODE);
const struct device *const pwm5_dev = DEVICE_DT_GET(PWM5_NODE);
const struct device *const pwm9_dev = DEVICE_DT_GET(PWM9_NODE);

/* Thread stack --------------------------------------------------------- */
K_THREAD_STACK_DEFINE(pwm_in_stack, 1024);
static struct k_thread pwm_in_thread_data;

/* -------------------- helpers ---------------------------------------- */
static inline float norm_from_us(int32_t pulse_us) {
    return 2.f * (pulse_us - REMOTE_PWM_MIN_US) / (float)(REMOTE_PWM_MAX_US - REMOTE_PWM_MIN_US) - 1.f;
}

static inline bool duty_valid(int32_t pulse_us) {
    return (pulse_us >= REMOTE_PWM_CLIP_MIN_US && pulse_us <= REMOTE_PWM_CLIP_MAX_US);
}

static inline bool duty_changed(int32_t new_us, int32_t prev_us) {
    return (prev_us == -1 || abs(new_us - prev_us) > 3); /* 3 µs hysteresis */
}

static inline void update_override_mode(int32_t diff_us);
static void relay_rc(struct pwm_in_channel *in, int32_t pulse_us);

static int32_t last_capture_us[PWM_INPUTS_MAX] = {-1, -1, -1, -1};

/* -------------------- capture callback -------------------------------- */
static void pwm_capture_cb(const struct device *dev, uint32_t chan,
                           uint32_t period_cycles, uint32_t pulse_cycles,
                           int status, void *user_data) {
    struct pwm_in_channel *in = (struct pwm_in_channel *)user_data;
    const int idx = in - pwm_inputs;

    if (status) {
        static uint32_t err_cnt = 0;
        if (idx == PWM_CH_GEAR && (++err_cnt % 2) == 0) {
            rc_remote_disconnected = true;
            set_pwm_pulse_us(in->out_pwm, (uint32_t)0);
        }
        return;
    }

    if (idx == PWM_CH_GEAR)
        rc_remote_disconnected = false;

    uint64_t pulse_us = 0;
    pwm_cycles_to_usec(dev, chan, pulse_cycles, &pulse_us);

    int32_t p_us = (int32_t)pulse_us;

    if (!duty_valid(p_us))
        return; /* toss outlier */

    if (idx == PWM_CH_DIFF)
        update_override_mode(p_us);

    if (!duty_changed(p_us, last_capture_us[idx]))
        return; /* debounced */
    
    // Save the last valid capture for immediate use in override mode
    last_capture_us[idx] = p_us;

    float norm = norm_from_us(p_us);
    if (in->norm_value_ptr)
        *in->norm_value_ptr = norm;

    relay_rc(in, p_us);
    publish_rc_message(in, norm);
}

static void update_override_mode(int32_t diff_us) {
    bool new_state = (diff_us >= OVERRIDE_THRESHOLD_US);
    if (new_state != last_override_state) {
        override_mode = new_state;
        last_override_state = new_state;
        printf("pwm_relay: override %s (diff %d µs)\n",
               override_mode ? "ENABLED — RC" : "DISABLED — robot", diff_us);
        
        if (override_mode) {
            /* When switching to override mode, relay the latest captured values */
            for (pwm_channel_t ch = 0; ch < PWM_INPUTS_MAX; ++ch) {
                if (pwm_inputs[ch].out_pwm && last_capture_us[ch] != -1) {
                    // Use the actual captured pulse width directly
                    set_pwm_pulse_us(pwm_inputs[ch].out_pwm, (uint32_t)last_capture_us[ch]);
                    printf("pwm_relay: Applied override value %d µs to channel %d\n", 
                           last_capture_us[ch], ch);
                }
            }
        }
    }
}

static void relay_rc(struct pwm_in_channel *in, int32_t pulse_us) {
    if (!override_mode || !in->out_pwm)
        return; /* robotics or no mapping */

    set_pwm_pulse_us(in->out_pwm, (uint32_t)pulse_us);
}

/* -------------------- input thread ----------------------------------- */
static void pwm_input_thread(void *a, void *b, void *c) {
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    for (pwm_channel_t ch = 0; ch < PWM_INPUTS_MAX; ++ch) {
        if (!pwm_inputs[ch].dev || !device_is_ready(pwm_inputs[ch].dev)) {
            printf("pwm_relay: device not ready on ch %d\n", ch);
            continue;
        }

        int rc = pwm_configure_capture(pwm_inputs[ch].dev, pwm_inputs[ch].channel,
                                       PWM_CAPTURE_TYPE_BOTH | PWM_CAPTURE_MODE_CONTINUOUS,
                                       pwm_capture_cb, &pwm_inputs[ch]);
        if (rc) {
            printf("pwm_relay: configure failed ch %d (%d)\n", ch, rc);
            continue;
        }
        rc = pwm_enable_capture(pwm_inputs[ch].dev, pwm_inputs[ch].channel);
        if (rc)
            printf("pwm_relay: enable failed ch %d (%d)\n", ch, rc);
    }

    for (;;)
        k_sleep(K_FOREVER);
}

/* -------------------- init ------------------------------------------- */
void pwm_in_init(void) {
    pwm_inputs[PWM_CH_STEERING] = (struct pwm_in_channel){
        .dev = pwm3_dev, .channel = 1, .topic = "/lli/remote/steering/norm", .norm_value_ptr = &remote_steering_norm_value, .out_pwm = &steering_pwm, .out_min_ns = DT_PROP(DT_NODELABEL(steeringservo), min_pulse), .out_max_ns = DT_PROP(DT_NODELABEL(steeringservo), max_pulse)};

    pwm_inputs[PWM_CH_GEAR] = (struct pwm_in_channel){
        .dev = pwm9_dev, .channel = 1, .topic = "/lli/remote/gear/norm", .norm_value_ptr = &remote_gear_norm_value, .out_pwm = &gear_pwm, .out_min_ns = DT_PROP(DT_NODELABEL(gearservo), min_pulse), .out_max_ns = DT_PROP(DT_NODELABEL(gearservo), max_pulse)};

    pwm_inputs[PWM_CH_DIFF] = (struct pwm_in_channel){
        .dev = pwm4_dev, .channel = 3, .topic = "/lli/remote/diff/norm", .norm_value_ptr = &remote_diff_norm_value, .out_pwm = &diff_pwm, .out_min_ns = DT_PROP(DT_NODELABEL(diffservo), min_pulse), .out_max_ns = DT_PROP(DT_NODELABEL(diffservo), max_pulse)};

    pwm_inputs[PWM_CH_ESC] = (struct pwm_in_channel){
        .dev = pwm5_dev, .channel = 1, .topic = "/lli/remote/esc/norm", .norm_value_ptr = &remote_esc_norm_value, .out_pwm = &throttle_pwm, .out_min_ns = DT_PROP(DT_NODELABEL(throttleesc), min_pulse), .out_max_ns = DT_PROP(DT_NODELABEL(throttleesc), max_pulse)};

    /* Apply neutral positions so servos are never left floating */
    set_pwm_norm(&steering_pwm, 0.f, pwm_inputs[PWM_CH_STEERING].out_min_ns, pwm_inputs[PWM_CH_STEERING].out_max_ns);
    set_pwm_norm(&gear_pwm, 0.f, pwm_inputs[PWM_CH_GEAR].out_min_ns, pwm_inputs[PWM_CH_GEAR].out_max_ns);
    set_pwm_norm(&diff_pwm, 0.f, pwm_inputs[PWM_CH_DIFF].out_min_ns, pwm_inputs[PWM_CH_DIFF].out_max_ns);
    set_pwm_norm(&throttle_pwm, 0.f, pwm_inputs[PWM_CH_ESC].out_min_ns, pwm_inputs[PWM_CH_ESC].out_max_ns);

    k_thread_create(&pwm_in_thread_data, pwm_in_stack,
                    K_THREAD_STACK_SIZEOF(pwm_in_stack),
                    pwm_input_thread, NULL, NULL, NULL,
                    REMOTE_PWM_THREAD_PRIORITY, 0, K_NO_WAIT);
}
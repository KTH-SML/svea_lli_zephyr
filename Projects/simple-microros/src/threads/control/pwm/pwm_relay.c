#include "pwm.h"
#include <std_msgs/msg/float32.h>
#include <stdio.h>
#include <stdlib.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>

// --- Global Variables Definitions ---
struct pwm_in_channel pwm_inputs[PWM_INPUTS_MAX];
bool override_mode = true; // Set to true for testing RC relay

// --- Remote control values ---
float remote_steering_norm_value = 0.0f;
float remote_gear_norm_value = 0.0f;
float remote_diff_norm_value = 0.0f;
float remote_esc_norm_value = 0.0f;

// --- PWM Device Tree Specifications with inline definitions ---
const struct pwm_dt_spec steering_pwm = PWM_DT_SPEC_GET(DT_NODELABEL(steeringservo));
const struct pwm_dt_spec gear_pwm = PWM_DT_SPEC_GET(DT_NODELABEL(gearservo));
const struct pwm_dt_spec diff_pwm = PWM_DT_SPEC_GET(DT_NODELABEL(diffservo));
const struct pwm_dt_spec throttle_pwm = PWM_DT_SPEC_GET(DT_NODELABEL(throttleesc));

// --- Device Definitions ---
const struct device *const pwm3_dev = DEVICE_DT_GET(PWM3_NODE);
const struct device *const pwm4_dev = DEVICE_DT_GET(PWM4_NODE);
const struct device *const pwm5_dev = DEVICE_DT_GET(PWM5_NODE);
const struct device *const pwm9_dev = DEVICE_DT_GET(PWM9_NODE);

// --- Thread Stack and Data ---
K_THREAD_STACK_DEFINE(pwm_in_stack, 1024);
struct k_thread pwm_in_thread_data;

// --- Helper Functions ---
static float calculate_normalized_value(int32_t duty_us) {
    return 2.0f * (duty_us - REMOTE_PWM_MIN_US) / (REMOTE_PWM_MAX_US - REMOTE_PWM_MIN_US) - 1.0f;
}

static bool is_duty_valid(int32_t duty_us) {
    return (duty_us >= REMOTE_PWM_MIN_US && duty_us <= REMOTE_PWM_MAX_US);
}

static bool duty_changed_significantly(int32_t new_duty, int32_t last_duty) {
    return (last_duty == -1 || abs(new_duty - last_duty) > 2); // Increase threshold to reduce noise
}

static void get_output_pwm_limits(const struct pwm_dt_spec *out_pwm,
                                  uint32_t *min_ns, uint32_t *max_ns) {
    //*min_ns = 1000000; // 1ms default
    //*max_ns = 2000000; // 2ms default

    if (out_pwm == &steering_pwm) {
        *min_ns = DT_PROP(DT_NODELABEL(steeringservo), min_pulse);
        *max_ns = DT_PROP(DT_NODELABEL(steeringservo), max_pulse);
    } else if (out_pwm == &diff_pwm) {
        *min_ns = DT_PROP(DT_NODELABEL(diffservo), min_pulse);
        *max_ns = DT_PROP(DT_NODELABEL(diffservo), max_pulse);
    } else if (out_pwm == &gear_pwm) {
        *min_ns = DT_PROP(DT_NODELABEL(gearservo), min_pulse);
        *max_ns = DT_PROP(DT_NODELABEL(gearservo), max_pulse);
    } else if (out_pwm == &throttle_pwm) {
        *min_ns = DT_PROP(DT_NODELABEL(throttleesc), min_pulse);
        *max_ns = DT_PROP(DT_NODELABEL(throttleesc), max_pulse);
    }
}

static void handle_pwm_relay(struct pwm_in_channel *input, float norm_value) {
    if (!input->out_pwm) {
        return;
    }

    // Only relay when override_mode is TRUE (RC control active)
    if (!override_mode) {
        printf("pwm_relay: Override mode disabled, not relaying RC signal\n");
        return;
    }

    uint32_t min_ns, max_ns;
    get_output_pwm_limits(input->out_pwm, &min_ns, &max_ns);

    printf("pwm_relay: Relaying RC %s: norm=%.2f (limits: %u-%u)\n",
           input->topic, norm_value, min_ns, max_ns);

    // Use the PWM actuator function for relay
    set_pwm_norm(input->out_pwm, norm_value, min_ns, max_ns);
}

// --- PWM Capture Callback ---
static void pwm_capture_callback(const struct device *dev, uint32_t chan,
                                 uint32_t period_cycles, uint32_t pulse_cycles,
                                 int status, void *user_data) {
    struct pwm_in_channel *input = (struct pwm_in_channel *)user_data;
    static int32_t last_duty[PWM_INPUTS_MAX] = {-1, -1, -1, -1};
    int channel_index = input - pwm_inputs;

    // Handle errors
    if (status != 0) {
        // Only log errors occasionally to reduce spam
        static uint32_t error_count = 0;
        if ((error_count++ % 100) == 0) {
            printf("pwm_relay: Capture errors on dev=%p chan=%u (count: %u)\n", dev, chan, error_count);
        }
        return;
    }

    // Convert cycles to microseconds
    uint64_t period_us = 0, pulse_us = 0;
    pwm_cycles_to_usec(dev, chan, period_cycles, &period_us);
    pwm_cycles_to_usec(dev, chan, pulse_cycles, &pulse_us);

    // Calculate duty cycle in microseconds
    int32_t duty_us = (period_us > 0) ? (pulse_us * 1000) / period_us : -1;

    // Validate duty cycle range - silently discard invalid readings
    if (!is_duty_valid(duty_us)) {
        return; // Remove the printf spam here
    }

    // Check if duty changed significantly
    if (!duty_changed_significantly(duty_us, last_duty[channel_index])) {
        return;
    }

    printf("pwm_relay: RC dev=%p chan=%u period=%llu pulse=%llu duty=%d\n",
           dev, chan, period_us, pulse_us, duty_us);

    // Update last duty value
    last_duty[channel_index] = duty_us;

    // Calculate normalized value (-1.0 to +1.0)
    float norm_value = calculate_normalized_value(duty_us);

    // Update normalized value pointer
    if (input->norm_value_ptr) {
        *(input->norm_value_ptr) = norm_value;
    }

    // Handle PWM relay (RC passthrough mode)
    handle_pwm_relay(input, norm_value);

    // Publish RC message (delegate to publisher)
    publish_rc_message(input, norm_value);
}

// --- PWM Input Thread ---
static void pwm_input_thread(void *arg1, void *arg2, void *arg3) {
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    printf("pwm_relay: Thread started\n");

    // Configure and enable capture for each channel
    for (pwm_channel_t ch = 0; ch < PWM_INPUTS_MAX; ++ch) {
        if (!pwm_inputs[ch].dev) {
            printf("pwm_relay: No device for channel %d\n", ch);
            continue;
        }

        if (!device_is_ready(pwm_inputs[ch].dev)) {
            printf("pwm_relay: Device not ready for channel %d\n", ch);
            continue;
        }

        printf("pwm_relay: Configuring capture: dev=%p chan=%u topic=%s\n",
               pwm_inputs[ch].dev, pwm_inputs[ch].channel, pwm_inputs[ch].topic);

        // Configure PWM capture
        int ret = pwm_configure_capture(
            pwm_inputs[ch].dev,
            pwm_inputs[ch].channel,
            PWM_CAPTURE_TYPE_BOTH | PWM_CAPTURE_MODE_CONTINUOUS,
            pwm_capture_callback,
            &pwm_inputs[ch]);

        if (ret != 0) {
            printf("pwm_relay: Capture config failed for chan=%u: %d\n",
                   pwm_inputs[ch].channel, ret);
            continue;
        }

        // Enable PWM capture
        ret = pwm_enable_capture(pwm_inputs[ch].dev, pwm_inputs[ch].channel);
        if (ret != 0) {
            printf("pwm_relay: Capture enable failed for chan=%u: %d\n",
                   pwm_inputs[ch].channel, ret);
            continue;
        }

        printf("pwm_relay: Capture enabled for channel %u\n", pwm_inputs[ch].channel);
    }

    // Thread sleeps forever - all work is done in callbacks
    while (1) {
        k_sleep(K_FOREVER);
    }
}

// --- Initialization Functions ---
void pwm_in_init(void) {
    // Initialize PWM input channel configurations
    pwm_inputs[PWM_CH_STEERING] = (struct pwm_in_channel){
        .dev = pwm3_dev,
        .channel = 1,
        .topic = "/steering/duty",
        .norm_value_ptr = &remote_steering_norm_value,
        .out_pwm = &steering_pwm};

    pwm_inputs[PWM_CH_GEAR] = (struct pwm_in_channel){
        .dev = pwm9_dev,
        .channel = 1,
        .topic = "/gear/duty",
        .norm_value_ptr = &remote_gear_norm_value,
        .out_pwm = &gear_pwm};

    pwm_inputs[PWM_CH_DIFF] = (struct pwm_in_channel){
        .dev = pwm4_dev,
        .channel = 3,
        .topic = "/diff/duty",
        .norm_value_ptr = &remote_diff_norm_value,
        .out_pwm = &diff_pwm};

    pwm_inputs[PWM_CH_ESC] = (struct pwm_in_channel){
        .dev = pwm5_dev,
        .channel = 1,
        .topic = "/esc/duty",
        .norm_value_ptr = &remote_esc_norm_value,
        .out_pwm = &throttle_pwm};

    // Create and start the PWM input thread
    k_thread_create(
        &pwm_in_thread_data,
        pwm_in_stack,
        K_THREAD_STACK_SIZEOF(pwm_in_stack),
        pwm_input_thread,
        NULL, NULL, NULL,
        REMOTE_PWM_THREAD_PRIORITY,
        0,
        K_NO_WAIT);

    printf("pwm_relay: Initialization complete\n");
}
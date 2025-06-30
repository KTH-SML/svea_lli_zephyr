#include "pwm_publisher.h"
#include "../common/common.h"
#include "../pwm_actuator/pwm_actuator.h"
#include <std_msgs/msg/float32.h>
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>

// --- PWM input configuration ---
typedef enum {
    PWM_CH_STEERING = 0,
    PWM_CH_GEAR = 1,
    PWM_CH_DIFF = 2,
    PWM_CH_ESC = 3,
    PWM_INPUTS_MAX
} pwm_channel_t;

struct pwm_in_channel {
    const struct device *dev;
    uint32_t channel;
    const char *topic;
    rcl_publisher_t pub;
    std_msgs__msg__Float32 msg;
    float *norm_value_ptr;
    const struct pwm_dt_spec *out_pwm;
};

static float norm_values[PWM_INPUTS_MAX] = {0};
static struct pwm_in_channel pwm_inputs[PWM_INPUTS_MAX];

// --- Overlay mapping ---
// TIM3_CH1 (PA6): steering
// TIM3_CH2 (PA7): gear
// TIM4_CH3 (PB8): diff
// TIM4_CH4 (PB9): esc

#define PWM3_NODE DT_NODELABEL(pwm3_in)
#define PWM4_NODE DT_NODELABEL(pwm4_in)
#define PWM5_NODE DT_NODELABEL(pwm5_in)
#define PWM9_NODE DT_NODELABEL(pwm9_in)
static const struct device *const pwm3_dev = DEVICE_DT_GET(PWM3_NODE);
static const struct device *const pwm4_dev = DEVICE_DT_GET(PWM4_NODE);
static const struct device *const pwm5_dev = DEVICE_DT_GET(PWM5_NODE);
static const struct device *const pwm9_dev = DEVICE_DT_GET(PWM9_NODE);

static void pwm_cb(const struct device *dev, uint32_t chan,
                   uint32_t per_cyc, uint32_t pul_cyc,
                   int status, void *ud) {
    struct pwm_in_channel *input = (struct pwm_in_channel *)ud;
    static int32_t last_duty[PWM_INPUTS_MAX] = {-1, -1, -1, -1};
    int idx = input - pwm_inputs;

    if (status) {
        printf("[PWM IN] cb: dev=%p chan=%u error=%d\n", dev, chan, status);
        return;
    }

    uint64_t period = 0, pulse = 0;
    pwm_cycles_to_usec(dev, chan, per_cyc, &period);
    pwm_cycles_to_usec(dev, chan, pul_cyc, &pulse);
    int32_t duty = (period) ? (pulse * 1000) / period : -1;

    if (duty <= REMOTE_PWM_MIN_US || duty >= REMOTE_PWM_MAX_US) {
        return;
    }

    if (last_duty[idx] == -1 || abs(duty - last_duty[idx]) > 1) {
        printf("[PWM IN] cb: dev=%p chan=%u period=%llu pulse=%llu duty=%d\n",
               dev, chan, period, pulse, duty);

        last_duty[idx] = duty;
        float norm = 2.0f * (duty - REMOTE_PWM_MIN_US) / (REMOTE_PWM_MAX_US - REMOTE_PWM_MIN_US) - 1.0f;
        if (input->norm_value_ptr)
            *(input->norm_value_ptr) = norm;

        // Use correct min/max ns for each output
        if (override_mode && input->out_pwm) {
            uint32_t min_ns = 1000000, max_ns = 2000000; // fallback
            if (input->out_pwm == &steering_pwm) {
                min_ns = steering_pwm_min_ns;
                max_ns = steering_pwm_max_ns;
            } else if (input->out_pwm == &diff_pwm) {
                min_ns = diff_pwm_min_ns;
                max_ns = diff_pwm_max_ns;
            } else if (input->out_pwm == &gear_pwm) {
                extern const uint32_t gear_pwm_min_ns, gear_pwm_max_ns;
                min_ns = gear_pwm_min_ns;
                max_ns = gear_pwm_max_ns;
            } else if (input->out_pwm == &throttle_pwm) {
                extern const uint32_t throttle_pwm_min_ns, throttle_pwm_max_ns;
                min_ns = throttle_pwm_min_ns;
                max_ns = throttle_pwm_max_ns;
            }
            printf("Calling set_pwm_norm for topic %s, norm=%.2f, min_ns=%u, max_ns=%u\n",
                   input->topic, norm, min_ns, max_ns);
            set_pwm_norm(input->out_pwm, norm, min_ns, max_ns);
        }

        input->msg.data = norm;
        if (input->pub.impl) {
            rcl_ret_t rc = rcl_publish(&input->pub, &input->msg, NULL);
            if (rc != RCL_RET_OK) {
                printf("[PWM IN] publish error: %d\n", rc);
            } else {
                printf("[PWM IN] published norm %.2f to %s\n", norm, input->topic);
            }
        }
    }
}

// --- Thread and stack ---
K_THREAD_STACK_DEFINE(pwm_in_stack, 1024);
struct k_thread pwm_in_thread_data;

static void pwm_in_thread(void *arg1, void *arg2, void *arg3) {
    printf("[PWM IN] thread: entered\n");

    for (pwm_channel_t ch = 0; ch < PWM_INPUTS_MAX; ++ch) {
        if (!pwm_inputs[ch].dev)
            continue;
        if (!device_is_ready(pwm_inputs[ch].dev)) {
            printf("[PWM IN] device not ready for channel %u\n", pwm_inputs[ch].channel);
            continue;
        }
        printf("[PWM IN] Configuring capture on dev=%p chan=%u topic=%s\n",
               pwm_inputs[ch].dev, pwm_inputs[ch].channel, pwm_inputs[ch].topic);
        int ret = pwm_configure_capture(pwm_inputs[ch].dev, pwm_inputs[ch].channel,
                                        PWM_CAPTURE_TYPE_BOTH | PWM_CAPTURE_MODE_CONTINUOUS,
                                        pwm_cb, &pwm_inputs[ch]);
        if (ret) {
            printf("[PWM IN] capture config failed for chan=%u (%d)\n", pwm_inputs[ch].channel, ret);
            continue;
        }
        ret = pwm_enable_capture(pwm_inputs[ch].dev, pwm_inputs[ch].channel);
        if (ret) {
            printf("[PWM IN] capture enable failed for chan=%u (%d)\n", pwm_inputs[ch].channel, ret);
            continue;
        }
        printf("[PWM IN] capture enabled for chan=%u\n", pwm_inputs[ch].channel);
    }

    while (1) {
        k_sleep(K_FOREVER);
    }
}

// --- Publisher init ---
void pwm_in_publishers_init(rcl_node_t *node) {
    pwm_inputs[PWM_CH_STEERING] = (struct pwm_in_channel){
        .dev = pwm3_dev, .channel = 1, .topic = "/steering/duty", .norm_value_ptr = &remote_steering_norm_value, .out_pwm = &steering_pwm};
    pwm_inputs[PWM_CH_GEAR] = (struct pwm_in_channel){
        .dev = pwm9_dev, .channel = 1, .topic = "/gear/duty", .norm_value_ptr = &remote_gear_norm_value, .out_pwm = &gear_pwm};
    pwm_inputs[PWM_CH_DIFF] = (struct pwm_in_channel){
        .dev = pwm4_dev, .channel = 3, .topic = "/diff/duty", .norm_value_ptr = &remote_diff_norm_value, .out_pwm = &diff_pwm};
    pwm_inputs[PWM_CH_ESC] = (struct pwm_in_channel){
        .dev = pwm5_dev, .channel = 1, .topic = "/esc/duty", .norm_value_ptr = &remote_esc_norm_value, .out_pwm = &throttle_pwm};
    k_sleep(K_MSEC(100)); // Allow time for devices to be ready
    rcl_ret_t rc;
    for (int i = 0; i < PWM_INPUTS_MAX; ++i) {
        rc = rclc_publisher_init_best_effort(
            &pwm_inputs[i].pub, node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            pwm_inputs[i].topic);
        if (rc != RCL_RET_OK) {
            printf("[PWM IN] publisher init failed for %s: %d\n", pwm_inputs[i].topic, rc);
        } else {
            printf("[PWM IN] publisher initialized for %s\n", pwm_inputs[i].topic);
        }
        k_sleep(K_MSEC(10)); // Small delay to allow time for publisher to be ready
    }

    k_thread_create(&pwm_in_thread_data, pwm_in_stack, 1024,
                    pwm_in_thread, NULL, NULL, NULL,
                    REMOTE_PWM_THREAD_PRIORITY, 0, K_NO_WAIT);
    printf("[PWM IN] thread started\n");
}
#include "servo_subscriber.h"
#include <std_msgs/msg/float32.h>
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
BUILD_ASSERT(DT_NODE_HAS_COMPAT(SERVO_NODE, pwm_servo), "servo DT node wrong");

static const struct pwm_dt_spec pwm = PWM_DT_SPEC_GET(SERVO_NODE);
static const uint32_t min_ns = DT_PROP(SERVO_NODE, min_pulse);
static const uint32_t max_ns = DT_PROP(SERVO_NODE, max_pulse);

static rcl_subscription_t sub;
static SERVO_MSGTYPE msg;

float prevNorm = 0.0f;

static void ros_cb(const void *msg_in) {
    const SERVO_MSGTYPE *m = msg_in;
    float norm = m->data;

    if (norm == prevNorm) {
        // printf("servo_subscriber: Norm unchanged: %f\n", norm);
        return;
    }
    prevNorm = norm;
    printf("servo_subscriber: Received updated msg: %f\n", norm);
    if (norm > 1.0f) {
        norm = 1.0f;
        printf("servo_subscriber: Clamped norm to 1.0\n");
    }
    if (norm < -1.0f) {
        norm = -1.0f;
        printf("servo_subscriber: Clamped norm to -1.0\n");
    }
    uint32_t pulse = (uint32_t)((norm + 1.f) * 0.5f * (max_ns - min_ns) + min_ns);
    printf("servo_subscriber: Calculated pulse: %u ns (min: %u, max: %u)\n", pulse, min_ns, max_ns);

    int ret = pwm_set_pulse_dt(&pwm, pulse);
    if (ret) {
        printf("servo_subscriber: PWM set failed: %d\n", ret);
    } else {
        printf("servo_subscriber: PWM set OK\n");
    }
}

void steering_servo_subscriber_init(rcl_node_t *node, rclc_executor_t *exec) {
    printf("servo_subscriber: Initializing steering servo subscriber...\n");
    if (!pwm_is_ready_dt(&pwm)) {
        printf("servo_subscriber: PWM device not ready\n");
        return;
    }
    printf("servo_subscriber: PWM device ready\n");

    rclc_subscription_init_default(
        &sub, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        SERVO_TOPIC);

    printf("servo_subscriber: Subscription initialized for topic: %s\n", SERVO_TOPIC);

    rclc_executor_add_subscription(exec, &sub, &msg,
                                   ros_cb, ON_NEW_DATA);

    printf("servo_subscriber: Subscription added to executor\n");
}

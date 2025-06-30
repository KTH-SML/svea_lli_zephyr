#include "servo_subscriber.h"
#include "../common/common.h"
#include "../pwm_actuator/pwm_actuator.h"
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>

static rcl_subscription_t sub;
static SERVO_MSGTYPE msg;

float prevNorm = 0.0f;

// --- Actuation function ---
static void steering_servo_set_norm(float norm) {
    set_pwm_norm(&steering_pwm, norm, steering_pwm_min_ns, steering_pwm_max_ns);
}

// --- ROS callback ---
static void ros_cb(const void *msg_in) {
    if (override_mode == true) {
        printf("servo_subscriber: Override mode active, ignoring incoming message.\n");
        return;
    }

    const SERVO_MSGTYPE *m = msg_in;
    float norm = m->data;

    if (norm == prevNorm) {
        return;
    }
    prevNorm = norm;
    printf("servo_subscriber: Received updated msg: %f\n", norm);

    steering_servo_set_norm(norm);
}

void steering_servo_subscriber_init(rcl_node_t *node, rclc_executor_t *exec) {
    printf("servo_subscriber: Initializing steering servo subscriber...\n");
    if (!pwm_is_ready_dt(&steering_pwm)) {
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

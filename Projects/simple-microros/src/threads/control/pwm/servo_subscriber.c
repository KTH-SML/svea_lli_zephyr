#include "pwm.h"
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/float32.h>
#include <stdio.h>

static rcl_subscription_t sub;
static SERVO_MSGTYPE msg;

float prevNorm = 0.0f;

// --- ROS callback for robotics control ---
static void ros_cb(const void *msg_in) {
    // Only accept robotics commands when override_mode is FALSE (robotics control active)
    if (override_mode == true) {
        printf("servo_subscriber: Override mode active, ignoring robotics command.\n");
        return;
    }

    const SERVO_MSGTYPE *m = msg_in;
    float norm = m->data;

    if (norm == prevNorm) {
        return;
    }
    prevNorm = norm;
    printf("servo_subscriber: Received robotics command: %.2f\n", (double)norm);

    // Direct actuation for robotics control
    set_pwm_norm(&steering_pwm, norm, DT_PROP(DT_NODELABEL(steeringservo), min_pulse), DT_PROP(DT_NODELABEL(steeringservo), max_pulse));
}

void steering_servo_subscriber_init(rcl_node_t *node, rclc_executor_t *exec) {
    printf("servo_subscriber: Initializing robotics servo subscriber...\n");

    if (!node || !exec) {
        printf("servo_subscriber: Error - node or executor is NULL\n");
        return;
    }

    if (!pwm_is_ready_dt(&steering_pwm)) {
        printf("servo_subscriber: PWM device not ready\n");
        return;
    }
    printf("servo_subscriber: PWM device ready\n");

    rcl_ret_t rc = rclc_subscription_init_default(
        &sub, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        SERVO_TOPIC);

    if (rc != RCL_RET_OK) {
        printf("servo_subscriber: Subscription initialization failed: %d\n", rc);
        return;
    }

    printf("servo_subscriber: Robotics subscription initialized for topic: %s\n", SERVO_TOPIC);

    rc = rclc_executor_add_subscription(exec, &sub, &msg,
                                        ros_cb, ON_NEW_DATA);

    if (rc != RCL_RET_OK) {
        printf("servo_subscriber: Failed to add subscription to executor: %d\n", rc);
        // Clean up the subscription on failure
        rcl_ret_t cleanup_rc = rcl_subscription_fini(&sub, node);
        if (cleanup_rc != RCL_RET_OK) {
            printf("servo_subscriber: Failed to cleanup subscription: %d\n", cleanup_rc);
        }
        return;
    }

    printf("servo_subscriber: Robotics subscription added to executor\n");
}

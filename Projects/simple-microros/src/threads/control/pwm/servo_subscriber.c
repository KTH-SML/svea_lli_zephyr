#include "pwm.h"
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/float32.h>
#include <stdio.h>

// --- Global servo channel array ---
struct servo_channel servo_outputs[SERVO_OUTPUTS_MAX];

// --- ROS callback for robotics control ---
static void servo_callback(const void *msg_in, void *context) {
    struct servo_channel *servo = (struct servo_channel *)context;
    
    // Only accept robotics commands when override_mode is FALSE (robotics control active)
    if (override_mode == true) {
        printf("servo_subscriber: Override mode active, ignoring command for %s\n", servo->topic);
        return;
    }

    const SERVO_MSGTYPE *m = msg_in;
    float norm = m->data;

    // Skip if value hasn't changed significantly
    if (fabsf(norm - servo->prev_norm) < 0.01f) {
        return;
    }
    
    servo->prev_norm = norm;
    printf("servo_subscriber: Received command %.2f for %s\n", (double)norm, servo->topic);

    // Clamp norm value to valid range
    if (norm > 1.0f) norm = 1.0f;
    if (norm < -1.0f) norm = -1.0f;

    // Direct actuation for robotics control
    set_pwm_norm(servo->pwm, norm, servo->min_ns, servo->max_ns);
}

void servo_subscribers_init(rcl_node_t *node, rclc_executor_t *exec) {
    printf("servo_subscriber: Initializing robotics servo subscribers...\n");

    if (!node || !exec) {
        printf("servo_subscriber: Error - node or executor is NULL\n");
        return;
    }

    // Initialize servo channel configurations with compile-time limits
    servo_outputs[SERVO_CH_STEERING] = (struct servo_channel){
        .topic = SERVO_TOPIC_STEERING,
        .pwm = &steering_pwm,
        .min_ns = DT_PROP(DT_NODELABEL(steeringservo), min_pulse),
        .max_ns = DT_PROP(DT_NODELABEL(steeringservo), max_pulse),
        .prev_norm = 0.0f
    };

    servo_outputs[SERVO_CH_GEAR] = (struct servo_channel){
        .topic = SERVO_TOPIC_GEAR,
        .pwm = &gear_pwm,
        .min_ns = DT_PROP(DT_NODELABEL(gearservo), min_pulse),
        .max_ns = DT_PROP(DT_NODELABEL(gearservo), max_pulse),
        .prev_norm = 0.0f
    };

    servo_outputs[SERVO_CH_DIFF] = (struct servo_channel){
        .topic = SERVO_TOPIC_DIFF,
        .pwm = &diff_pwm,
        .min_ns = DT_PROP(DT_NODELABEL(diffservo), min_pulse),
        .max_ns = DT_PROP(DT_NODELABEL(diffservo), max_pulse),
        .prev_norm = 0.0f
    };

    servo_outputs[SERVO_CH_THROTTLE] = (struct servo_channel){
        .topic = SERVO_TOPIC_THROTTLE,
        .pwm = &throttle_pwm,
        .min_ns = DT_PROP(DT_NODELABEL(throttleesc), min_pulse),
        .max_ns = DT_PROP(DT_NODELABEL(throttleesc), max_pulse),
        .prev_norm = 0.0f
    };

    // Initialize subscriptions for each servo channel
    for (servo_channel_t ch = 0; ch < SERVO_OUTPUTS_MAX; ++ch) {
        struct servo_channel *servo = &servo_outputs[ch];

        // Check if PWM device is ready
        if (!pwm_is_ready_dt(servo->pwm)) {
            printf("servo_subscriber: PWM device not ready for %s\n", servo->topic);
            continue;
        }

        printf("servo_subscriber: PWM device ready for %s\n", servo->topic);

        // Initialize subscription
        rcl_ret_t rc = rclc_subscription_init_default(
            &servo->sub, node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            servo->topic);

        if (rc != RCL_RET_OK) {
            printf("servo_subscriber: Subscription init failed for %s: %d\n", servo->topic, rc);
            continue;
        }

        printf("servo_subscriber: Subscription initialized for %s\n", servo->topic);

        // Add subscription to executor with context pointer
        rc = rclc_executor_add_subscription_with_context(
            exec, &servo->sub, &servo->msg, servo_callback, servo, ON_NEW_DATA);

        if (rc != RCL_RET_OK) {
            printf("servo_subscriber: Failed to add subscription to executor for %s: %d\n", servo->topic, rc);
            
            // Clean up the subscription on failure
            rcl_ret_t cleanup_rc = rcl_subscription_fini(&servo->sub, node);
            if (cleanup_rc != RCL_RET_OK) {
                printf("servo_subscriber: Failed to cleanup subscription for %s: %d\n", servo->topic, cleanup_rc);
            }
            continue;
        }

        printf("servo_subscriber: Subscription added to executor for %s\n", servo->topic);
    }

    printf("servo_subscriber: All servo subscribers initialized\n");
}

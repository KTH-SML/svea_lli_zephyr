#include "pwm.h"
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/float32.h>
#include <stdio.h>
#include <zephyr/kernel.h>

void pwm_in_publishers_init(rcl_node_t *node) {
    if (!node) {
        printf("pwm_publisher: Error: Node is NULL\n");
        return;
    }

    printf("pwm_publisher: Initializing ROS publishers for RC values...\n");

    // Small delay to ensure devices are ready
    k_sleep(K_MSEC(100));

    // Initialize publishers for each PWM input channel
    for (int i = 0; i < PWM_INPUTS_MAX; ++i) {
        if (!pwm_inputs[i].topic) {
            printf("pwm_publisher: Skipping channel %d: no topic defined\n", i);
            continue;
        }

        rcl_ret_t rc = rclc_publisher_init_best_effort(
            &pwm_inputs[i].pub,
            node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            pwm_inputs[i].topic);

        if (rc != RCL_RET_OK) {
            printf("pwm_publisher: Publisher init failed for %s: %d\n",
                   pwm_inputs[i].topic, rc);
        } else {
            printf("pwm_publisher: Publisher initialized for %s\n",
                   pwm_inputs[i].topic);
        }

        // Small delay between publisher initializations
        k_sleep(K_MSEC(10));
    }

    printf("pwm_publisher: RC publisher initialization complete\n");
}

void publish_rc_message(struct pwm_in_channel *input, float norm_value) {
    input->msg.data = norm_value;

    if (!input->pub.impl) {
        return; // Publisher not initialized
    }

    rcl_ret_t rc = rcl_publish(&input->pub, &input->msg, NULL);
    if (rc != RCL_RET_OK) {
        printf("pwm_publisher: Publish failed for %s: error %d\n", input->topic, rc);
    } else {
        // printf("pwm_publisher: Published RC %.2f to %s\n", norm_value, input->topic);
    }
}
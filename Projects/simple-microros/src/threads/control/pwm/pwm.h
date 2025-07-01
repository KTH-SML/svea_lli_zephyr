#pragma once

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/float32.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>

// --- PWM Channel Definitions ---
typedef enum {
    PWM_CH_STEERING = 0,
    PWM_CH_GEAR = 1,
    PWM_CH_DIFF = 2,
    PWM_CH_ESC = 3,
    PWM_INPUTS_MAX
} pwm_channel_t;

// --- PWM Input Channel Structure ---
struct pwm_in_channel {
    const struct device *dev;          // PWM device
    uint32_t channel;                  // PWM channel number
    const char *topic;                 // ROS topic name
    rcl_publisher_t pub;               // ROS publisher
    std_msgs__msg__Float32 msg;        // ROS message
    float *norm_value_ptr;             // Pointer to normalized value storage
    const struct pwm_dt_spec *out_pwm; // Output PWM for relay mode
};

// --- Global Variables ---
extern struct pwm_in_channel pwm_inputs[PWM_INPUTS_MAX];
extern bool override_mode;

// --- PWM Signal Limits (microseconds) ---
#define REMOTE_PWM_MIN_US 50
#define REMOTE_PWM_MAX_US 250
#define REMOTE_PWM_THREAD_PRIORITY 5

// --- Device References ---
#define PWM3_NODE DT_NODELABEL(pwm3_in)
#define PWM4_NODE DT_NODELABEL(pwm4_in)
#define PWM5_NODE DT_NODELABEL(pwm5_in)
#define PWM9_NODE DT_NODELABEL(pwm9_in)

extern const struct device *const pwm3_dev;
extern const struct device *const pwm4_dev;
extern const struct device *const pwm5_dev;
extern const struct device *const pwm9_dev;

// --- External Variables ---
extern float remote_steering_norm_value;
extern float remote_gear_norm_value;
extern float remote_diff_norm_value;
extern float remote_esc_norm_value;

extern const struct pwm_dt_spec steering_pwm;
extern const struct pwm_dt_spec gear_pwm;
extern const struct pwm_dt_spec diff_pwm;
extern const struct pwm_dt_spec throttle_pwm;

// --- Function Declarations ---
void pwm_in_init(void);
void pwm_in_publishers_init(rcl_node_t *node);
void publish_rc_message(struct pwm_in_channel *input, float norm_value);

#define SERVO_THREAD_STACK_SIZE 1024
#define SERVO_THREAD_PRIORITY 4

#define SERVO_TOPIC "/lli/servo/steering"
#define SERVO_MSGTYPE std_msgs__msg__Float32

void steering_servo_subscriber_init(rcl_node_t *node, rclc_executor_t *exec);

void set_pwm_norm(const struct pwm_dt_spec *pwm, float norm, uint32_t min_ns, uint32_t max_ns);
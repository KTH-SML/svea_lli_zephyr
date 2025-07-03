#pragma once

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/float32.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>

/* ---------------------------------------------------------------------
 *  PWM Channel IDs
 * ------------------------------------------------------------------ */
typedef enum {
    PWM_CH_STEERING = 0,
    PWM_CH_GEAR = 1,
    PWM_CH_DIFF = 2,
    PWM_CH_ESC = 3,
    PWM_INPUTS_MAX
} pwm_channel_t;

/* ---------------------------------------------------------------------
 *  Servo Channel IDs
 * ------------------------------------------------------------------ */
typedef enum {
    SERVO_CH_STEERING = 0,
    SERVO_CH_GEAR = 1,
    SERVO_CH_DIFF = 2,
    SERVO_CH_THROTTLE = 3,
    SERVO_OUTPUTS_MAX
} servo_channel_t;

/* ---------------------------------------------------------------------
 *  PWM Input channel descriptor
 * ------------------------------------------------------------------ */
struct pwm_in_channel {
    const struct device *dev;   /* PWM device */
    uint32_t channel;           /* PWM channel number */
    const char *topic;          /* ROS topic name */
    rcl_publisher_t pub;        /* ROS publisher */
    std_msgs__msg__Float32 msg; /* ROS message */
    float *norm_value_ptr;      /* Ptr to current value */

    /* RC-passthrough (override) */
    const struct pwm_dt_spec *out_pwm; /* Output PWM */
    uint32_t out_min_ns;               /* Compile-time limits */
    uint32_t out_max_ns;
};

/* ---------------------------------------------------------------------
 *  Servo output descriptor
 * ------------------------------------------------------------------ */
struct servo_channel {
    const char *topic;             /* ROS topic name */
    rcl_subscription_t sub;        /* ROS subscription */
    std_msgs__msg__Float32 msg;    /* ROS message */
    const struct pwm_dt_spec *pwm; /* PWM spec */
    uint32_t min_ns;               /* Compile-time limits */
    uint32_t max_ns;
    float prev_norm; /* Last commanded value */
};

/* ---------------------------------------------------------------------
 *  Global objects — defined in *.c
 * ------------------------------------------------------------------ */
extern struct pwm_in_channel pwm_inputs[PWM_INPUTS_MAX];
extern struct servo_channel servo_outputs[SERVO_OUTPUTS_MAX];
extern bool override_mode;
extern bool rc_remote_disconnected;

/* ---------------------------------------------------------------------
 *  RC PWM limits (µs) — 1000 µs ↔ -1, 2000 µs ↔ +1
 * ------------------------------------------------------------------ */
#define REMOTE_PWM_MIN_US 1000u     /* –1.0 in normalised space */
#define REMOTE_PWM_MAX_US 2000u     /* +1.0 in normalised space */
#define REMOTE_PWM_CLIP_MIN_US 900u /* safety margins */
#define REMOTE_PWM_CLIP_MAX_US 2100u
#define REMOTE_PWM_THREAD_PRIORITY 4

/* ---------------------------------------------------------------------
 *  Device tree nodes (inputs)
 * ------------------------------------------------------------------ */
#define PWM3_NODE DT_NODELABEL(pwm3_in)
#define PWM4_NODE DT_NODELABEL(pwm4_in)
#define PWM5_NODE DT_NODELABEL(pwm5_in)
#define PWM9_NODE DT_NODELABEL(pwm9_in)

/* ---------------------------------------------------------------------
 *  External DT-specs (outputs)
 * ------------------------------------------------------------------ */
extern const struct pwm_dt_spec steering_pwm;
extern const struct pwm_dt_spec gear_pwm;
extern const struct pwm_dt_spec diff_pwm;
extern const struct pwm_dt_spec throttle_pwm;

/* ---------------------------------------------------------------------
 *  Function prototypes
 * ------------------------------------------------------------------ */
void pwm_in_init(void);
void pwm_in_publishers_init(rcl_node_t *node);
void publish_rc_message(struct pwm_in_channel *input, float norm_value);

void servo_subscribers_init(rcl_node_t *node, rclc_executor_t *exec);

/* Generic helpers */
void set_pwm_norm(const struct pwm_dt_spec *pwm,
                  float norm,
                  uint32_t min_ns,
                  uint32_t max_ns);

void set_pwm_pulse_us(const struct pwm_dt_spec *pwm,
                      uint32_t pulse_us); /* direct pulse pass-through */

/* ---------------------------------------------------------------------
 *  Servo control ROS topics
 * ------------------------------------------------------------------ */
#define SERVO_TOPIC_STEERING "/lli/ctrl/steering"
#define SERVO_TOPIC_GEAR "/lli/ctrl/gear"
#define SERVO_TOPIC_DIFF "/lli/ctrl/diff"
#define SERVO_TOPIC_THROTTLE "/lli/ctrl/throttle"
#define SERVO_MSGTYPE std_msgs__msg__Float32

/* LED control for debug indicators */
typedef bool (*led_condition_fn)(void *data);

struct led_control {
    const struct gpio_dt_spec *led; // LED GPIO spec
    led_condition_fn condition;     // Function that returns whether LED should be on/blinking
    void *condition_data;           // Data passed to condition function
    bool blink;                     // Whether to blink or stay solid when condition is true
    uint32_t blink_on_ms;           // Time LED stays on during blink
    uint32_t blink_off_ms;          // Time LED stays off during blink
    bool current_state;             // Current LED state (internal use)
};

void led_control_init(void);
void led_register(struct led_control *ctrl);
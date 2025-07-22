/*  wheel_enc_autotune.c  ───────────────────────────────────────────────
 *  Counts both edges, derives all timing from wheel + speed limits.
 *  ‑ MAX_SPEED sets the debounce filter
 *  ‑ MIN_SPEED sets the rolling‑window length
 *  Buffer length adapts so it never overflows at MAX_SPEED.
 *  Copy into src/wheel_enc.c
 */

#include "control.h"
#include <math.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(wheel_enc, LOG_LEVEL_INF);

/* ── user‑visible wheel + vehicle parameters ────────────────────────── */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define TICKS_PER_REV 80.0f /* encoder spec                       */
#define WHEEL_DIAM 0.115f   /* metres                             */
#define MAX_SPEED 3.0f      /* m/s  (top speed)                   */
#define MIN_SPEED 0.10f     /* m/s  (lowest speed to measure well)*/

#define WHEELBASE 0.32f

#define CALIBRATION_FACTOR 0.93f /* factor to scale ticks to m/s     */
/* ── compile‑time geometry helpers ------------------------------------ */
#define WHEEL_CIRC (M_PI * WHEEL_DIAM)             /* m / revolution    */
#define DIST_PER_TICK (WHEEL_CIRC / TICKS_PER_REV) /* m / edge          */

/* ── debounce: 80 % of the shortest pulse at MAX_SPEED --------------- */
#define DEBOUNCE_US ((uint32_t)(DIST_PER_TICK / MAX_SPEED * 1e6f * 0.8f + 0.5f))

/* ── rolling‑window: long enough to hold 2 ticks at MIN_SPEED --------- */
#define WINDOW_MS_F (2.0f * DIST_PER_TICK / MIN_SPEED * 1000.0f)
static const uint32_t WINDOW_MS = (uint32_t)(WINDOW_MS_F + 0.5f);

/* ── buffer size: ticks that fit into that window at MAX_SPEED (+4) --- */
#define TICKS_IN_WIN_MAX_F (MAX_SPEED / DIST_PER_TICK * (WINDOW_MS_F / 1000.0f))
#define MAX_TICKS ((uint16_t)(TICKS_IN_WIN_MAX_F + 4.5f))

/* ── Devicetree aliases ------------------------------------------------ */
#define WL_NODE DT_ALIAS(wheel_left_gpio)
#define WR_NODE DT_ALIAS(wheel_right_gpio)
BUILD_ASSERT(DT_NODE_HAS_STATUS(WL_NODE, okay), "wheel_left_gpio alias missing");
BUILD_ASSERT(DT_NODE_HAS_STATUS(WR_NODE, okay), "wheel_right_gpio alias missing");

/* ── wheel context ----------------------------------------------------- */
struct wheel_ctx {
    struct gpio_dt_spec gpio;
    struct gpio_callback cb;

    uint32_t times[MAX_TICKS];
    volatile uint16_t len;

    uint32_t last_us; /* debounce filter                     */
};

static struct wheel_ctx w[2] = {
    {.gpio = GPIO_DT_SPEC_GET(WL_NODE, gpios)},
    {.gpio = GPIO_DT_SPEC_GET(WR_NODE, gpios)},
};

/* ── helper: drop old stamps ------------------------------------------ */
static inline void prune_old(struct wheel_ctx *c, uint32_t now_ms) {
    while (c->len && (now_ms - c->times[0] > WINDOW_MS)) {
        for (uint16_t i = 1; i < c->len; i++)
            c->times[i - 1] = c->times[i];
        c->len--;
    }
}

/* ── ISR: debounce → prune → append ----------------------------------- */
static inline void push_tick(struct wheel_ctx *c) {
    uint32_t now_us = k_cyc_to_us_near32(k_cycle_get_32());
    if (now_us - c->last_us < DEBOUNCE_US)
        return;
    c->last_us = now_us;

    uint32_t now_ms = k_uptime_get_32();
    prune_old(c, now_ms);

    if (c->len < MAX_TICKS)
        c->times[c->len++] = now_ms;
    else { /* shouldn’t happen          */
        for (uint16_t i = 1; i < MAX_TICKS; i++)
            c->times[i - 1] = c->times[i];
        c->times[MAX_TICKS - 1] = now_ms;
    }
}

static void wl_isr(const struct device *d, struct gpio_callback *cb,
                   uint32_t p) {
    ARG_UNUSED(d);
    ARG_UNUSED(cb);
    ARG_UNUSED(p);
    push_tick(&w[0]);
}

static void wr_isr(const struct device *d, struct gpio_callback *cb,
                   uint32_t p) {
    ARG_UNUSED(d);
    ARG_UNUSED(cb);
    ARG_UNUSED(p);
    push_tick(&w[1]);
}

/* ── API helpers ------------------------------------------------------- */
static inline uint16_t count(struct wheel_ctx *c) {
    prune_old(c, k_uptime_get_32());
    return c->len;
}

void wheel_enc_init(void) {
    for (int i = 0; i < 2; i++) {
        gpio_pin_configure_dt(&w[i].gpio, GPIO_INPUT | GPIO_PULL_UP);
        gpio_pin_interrupt_configure_dt(&w[i].gpio, GPIO_INT_EDGE_BOTH);
        gpio_init_callback(&w[i].cb, i ? wr_isr : wl_isr, BIT(w[i].gpio.pin));
        gpio_add_callback(w[i].gpio.port, &w[i].cb);
        w[i].len = 0;
    }
    wheel_enc_start_odom_thread();
}

uint16_t wheel_left_ticks(void) { return count(&w[0]); }
uint16_t wheel_right_ticks(void) { return count(&w[1]); }

/* ── micro‑ROS odometry publisher (window‑based) ----------------------- */
#include "ros_iface.h"
#include <geometry_msgs/msg/twist_with_covariance_stamped.h> // Use CovarianceStamped

#define VELOCITY_SCALE (DIST_PER_TICK) /* m per tick          */
#define FACTOR (VELOCITY_SCALE * (1000.0f / WINDOW_MS))

static geometry_msgs__msg__TwistWithCovarianceStamped odom_msg; // Change type
static struct k_thread odom_thread_data;
K_THREAD_STACK_DEFINE(odom_stack, 2048);

static void odom_thread(void *a, void *b, void *c) {
    while (!ros_initialized)
        k_sleep(K_MSEC(100));

    for (;;) {

        float vL = wheel_left_ticks() * FACTOR * CALIBRATION_FACTOR;
        float vR = wheel_right_ticks() * FACTOR * CALIBRATION_FACTOR;

        odom_msg.twist.twist.linear.x = 0.5f * (vL + vR) * (forward_guess ? 1.0f : -1.0f);
        odom_msg.twist.twist.angular.z = (vL - vR) / WHEELBASE / 2 * (forward_guess ? 1.0f : -1.0f);

        // Set covariance: 0.1 along diagonal, rest zero
        for (int i = 0; i < 36; i++)
            odom_msg.twist.covariance[i] = 0;
        odom_msg.twist.covariance[0] = 0.1;  // linear.x
        odom_msg.twist.covariance[7] = 0.1;  // linear.y
        odom_msg.twist.covariance[14] = 0.1; // linear.z
        odom_msg.twist.covariance[21] = 0.1; // angular.x
        odom_msg.twist.covariance[28] = 0.1; // angular.y
        odom_msg.twist.covariance[35] = 0.1; // angular.z

        odom_msg.header.stamp.sec = (int32_t)(ros_iface_epoch_millis() / 1000ULL);
        odom_msg.header.stamp.nanosec = (uint32_t)(ros_iface_epoch_nanos() % 1000000000ULL);

        strncpy(odom_msg.header.frame_id.data, "wheel_encoder", odom_msg.header.frame_id.capacity);
        odom_msg.header.frame_id.size = strlen("wheel_encoder");
        odom_msg.header.frame_id.capacity = odom_msg.header.frame_id.size + 1;

        rcl_publish(&encoders_pub, &odom_msg, NULL);
        k_sleep(K_MSEC(5));
    }
}

void wheel_enc_start_odom_thread(void) {
    k_thread_create(&odom_thread_data, odom_stack,
                    K_THREAD_STACK_SIZEOF(odom_stack),
                    odom_thread, NULL, NULL, NULL,
                    6, 0, K_NO_WAIT);
}

/*  wheel_enc.c  – minimal dual‑wheel encoder for Zephyr
 *  - counts both edges
 *  - 500 ms rolling window
 *  - debounce time derived from wheel geometry & max speed
 *  - *NO* duplicate LOG macros
 */

#include <math.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

/* ── logging (one time only!) ─────────────────────────────────────────── */
// LOG_MODULE_REGISTER(wheel_enc, LOG_LEVEL_INF);

/* ── Devicetree aliases ──────────────────────────────────────────────── */
#define WL_NODE DT_ALIAS(wheel_left_gpio)
#define WR_NODE DT_ALIAS(wheel_right_gpio)
BUILD_ASSERT(DT_NODE_HAS_STATUS(WL_NODE, okay), "wheel_left_gpio alias missing");
BUILD_ASSERT(DT_NODE_HAS_STATUS(WR_NODE, okay), "wheel_right_gpio alias missing");

/* ── wheel / vehicle constants (same as your Arduino sketch) ─────────── */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define TICKS_PER_REV 80.0f
#define WHEEL_DIAM 0.115f /* [m] */
#define MAX_SPEED 3.0f    /* [m/s] */

#define WHEEL_CIRC (M_PI * WHEEL_DIAM)
#define MIN_PULSE_S ((WHEEL_CIRC / TICKS_PER_REV) / MAX_SPEED)
#define DEBOUNCE_US ((uint32_t)(MIN_PULSE_S * 1e6f * 0.8f)) // max speed 80%

/* ── rolling‑window settings ─────────────────────────────────────────── */
#define MAX_TICKS 128u /* array length                            */
#define WINDOW_MS 250u /* tick lifetime                           */

/* ── per‑wheel context ───────────────────────────────────────────────── */
struct wheel_ctx {
    struct gpio_dt_spec gpio;
    struct gpio_callback cb;

    uint32_t times[MAX_TICKS]; /* ms timestamps                       */
    volatile uint8_t len;      /* valid entries                       */

    uint32_t last_us; /* debounce helper                     */
};

static struct wheel_ctx w[2] = {
    {.gpio = GPIO_DT_SPEC_GET(WL_NODE, gpios)},
    {.gpio = GPIO_DT_SPEC_GET(WR_NODE, gpios)},
};

/* ── helper: drop obsolete timestamps ─────────────────────────────────── */
static void prune_old(struct wheel_ctx *c, uint32_t now_ms) {
    while (c->len && (now_ms - c->times[0] > WINDOW_MS)) {
        for (uint8_t i = 1; i < c->len; i++)
            c->times[i - 1] = c->times[i]; /* shift left */
        c->len--;
    }
}

/* ── ISR: debounce → prune → append ──────────────────────────────────── */
static inline void push_tick(struct wheel_ctx *c) {
    uint32_t now_us = k_cyc_to_us_near32(k_cycle_get_32());
    if (now_us - c->last_us < DEBOUNCE_US)
        return;
    c->last_us = now_us;

    uint32_t now_ms = k_uptime_get_32();
    prune_old(c, now_ms);

    if (c->len < MAX_TICKS)
        c->times[c->len++] = now_ms;
    else {
        for (uint8_t i = 1; i < MAX_TICKS; i++)
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

/* ── public helpers ──────────────────────────────────────────────────── */
static uint16_t count(struct wheel_ctx *c) {
    uint32_t now_ms = k_uptime_get_32();
    prune_old(c, now_ms);
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

/* ───── Optional: micro‑ROS odometry publisher (Twist) ─────────────────── */
#include "ros_iface.h"
#include <geometry_msgs/msg/twist.h>

#define M_PI 3.14159265358979323846f
#define WHEEL_DIAM 0.115f
#define TICKS_PER_REV 80.0f
#define WHEEL_CIRC (M_PI * WHEEL_DIAM)
#define VELOCITY_SCALE (WHEEL_CIRC / TICKS_PER_REV)
#define ODOM_PUB_MS 50u

static geometry_msgs__msg__Twist odom_msg;
static struct k_thread odom_thread_data;
K_THREAD_STACK_DEFINE(odom_stack, 1024);

static void odom_thread(void *a, void *b, void *c) {
    while (!ros_initialized)
        k_sleep(K_MSEC(100));

    uint32_t last_L = 0, last_R = 0, last_t = k_uptime_get_32();

    for (;;) {
        k_sleep(K_MSEC(ODOM_PUB_MS));
        uint32_t now = k_uptime_get_32();

        /* get ticks already averaged over WINDOW_MS (= 50 ms) */
        uint16_t L = wheel_left_ticks();
        uint16_t R = wheel_right_ticks();

        /* ① convert to m/s directly — no last_L / last_R / dt */
        const float FACTOR = VELOCITY_SCALE * (1000.0f / WINDOW_MS); /* 20.0 for 50 ms */
        float vL = L * FACTOR;
        float vR = R * FACTOR;

        odom_msg.linear.x = 0.5f * (vL + vR);
        odom_msg.angular.z = (vR - vL);

        rcl_publish(&encoders_pub, &odom_msg, NULL);
    }
}

void wheel_enc_start_odom_thread(void) {
    k_thread_create(&odom_thread_data, odom_stack,
                    K_THREAD_STACK_SIZEOF(odom_stack),
                    odom_thread, NULL, NULL, NULL,
                    5 /*prio*/, 0, K_NO_WAIT);
}

/* ───── Logging (disabled by default) ──────────────────────────────────── */
LOG_MODULE_REGISTER(wheel_enc, LOG_LEVEL_INF);

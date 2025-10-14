/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * SVEA Note.
 *
 * Part of the SVEA Low‑Level Interface (Zephyr) application.
 * Author: Nils Kiefer
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "control.h"
#include "ros_iface.h"

#include <math.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(wheel_enc, LOG_LEVEL_INF);

/* ── user-visible wheel & vehicle parameters ─────────────────────────── */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define TICKS_PER_REV 80.0f /* encoder gives this many edges / rev   */
#define WHEEL_DIAM 0.115f   /* metres                                */
#define MAX_SPEED 3.0f      /* m s⁻¹  (top speed)                    */
#define MIN_SPEED 0.10f     /* m s⁻¹  (lowest speed to measure) helps reduce jitter if the wheel stops at the edge of a "tooth"     */

#define WHEELBASE 0.32f   /* metres – centre-to-centre of wheels   */
#define SPEED_SCALE 0.93f /* empirical scale from ideal to real    */

/* ── derived geometry -------------------------------------------------- */
#define WHEEL_CIRC (M_PI * WHEEL_DIAM)             /* m / rev  */
#define DIST_PER_TICK (WHEEL_CIRC / TICKS_PER_REV) /* m / edge */

/* ── timing ▸ debounce: 80 % of shortest pulse at MAX_SPEED ----------- */
#define DEBOUNCE_US ((uint32_t)(DIST_PER_TICK / MAX_SPEED * 1e6f * 0.8f + 0.5f))

/* ── timing ▸ rolling window: 2 ticks at MIN_SPEED -------------------- */
#define WINDOW_MS_F (2.0f * DIST_PER_TICK / MIN_SPEED * 1000.0f)
static const uint32_t WINDOW_MS = (uint32_t)(WINDOW_MS_F + 0.5f);

/* ── buffer length: how many ticks fit in that window at MAX_SPEED ---- */
#define TICKS_IN_WIN_MAX_F (MAX_SPEED / DIST_PER_TICK * (WINDOW_MS_F / 1000.0f))
#define MAX_TICKS ((uint16_t)(TICKS_IN_WIN_MAX_F + 4.5f))

/* ── Devicetree references --------------------------------------------- */
#define WHEEL_SPEC_NODE DT_PATH(zephyr_user)
BUILD_ASSERT(DT_NODE_HAS_PROP(WHEEL_SPEC_NODE, wheel_left_gpios),
             "wheel_left_gpios missing in zephyr,user");
BUILD_ASSERT(DT_NODE_HAS_PROP(WHEEL_SPEC_NODE, wheel_right_gpios),
             "wheel_right_gpios missing in zephyr,user");

/* ── wheel context ----------------------------------------------------- */
struct wheel_ctx {
    struct gpio_dt_spec gpio;
    struct gpio_callback cb;

    uint32_t times[MAX_TICKS]; /* ms timestamps (ring buffer)      */
    volatile uint16_t head;    /* index of oldest entry            */
    volatile uint16_t len;     /* number of valid entries          */

    uint32_t last_us;        /* debounce timer (wrap-safe)       */
    volatile uint16_t burst; /* ticks since last publish         */
};

static struct wheel_ctx w[2] = {
    {.gpio = GPIO_DT_SPEC_GET(WHEEL_SPEC_NODE, wheel_left_gpios)},
    {.gpio = GPIO_DT_SPEC_GET(WHEEL_SPEC_NODE, wheel_right_gpios)},
};

/* ── helpers ----------------------------------------------------------- */
static inline void rb_push(struct wheel_ctx *c, uint32_t t_ms) {
    if (c->len < MAX_TICKS) {
        uint16_t tail = (c->head + c->len) % MAX_TICKS;
        c->times[tail] = t_ms;
        c->len++;
    } else { /* should never hit thanks to sizing math      */
        c->times[c->head] = t_ms;
        c->head = (c->head + 1) % MAX_TICKS;
    }
}

static inline void prune_old(struct wheel_ctx *c, uint32_t now_ms) {
    while (c->len &&
           (uint32_t)(now_ms - c->times[c->head]) > WINDOW_MS) {
        c->head = (c->head + 1) % MAX_TICKS;
        c->len--;
    }
}

/* ── ISR: debounce + ring-buffer insert -------------------------------- */
static inline void push_tick(struct wheel_ctx *c) {
    uint32_t now_us = k_cyc_to_us_near32(k_cycle_get_32());

    /* wrap-safe subtraction */
    if ((uint32_t)(now_us - c->last_us) < DEBOUNCE_US)
        return;
    c->last_us = now_us;

    rb_push(c, k_uptime_get_32());
    c->burst++;
}

static void wl_isr(const struct device *d, struct gpio_callback *cb, uint32_t p) {
    ARG_UNUSED(d);
    ARG_UNUSED(cb);
    ARG_UNUSED(p);
    push_tick(&w[0]);
}
static void wr_isr(const struct device *d, struct gpio_callback *cb, uint32_t p) {
    ARG_UNUSED(d);
    ARG_UNUSED(cb);
    ARG_UNUSED(p);
    push_tick(&w[1]);
}

/* ── core: instantaneous speed (m s⁻¹) -------------------------------- */
static inline float speed(struct wheel_ctx *c) {
    uint32_t now_ms = k_uptime_get_32();
    prune_old(c, now_ms); /* heavy bit runs in thread ctx */

    /* atomic snapshot of buffer metadata */
    unsigned int key = irq_lock();
    uint16_t n = c->len;
    uint16_t h = c->head;
    irq_unlock(key);

    if (n < 2)
        return 0.0f; /* no motion */

    /* oldest and newest samples (safe w/ ring) */
    uint32_t t0 = c->times[h];
    uint32_t t1 = c->times[(h + n - 1) % MAX_TICKS];

    float dt = (t1 - t0) * 1e-3f; /* ms → s */
    if (dt <= 0.0f)
        return 0.0f; /* shouldn't happen */

    float v = (n - 1) * DIST_PER_TICK / dt;
    return (fabsf(v) < MIN_SPEED) ? 0.0f : v;
}

/* ── public API -------------------------------------------------------- */
float wheel_left_speed(void) { return speed(&w[0]) * SPEED_SCALE; }
float wheel_right_speed(void) { return speed(&w[1]) * SPEED_SCALE; }

/* ── init: configure GPIO + start odom thread ------------------------- */
static void odom_thread(void *, void *, void *);
static void wheel_enc_start_odom_thread(void);

void wheel_enc_init(void) {
    LOG_INF("Wheel encoders: init (L=%s/%u, R=%s/%u)",
            w[0].gpio.port->name, w[0].gpio.pin,
            w[1].gpio.port->name, w[1].gpio.pin);

    for (int i = 0; i < 2; i++) {
        gpio_pin_configure_dt(&w[i].gpio, GPIO_INPUT);
        gpio_pin_interrupt_configure_dt(&w[i].gpio, GPIO_INT_EDGE_BOTH);
        gpio_init_callback(&w[i].cb, i ? wr_isr : wl_isr, BIT(w[i].gpio.pin));
        gpio_add_callback(w[i].gpio.port, &w[i].cb);

        w[i].head = w[i].len = 0;
        w[i].last_us = 0;
        w[i].burst = 0;
    }
    wheel_enc_start_odom_thread();
}

/* ── micro-ROS odometry publisher ------------------------------------- */
#include <geometry_msgs/msg/twist_with_covariance_stamped.h>

static geometry_msgs__msg__TwistWithCovarianceStamped odom_msg;

static struct k_thread odom_thread_data;
K_THREAD_STACK_DEFINE(odom_stack, 2048);

static void odom_thread(void *a, void *b, void *c) {
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    uint32_t last_log_ms = k_uptime_get_32();

    /* Initialize message once */
    if (!geometry_msgs__msg__TwistWithCovarianceStamped__init(&odom_msg)) {
        LOG_WRN("Wheel encoders: failed to init odom_msg");
    } else {
        // diagonal covariance = 0.1, rest 0
        memset(odom_msg.twist.covariance, 0, sizeof(odom_msg.twist.covariance));
        odom_msg.twist.covariance[0] =
            odom_msg.twist.covariance[7] =
                odom_msg.twist.covariance[14] =
                    odom_msg.twist.covariance[21] =
                        odom_msg.twist.covariance[28] =
                            odom_msg.twist.covariance[35] = 0.1f;
        (void)rosidl_runtime_c__String__assign(&odom_msg.header.frame_id, "wheel_encoder");
    }

    for (;;) {
        float vL = wheel_left_speed();
        float vR = wheel_right_speed();

        // Snapshot tick bursts since last publish
        unsigned int key_b = irq_lock();
        uint16_t ticksL = w[0].burst;
        uint16_t ticksR = w[1].burst;
        w[0].burst = 0;
        w[1].burst = 0;
        irq_unlock(key_b);

        bool fwd = 1; // assume forward, done on ros side instead

        odom_msg.twist.twist.linear.x = 0.5f * (vL + vR) * (fwd ? 1.f : -1.f);
        odom_msg.twist.twist.linear.y = (float)ticksL; // publish latest tick counts
        odom_msg.twist.twist.linear.z = (float)ticksR;
        /* 2× gain compensates empirically for skid-steer slip */
        odom_msg.twist.twist.angular.x = vL;
        odom_msg.twist.twist.angular.y = vR;

        odom_msg.twist.twist.angular.z = ((vL - vR) / WHEELBASE) *
                                         (fwd ? 1.f : -1.f) * 2.f;

        if (!ros_initialized) {
            uint32_t now = k_uptime_get_32();
            if ((uint32_t)(now - last_log_ms) >= 1000U) {
                LOG_INF("Wheel encoders waiting for ROS (vL=%.2f m/s, vR=%.2f m/s, lin.x=%.3f, ang.z=%.3f)",
                        (double)vL, (double)vR,
                        (double)odom_msg.twist.twist.linear.x,
                        (double)odom_msg.twist.twist.angular.z);
                last_log_ms = now;
            }
            k_sleep(K_MSEC(50));
            continue;
        }

        uint64_t ms = ros_iface_epoch_millis();
        odom_msg.header.stamp.sec = (int32_t)(ms / 1000ULL);
        odom_msg.header.stamp.nanosec = (uint32_t)(ros_iface_epoch_nanos() % 1000000000ULL);

        (void)ros_publish_try(&encoders_pub, &odom_msg);
        k_sleep(K_MSEC(20));
    }
}

static void wheel_enc_start_odom_thread(void) {
    k_thread_create(&odom_thread_data, odom_stack,
                    K_THREAD_STACK_SIZEOF(odom_stack),
                    odom_thread, NULL, NULL, NULL,
                    6 /* prio */, 0, K_NO_WAIT);
}

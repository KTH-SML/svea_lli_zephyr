/* dual wheel encoder – rolling-window counter ---------------------------- */
#include "wheel_enc.h"
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

/* ───────── Devicetree ───────── */
#define WHEEL_NODE(n) DT_ALIAS(n) /* aliases   */
#define WL_NODE WHEEL_NODE(wheel_left_gpio)
#define WR_NODE WHEEL_NODE(wheel_right_gpio)

BUILD_ASSERT(DT_NODE_HAS_STATUS(WL_NODE, okay), "left alias missing");
BUILD_ASSERT(DT_NODE_HAS_STATUS(WR_NODE, okay), "right alias missing");

/* read props (defaults: 500 µs debounce, 500 ms window) */
#define DEB_US(node) DT_PROP_OR(node, debounce_us, 500)
#define WIN_MS(node) DT_PROP_OR(node, window_ms, 500)

/* ───────── implementation constants ───────── */
#define BUF_SZ 256 /* power-of-2               */
#define MASK (BUF_SZ - 1)

/* one struct per wheel ---------------------------------------------------- */
struct wheel_ctx {
    struct gpio_dt_spec gpio;
    struct gpio_callback cb;
    uint32_t stamp[BUF_SZ]; /* ms timestamps                        */
    volatile uint16_t head; /* next write                           */
    volatile uint16_t tail; /* oldest valid                         */
    const uint16_t debounce_us;
    const uint16_t window_ms;
    uint32_t last_us;
};

#define WHEEL_INIT(node) {                 \
    .gpio = GPIO_DT_SPEC_GET(node, gpios), \
    .debounce_us = DEB_US(node),           \
    .window_ms = WIN_MS(node)}

static struct wheel_ctx w[2] = {WHEEL_INIT(WL_NODE), WHEEL_INIT(WR_NODE)};

/* ───────── very-light ISR: store time only ───────── */
static inline void push_tick(struct wheel_ctx *c) {
    uint32_t now_us = k_cyc_to_us_near32(k_cycle_get_32());
    if (now_us - c->last_us < c->debounce_us)
        return;
    c->last_us = now_us;
    c->stamp[c->head & MASK] = now_us / 1000; /* ms  */
    c->head++;                                /* wrap via uint16_t */
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

/* ───────── helpers ───────── */
static inline uint16_t count(struct wheel_ctx *c) {
    uint32_t now = k_uptime_get_32();
    while (c->tail != c->head &&
           now - c->stamp[c->tail & MASK] > c->window_ms)
        c->tail++;
    return c->head - c->tail;
}

/* ───────── public API ───────── */
void wheel_enc_init(void) {
    for (int i = 0; i < 2; i++) {
        gpio_pin_configure_dt(&w[i].gpio, GPIO_INPUT | GPIO_PULL_UP);
        gpio_pin_interrupt_configure_dt(&w[i].gpio,
                                        GPIO_INT_EDGE_TO_ACTIVE);
        gpio_init_callback(&w[i].cb, i ? wr_isr : wl_isr,
                           BIT(w[i].gpio.pin));
        gpio_add_callback(w[i].gpio.port, &w[i].cb);
    }
}

uint16_t wheel_left_ticks(void) { return count(&w[0]); }
uint16_t wheel_right_ticks(void) { return count(&w[1]); }

LOG_MODULE_REGISTER(wheel_enc, LOG_LEVEL_INF);

static void mon_thread(void *, void *, void *) {
    for (;;) {
        uint32_t now = k_uptime_get_32();
        LOG_INF("now: %u ms | L:%u (last %u ms)  R:%u (last %u ms)",
                now,
                wheel_left_ticks(), w[0].window_ms,
                wheel_right_ticks(), w[1].window_ms);
        k_sleep(K_SECONDS(1));
    }
}

K_THREAD_DEFINE(wheel_mon_tid, 1024, mon_thread, NULL, NULL, NULL, 7, 0, 0);

/* Dual wheel-encoder driver – GPIO interrupts with software debounce     */
#include "wheel_enc.h"
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(wheel_enc, LOG_LEVEL_INF);

/* ───────── devicetree handles ───────── */
#define WL_NODE DT_ALIAS(wheel_left_gpio)
#define WR_NODE DT_ALIAS(wheel_right_gpio)
BUILD_ASSERT(DT_NODE_HAS_STATUS(WL_NODE, okay), "wheel_left_gpio alias missing");
BUILD_ASSERT(DT_NODE_HAS_STATUS(WR_NODE, okay), "wheel_right_gpio alias missing");

/* One struct per wheel                                                    */
struct wheel_ctx {
    struct gpio_dt_spec gpio;
    struct gpio_callback cb;
    atomic_t ticks;
    uint32_t debounce_us;
    uint32_t last_irq_us;
    uint64_t start_ms;
};

static struct wheel_ctx wheels[2] = {
    {.gpio = GPIO_DT_SPEC_GET(WL_NODE, gpios),
     .debounce_us = DT_PROP(WL_NODE, debounce_us)},
    {.gpio = GPIO_DT_SPEC_GET(WR_NODE, gpios),
     .debounce_us = DT_PROP(WR_NODE, debounce_us)},
};

/* ───────── ISR helper ───────── */
static inline void wheel_isr(struct wheel_ctx *w) {
    uint32_t now = k_cyc_to_us_near32(k_cycle_get_32());
    if (now - w->last_irq_us >= w->debounce_us) {
        w->last_irq_us = now;
        atomic_inc(&w->ticks);
    }
}

/* ISR stubs – one per wheel                                               */
static void wl_isr(const struct device *d, struct gpio_callback *cb, uint32_t p) {
    ARG_UNUSED(d);
    ARG_UNUSED(cb);
    ARG_UNUSED(p);
    wheel_isr(&wheels[0]);
}

static void wr_isr(const struct device *d, struct gpio_callback *cb, uint32_t p) {
    ARG_UNUSED(d);
    ARG_UNUSED(cb);
    ARG_UNUSED(p);
    wheel_isr(&wheels[1]);
}

/* ───────── public API ───────── */
void wheel_enc_init(void) {
    for (int i = 0; i < 2; ++i) {
        const struct device *port = wheels[i].gpio.port;
        if (!device_is_ready(port)) {
            LOG_ERR("GPIO %d not ready", i);
            continue;
        }
        gpio_pin_configure_dt(&wheels[i].gpio, GPIO_INPUT | GPIO_PULL_UP);
        gpio_pin_interrupt_configure_dt(&wheels[i].gpio, GPIO_INT_EDGE_TO_ACTIVE);

        gpio_init_callback(&wheels[i].cb, (i == 0) ? wl_isr : wr_isr,
                           BIT(wheels[i].gpio.pin));
        gpio_add_callback(port, &wheels[i].cb);

        wheels[i].start_ms = k_uptime_get();
        LOG_INF("Wheel %d on %s/%d ready", i, port->name, wheels[i].gpio.pin);
    }
}

void wheel_enc_set_debounce_us(uint32_t l, uint32_t r) {
    wheels[0].debounce_us = MAX(l, 50);
    wheels[1].debounce_us = MAX(r, 50);
}

void wheel_enc_reset_left(void) {
    atomic_set(&wheels[0].ticks, 0);
    wheels[0].start_ms = k_uptime_get();
}

void wheel_enc_reset_right(void) {
    atomic_set(&wheels[1].ticks, 0);
    wheels[1].start_ms = k_uptime_get();
}

uint32_t wheel_left_ticks(void) { return atomic_get(&wheels[0].ticks); }
uint32_t wheel_right_ticks(void) { return atomic_get(&wheels[1].ticks); }

uint64_t wheel_left_start_ms(void) { return wheels[0].start_ms; }
uint64_t wheel_right_start_ms(void) { return wheels[1].start_ms; }

/* Optional monitor thread (comment out if not needed)                     */
static void mon_thread(void *, void *, void *) {
    for (;;) {
        uint64_t now = k_uptime_get();
        LOG_INF("now: %llu ms | L:%u (since %llu ms)  R:%u (since %llu ms)",
                now,
                wheel_left_ticks(), wheel_left_start_ms(),
                wheel_right_ticks(), wheel_right_start_ms());
        k_sleep(K_SECONDS(1));
    }
}
K_THREAD_DEFINE(wheel_mon_tid, 1024, mon_thread, NULL, NULL, NULL, 7, 0, 0);

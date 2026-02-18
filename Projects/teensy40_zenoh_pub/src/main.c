#include <stdio.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <zenoh-pico.h>

#define MODE "client"
#define KEYEXPR "demo/teensy40/heartbeat"
#define STARTUP_DELAY_MS 2000U
#define PUB_PERIOD_MS 1U

#if !DT_NODE_EXISTS(DT_ALIAS(zenoh_uart))
#error "Missing devicetree alias 'zenoh-uart'"
#endif
#define ZENOH_UART_NODE DT_ALIAS(zenoh_uart)

#if !DT_NODE_HAS_PROP(ZENOH_UART_NODE, current_speed)
#error "'zenoh-uart' node must define current-speed"
#endif
#define ZENOH_BAUDRATE DT_PROP(ZENOH_UART_NODE, current_speed)

static int open_session(z_owned_session_t *session, const char *locator)
{
    z_owned_config_t config;

    if (z_config_default(&config) < 0) {
        return -1;
    }
    if (zp_config_insert(z_loan_mut(config), Z_CONFIG_MODE_KEY, MODE) < 0) {
        z_drop(z_move(config));
        return -1;
    }
    if (zp_config_insert(z_loan_mut(config), Z_CONFIG_CONNECT_KEY, locator) < 0) {
        z_drop(z_move(config));
        return -1;
    }
    if (z_open(session, z_move(config), NULL) < 0) {
        return -1;
    }
    if (zp_start_read_task(z_loan_mut(*session), NULL) < 0) {
        z_drop(z_move(*session));
        return -1;
    }
    if (zp_start_lease_task(z_loan_mut(*session), NULL) < 0) {
        z_drop(z_move(*session));
        return -1;
    }
    return 0;
}

int main(void)
{
    const struct device *uart_dev = DEVICE_DT_GET(ZENOH_UART_NODE);
    const char *uart_name = DEVICE_DT_NAME(ZENOH_UART_NODE);
    z_owned_session_t session;
    z_owned_publisher_t pub;
    z_view_keyexpr_t ke;
    char locator[96];
    uint32_t count = 0;

    if (!device_is_ready(uart_dev)) {
        printk("zenoh: UART not ready: %s\n", uart_name);
        return -1;
    }

    snprintk(locator, sizeof(locator), "serial/%s#baudrate=%u", uart_name, (unsigned int)ZENOH_BAUDRATE);
    printk("fw: teensy40-zenoh-pub-min\n");
    printk("zenoh: waiting %u ms for serial to settle...\n", (unsigned int)STARTUP_DELAY_MS);
    k_sleep(K_MSEC(STARTUP_DELAY_MS));
    printk("zenoh: locator=%s\n", locator);
    printk("zenoh: keyexpr=%s\n", KEYEXPR);

    printk("z_open...\n");
    if (open_session(&session, locator) < 0) {
        printk("z_open failed\n");
        return -1;
    }
    printk("z_open ok\n");

    z_view_keyexpr_from_str_unchecked(&ke, KEYEXPR);
    if (z_declare_publisher(z_loan(session), &pub, z_loan(ke), NULL) < 0) {
        printk("z_declare_publisher failed\n");
        z_drop(z_move(session));
        return -1;
    }
    printk("declare ok\n");

    while (1) {
        char buf[64];
        z_owned_bytes_t payload;
        int len = snprintk(buf, sizeof(buf), "teensy40-count=%u", (unsigned int)count++);
        if (len < 0) {
            printk("format failed\n");
            break;
        }
        if (z_bytes_copy_from_str(&payload, buf) < 0) {
            printk("bytes alloc failed\n");
            break;
        }
        if (z_publisher_put(z_loan(pub), z_move(payload), NULL) < 0) {
            printk("put failed\n");
            break;
        }
        if ((count % 50U) == 0U) {
            printk("zenoh: published %u messages\n", (unsigned int)count);
        }
        k_sleep(K_MSEC(PUB_PERIOD_MS));
    }

    z_drop(z_move(pub));
    z_drop(z_move(session));
    return -1;
}

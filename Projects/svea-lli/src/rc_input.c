/* rc_input.c – SBUS (USART2) reader producing virtual PWM values */

#include "rc_input.h"
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(rc_input_hw, LOG_LEVEL_INF);

#define RC_STEER 0
#define RC_HIGH_GEAR 1
#define RC_THROTTLE 2
#define RC_OVERRIDE 3
#define NUM_RC_CH 4

static const struct device *uart2;
static uint16_t sbus_raw[16];
static uint32_t last_frame_ms;

static inline uint32_t map_sbus_to_us(uint16_t v) {
    /* Typical SBUS range ~172..1811 -> map to 1000..2000us */
    if (v < 172) v = 172;
    if (v > 1811) v = 1811;
    return 1000 + (uint32_t)(v - 172) * (1000) / (1811 - 172);
}

static void sbus_thread(void *, void *, void *) {
    /* Ensure UART is initialized before polling */
    while (uart2 == NULL || !device_is_ready(uart2)) {
        k_sleep(K_MSEC(10));
    }
    uint8_t buf[25];
    size_t idx = 0;
    for (;;) {
        uint8_t b;
        if (uart_poll_in(uart2, &b) == 0) {
            if (idx == 0) {
                if (b != 0x0F) continue; /* SBUS start */
            }
            buf[idx++] = b;
            if (idx == sizeof(buf)) {
                /* basic end check (flags/end often 0x00) */
                /* Unpack 16 channels of 11 bits from bytes 1..22 */
                const uint8_t *p = &buf[1];
                sbus_raw[0]  = (p[0]       | (p[1] << 8)) & 0x07FF;
                sbus_raw[1]  = ((p[1] >>3) | (p[2] <<5))  & 0x07FF;
                sbus_raw[2]  = ((p[2] >>6) | (p[3] <<2) | (p[4] <<10)) & 0x07FF;
                sbus_raw[3]  = ((p[4] >>1) | (p[5] <<7))  & 0x07FF;
                sbus_raw[4]  = ((p[5] >>4) | (p[6] <<4))  & 0x07FF;
                sbus_raw[5]  = ((p[6] >>7) | (p[7] <<1) | (p[8] <<9)) & 0x07FF;
                sbus_raw[6]  = ((p[8] >>2) | (p[9] <<6))  & 0x07FF;
                sbus_raw[7]  = ((p[9] >>5) | (p[10]<<3))  & 0x07FF;
                sbus_raw[8]  = (p[11]      | (p[12] <<8)) & 0x07FF;
                sbus_raw[9]  = ((p[12]>>3) | (p[13] <<5)) & 0x07FF;
                sbus_raw[10] = ((p[13]>>6) | (p[14] <<2) | (p[15]<<10)) & 0x07FF;
                sbus_raw[11] = ((p[15]>>1) | (p[16] <<7)) & 0x07FF;
                sbus_raw[12] = ((p[16]>>4) | (p[17] <<4)) & 0x07FF;
                sbus_raw[13] = ((p[17]>>7) | (p[18] <<1) | (p[19]<<9)) & 0x07FF;
                sbus_raw[14] = ((p[19]>>2) | (p[20] <<6)) & 0x07FF;
                sbus_raw[15] = ((p[20]>>5) | (p[21] <<3)) & 0x07FF;

                last_frame_ms = k_uptime_get_32();
                idx = 0; /* sync next frame */
            }
        } else {
            k_sleep(K_USEC(200));
        }
    }
}

/* Defer thread creation until rc_input_init(), after UART2 is ready */
static K_THREAD_STACK_DEFINE(sbus_stack, 1024);
static struct k_thread sbus_thread_data;

void rc_input_init(void) {
    uart2 = DEVICE_DT_GET(DT_NODELABEL(usart2));
    if (!device_is_ready(uart2)) {
        LOG_ERR("USART2 (SBUS) not ready");
        return;
    }
    LOG_INF("RC input: SBUS on USART2");
    k_thread_create(&sbus_thread_data, sbus_stack, K_THREAD_STACK_SIZEOF(sbus_stack),
                    sbus_thread, NULL, NULL, NULL,
                    4, 0, K_NO_WAIT);
    k_thread_name_set(&sbus_thread_data, "sbus_tid");
}

uint32_t rc_get_pulse_us(rc_channel_t ch) {
    /* Map channels: 0=steer, 1=throttle, 2=high_gear, 4=override (example) */
    switch (ch) {
    case RC_STEER:    return map_sbus_to_us(sbus_raw[0]);
    case RC_THROTTLE: return map_sbus_to_us(sbus_raw[1]);
    case RC_HIGH_GEAR: return map_sbus_to_us(sbus_raw[2]);
    case RC_OVERRIDE: return map_sbus_to_us(sbus_raw[4]);
    default: return 1500;
    }
}

uint32_t rc_get_period_us(rc_channel_t idx) {
    ARG_UNUSED(idx);
    return 0;
}

uint32_t rc_get_capture_raw(rc_channel_t ch) {
    switch (ch) {
    case RC_STEER:    return sbus_raw[0];
    case RC_THROTTLE: return sbus_raw[1];
    case RC_HIGH_GEAR: return sbus_raw[2];
    case RC_OVERRIDE: return sbus_raw[4];
    default: return 0;
    }
}

uint32_t rc_get_age_us(rc_channel_t ch) {
    ARG_UNUSED(ch);
    uint32_t now = k_uptime_get_32();
    uint32_t last = last_frame_ms;
    return (uint32_t)(now - last) * 1000U;
}

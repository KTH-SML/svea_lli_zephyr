#include "pwm.h"
#include <stdio.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#define MAX_LEDS 8
#define LED_THREAD_STACK_SIZE 512
#define LED_CHECK_INTERVAL_MS 50

static struct led_control *led_controls[MAX_LEDS];
static int led_count = 0;

K_THREAD_STACK_DEFINE(led_thread_stack, LED_THREAD_STACK_SIZE);
static struct k_thread led_thread_data;
static k_tid_t led_thread_tid;

static void led_control_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (1) {
        for (int i = 0; i < led_count; i++) {
            struct led_control *led = led_controls[i];
            bool condition_met = led->condition(led->condition_data);

            if (condition_met) {
                if (led->blink) {
                    // Toggle the LED state based on timing
                    k_timeout_t next_interval;
                    if (led->current_state) {
                        next_interval = K_MSEC(led->blink_off_ms);
                    } else {
                        next_interval = K_MSEC(led->blink_on_ms);
                    }

                    if (k_uptime_get() % (led->blink_on_ms + led->blink_off_ms) < led->blink_on_ms) {
                        if (!led->current_state) {
                            led->current_state = true;
                            gpio_pin_set_dt(led->led, 1);
                        }
                    } else {
                        if (led->current_state) {
                            led->current_state = false;
                            gpio_pin_set_dt(led->led, 0);
                        }
                    }
                } else {
                    // Solid ON
                    if (!led->current_state) {
                        led->current_state = true;
                        gpio_pin_set_dt(led->led, 1);
                    }
                }
            } else {
                // Condition not met, turn off
                if (led->current_state) {
                    led->current_state = false;
                    gpio_pin_set_dt(led->led, 0);
                }
            }
        }

        k_sleep(K_MSEC(LED_CHECK_INTERVAL_MS));
    }
}

void led_register(struct led_control *ctrl) {
    if (led_count >= MAX_LEDS) {
        printf("led_control: Too many LEDs registered\n");
        return;
    }

    if (!device_is_ready(ctrl->led->port)) {
        printf("led_control: LED device not ready\n");
        return;
    }

    gpio_pin_configure_dt(ctrl->led, GPIO_OUTPUT_INACTIVE);
    ctrl->current_state = false;

    led_controls[led_count++] = ctrl;
    printf("led_control: Registered LED %d\n", led_count - 1);
}

void led_control_init(void) {
    led_count = 0;

    led_thread_tid = k_thread_create(&led_thread_data, led_thread_stack,
                                     K_THREAD_STACK_SIZEOF(led_thread_stack),
                                     led_control_thread, NULL, NULL, NULL,
                                     8, 0, K_NO_WAIT);

    if (led_thread_tid) {
        k_thread_name_set(led_thread_tid, "led_control");
        printf("led_control: Thread started\n");
    }
}
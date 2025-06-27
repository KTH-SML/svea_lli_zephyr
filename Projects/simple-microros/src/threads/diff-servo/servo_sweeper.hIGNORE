#pragma once
#include <zephyr/kernel.h>

void servo_sweeper_thread(void *arg1, void *arg2, void *arg3);

extern struct k_thread servo_sweeper_thread_data;
extern K_THREAD_STACK_DEFINE(servo_sweeper_stack, 1024);
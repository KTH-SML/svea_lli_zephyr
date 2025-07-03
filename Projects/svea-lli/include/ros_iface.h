#pragma once
#include "remote.h"
#include <zephyr/kernel.h>

int ros_iface_start(void);
void ros_publish_rc(const struct RcFrame *f);

extern struct k_poll_signal ros_sig; /* raised on new /lli/ctrl msg */
extern volatile bool ros_cmd_ready;
extern uint8_t ros_cmd_duty[4]; /* diff, steer, gear, throttle */

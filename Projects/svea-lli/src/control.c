#include "control.h"
#include "remote.h"
#include "ros_iface.h"
#include "servo.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(control, LOG_LEVEL_INF);

#define ROS_CMD_TIMEOUT_MS 500

static void control_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    RemoteState rc_frame = {0};
    ros_command_t ros_cmd = {0};

    LOG_INF("Control thread started");

    while (1) {
        const bool rc_is_valid = remote_is_valid();
        if (!rc_is_valid) {
            turn_off_all_servos();
            k_sleep(K_MSEC(50));
            continue; // Wait for a valid RC frame
        }

        // Wait for a new RC frame to arrive
        if (k_msgq_get(&rc_q, &rc_frame, K_MSEC(100)) == 0) {
            // A new RC frame was received. Publish it.
            ros_publish_rc(&rc_frame, remote_is_valid());
        }

        const bool rc_override_active = (rc_frame.override_us > 1500);

        if (rc_override_active && rc_is_valid) {
            // Manual override: Use RC inputs
            LOG_DBG("Control source: RC Override");
            servo_request(0, rc_frame.steer);
            servo_request(1, rc_frame.high_gear_us);
            servo_request(2, rc_frame.throttle);
            // Differential lock is not typically on a 3-channel RC,
            // so we might let ROS control it or set a default.
            // For now, we let ROS control it via its last command.
            set_diff_state(ros_cmd.diff_locked);
        } else {
            // Get the latest command from ROS
            ros_get_command(&ros_cmd);
            const bool ros_cmd_is_fresh = (k_uptime_get() - ros_cmd.timestamp) < ROS_CMD_TIMEOUT_MS;
            if (ros_cmd_is_fresh && rc_is_valid) {
                // Auto mode: Use ROS commands
                LOG_DBG("Control source: ROS");
                servo_request(0, ros_cmd.steering_us);
                servo_request(1, ros_cmd.high_gear ? 2000 : 1000);
                servo_request(2, ros_cmd.throttle_us);
                set_diff_state(ros_cmd.diff_locked);
            } else {
                // Failsafe: RC is lost/not overriding, and ROS commands are stale
                // LOG_WRN("Control source: Failsafe (RC valid: %d, ROS fresh: %d)", rc_is_valid, ros_cmd_is_fresh);
                turn_off_all_servos();
            }
        }

        k_sleep(K_MSEC(1)); // Control loop runs at ~50Hz
    }
}

K_THREAD_DEFINE(control_tid, 4096, control_thread, NULL, NULL, NULL, -1, 0, 0);

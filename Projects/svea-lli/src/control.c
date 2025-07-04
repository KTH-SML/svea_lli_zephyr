#include "control.h"
#include "remote.h"
#include "ros_iface.h"
#include "servo.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(control, LOG_LEVEL_INF);

void center_all_servos(void) {
    LOG_INF("Centering all servos (failsafe)");
    servo_request(0, 1500); // Steering
    servo_request(1, 1500); // Gear
    servo_request(2, 1500); // Throttle
    servo_request(3, 1500); // Diff
}

void control_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    RemoteState rc_frame = {0};

    LOG_INF("Control thread started (priority 0)");

    while (1) {
        struct k_poll_event events[] = {
            K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
                                     K_POLL_MODE_NOTIFY_ONLY, &rc_q)};

        // Poll for RC data with timeout
        int ret = k_poll(events, ARRAY_SIZE(events), K_MSEC(100));

        // Handle RC input
        if (ret == 0 && events[0].state == K_POLL_STATE_MSGQ_DATA_AVAILABLE) {
            if (k_msgq_get(&rc_q, &rc_frame, K_NO_WAIT) == 0) {
                // Publish RC data to ROS
                ros_publish_rc(&rc_frame);
            }
            events[0].state = K_POLL_STATE_NOT_READY;
        }

        // Control logic
        bool override_active = (rc_frame.override_us > 1700);

        if (override_active && rc_valid) {
            // Use RC inputs - manual override
            LOG_DBG("Using RC control (override active)");
            servo_request(0, rc_frame.steer);
            servo_request(1, rc_frame.gear);
            servo_request(2, rc_frame.throttle);
            servo_request(3, 1500); // Center diff during RC control

            // Reset ROS command validity to prevent interference
            ros_cmd_valid = false;
        } else if (ros_cmd_valid) {
            // ROS commands are being processed by servo callbacks
            LOG_DBG("Using ROS control");
            // Commands are handled automatically by ROS callbacks
            // Just reset the flag after some time to detect timeouts
            static int64_t last_ros_cmd = 0;
            int64_t now = k_uptime_get();

            if (last_ros_cmd == 0) {
                last_ros_cmd = now;
            }

            // If no new ROS commands for 500ms, consider it timeout
            if (now - last_ros_cmd > 500) {
                ros_cmd_valid = false;
                LOG_WRN("ROS command timeout, switching to failsafe");
            } else if (ros_cmd_valid) {
                last_ros_cmd = now;
            }
        } else {
            // Failsafe: center all servos
            LOG_DBG("Failsafe mode - centering servos");
            center_all_servos();
        }

        // Brief sleep to prevent excessive CPU usage
        k_sleep(K_MSEC(10));
    }
}

void control_start(void) {
    LOG_INF("Starting control system");
}

K_THREAD_DEFINE(control_tid, 4096, control_thread, NULL, NULL, NULL, 0, 0, 0);
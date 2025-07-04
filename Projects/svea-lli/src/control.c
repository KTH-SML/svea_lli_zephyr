#include "control.h"
#include "remote.h"
#include "ros_iface.h"
#include "servo.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(control, LOG_LEVEL_INF);

// Add mutex for ros command timestamp
static K_MUTEX_DEFINE(ros_cmd_mutex);
static int64_t last_ros_cmd = 0;
static atomic_t ros_cmd_valid_atomic; // Replace global variable with atomic

static bool diff_state = true; // Default to true at startup

void set_diff_state(bool activated) {
    diff_state = activated;
    uint32_t front_us = activated ? 2000 : 1000;
    uint32_t rear_us = activated ? 1000 : 2000;
    servo_request(3, front_us); // diff front
    servo_request(4, rear_us);  // diff rear
}

void center_all_servos(void) {
    LOG_DBG("Centering all servos (failsafe)");
    servo_request(0, 0); // Steering
    servo_request(1, 0); // Gear
    servo_request(2, 0); // Throttle
    // Center diffs, then restore last known state
    set_diff_state(diff_state);
}

void control_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    RemoteState rc_frame = {0};
    int64_t last_ros_cmd = k_uptime_get(); // Initialize at thread start

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

        if (override_active && remote_is_valid()) {
            // Use RC inputs - manual override
            LOG_DBG("Using RC control (override active)");
            servo_request(0, rc_frame.steer);
            servo_request(1, rc_frame.gear);
            servo_request(2, rc_frame.throttle);
            // set_diff_state(true); // or false, depending on your logic

            // Reset ROS command validity to prevent interference
            atomic_set(&ros_cmd_valid_atomic, 0);
        } else if (atomic_get(&ros_cmd_valid_atomic)) {
            // ROS commands are being processed by servo callbacks
            LOG_DBG("Using ROS control");
            // Commands are handled automatically by ROS callbacks
            // Just reset the flag after some time to detect timeouts
            int64_t now = k_uptime_get();
            k_mutex_lock(&ros_cmd_mutex, K_FOREVER);
            if (last_ros_cmd == 0) {
                last_ros_cmd = now;
            }

            // If no new ROS commands for 500ms, consider it timeout
            if (now - last_ros_cmd > 500) {
                atomic_set(&ros_cmd_valid_atomic, 0);
                LOG_WRN("ROS command timeout, switching to failsafe");
            } else if (atomic_get(&ros_cmd_valid_atomic)) {
                last_ros_cmd = now;
            }
            k_mutex_unlock(&ros_cmd_mutex);
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
#include "remote.h"
#include "ros_iface.h"
#include "servo.h"
#include <zephyr/kernel.h>

static void control(void *a, void *b, void *c) {
    struct RcFrame rc = {0};

    struct k_poll_event ev[] = {
        K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
                                 K_POLL_MODE_NOTIFY_ONLY, &rc_q),
        K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL,
                                 K_POLL_MODE_NOTIFY_ONLY, &ros_sig)};

    for (;;) {
        k_poll(ev, 2, K_FOREVER);

        if (ev[0].state) {
            rc_wait(&rc, K_NO_WAIT);
            ros_publish_rc(&rc);
        }

        if (override_active && rc_valid) { /* 1️⃣ RC wins */
            servo_set(SERVO_STEER, rc.steer);
            servo_set(SERVO_GEAR, rc.gear);
            servo_set(SERVO_THROTTLE, rc.throttle);
        } else if (ros_cmd_ready) { /* 2️⃣ ROS next */
            uint8_t diff = ros_cmd_duty[0];
            /* mirrored diff: front = diff, rear = 200-(diff-100) */
            servo_set(SERVO_DIFF_FRONT, diff);
            servo_set(SERVO_DIFF_REAR, 200 - (diff - 100));

            servo_set(SERVO_STEER, ros_cmd_duty[1]);
            servo_set(SERVO_GEAR, ros_cmd_duty[2]);
            servo_set(SERVO_THROTTLE, ros_cmd_duty[3]);
            ros_cmd_ready = false;
        } else { /* 3️⃣ failsafe */
            servo_safe_centre();
        }

        ev[0].state = ev[1].state = K_POLL_STATE_NOT_READY;
    }
}

int control_start(void) {
    static uint8_t st[2048];
    k_thread_create(&(struct k_thread){}, st, sizeof st,
                    control, NULL, NULL, NULL, 0, 0, K_NO_WAIT);
    return 0;
}

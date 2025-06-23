#include <version.h>

#if ZEPHYR_VERSION_CODE >= ZEPHYR_VERSION(3, 1, 0)
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/posix/time.h>
#else
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <posix/time.h>
#include <zephyr.h>
#endif

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <std_msgs/msg/string.h>

#include <rclc/executor.h>
#include <rclc/rclc.h>

#include <microros_transports.h>
#include <rmw_microros/rmw_microros.h>

#define RCCHECK(fn)                                                                      \
    {                                                                                    \
        rcl_ret_t temp_rc = fn;                                                          \
        if ((temp_rc != RCL_RET_OK)) {                                                   \
            printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
            for (;;) {                                                                   \
            };                                                                           \
        }                                                                                \
    }
#define RCSOFTCHECK(fn)                                                                    \
    {                                                                                      \
        rcl_ret_t temp_rc = fn;                                                            \
        if ((temp_rc != RCL_RET_OK)) {                                                     \
            printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
        }                                                                                  \
    }

rcl_publisher_t publisher;
std_msgs__msg__String msg;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        static char str_data[] = "px4 is for simps";
        msg.data.data = str_data;
        msg.data.size = strlen(str_data);
        msg.data.capacity = sizeof(str_data);
        rcl_ret_t rc = rcl_publish(&publisher, &msg, NULL);
        if (rc != RCL_RET_OK) {
            printf("[timer_callback] Publish failed: %d\n", rc);
        }
    }
}

int main(void) {
    while (1) {
        rmw_uros_set_custom_transport(
            MICRO_ROS_FRAMING_REQUIRED,
            (void *)&default_params,
            zephyr_transport_open,
            zephyr_transport_close,
            zephyr_transport_write,
            zephyr_transport_read);

        rcl_allocator_t allocator = rcl_get_default_allocator();
        rclc_support_t support;
        rcl_node_t node;
        rcl_timer_t timer;
        rclc_executor_t executor;
        rcl_ret_t rc;
        bool setup_ok = true;

        // Try to set up micro-ROS entities
        rc = rclc_support_init(&support, 0, NULL, &allocator);
        if (rc != RCL_RET_OK) {
            printf("Support init failed: %d\n", rc);
            setup_ok = false;
        }
        if (setup_ok) {
            rc = rclc_node_init_default(&node, "zephyr_string_publisher", "", &support);
            if (rc != RCL_RET_OK) {
                printf("Node init failed: %d\n", rc);
                setup_ok = false;
            }
        }
        if (setup_ok) {
            rc = rclc_publisher_init_default(
                &publisher,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                "zephyr_string_publisher");
            if (rc != RCL_RET_OK) {
                printf("Publisher init failed: %d\n", rc);
                setup_ok = false;
            }
        }
        if (setup_ok) {
            const unsigned int timer_timeout = 1000;
            rc = rclc_timer_init_default(
                &timer,
                &support,
                RCL_MS_TO_NS(timer_timeout),
                timer_callback);
            if (rc != RCL_RET_OK) {
                printf("Timer init failed: %d\n", rc);
                setup_ok = false;
            }
        }
        if (setup_ok) {
            rc = rclc_executor_init(&executor, &support.context, 1, &allocator);
            if (rc != RCL_RET_OK) {
                printf("Executor init failed: %d\n", rc);
                setup_ok = false;
            }
        }
        if (setup_ok) {
            rc = rclc_executor_add_timer(&executor, &timer);
            if (rc != RCL_RET_OK) {
                printf("Executor add timer failed: %d\n", rc);
                setup_ok = false;
            }
        }

        // Main publish loop: only spin executor, break if error
        if (setup_ok) {
            printf("micro-ROS setup complete, entering publish loop.\n");
            while (1) {
                rc = rclc_executor_spin_some(&executor, 100);
                if (rc != RCL_RET_OK) {
                    printf("Executor spin error: %d, will reset micro-ROS entities.\n", rc);
                    break; // Exit to outer loop to reset everything
                }
                usleep(100000);
            }
        } else {
            printf("micro-ROS setup failed, retrying in 1s...\n");
            k_sleep(K_MSEC(1000));
        }

        // Clean up resources before retrying
        rcl_ret_t fini_rc;
        fini_rc = rcl_publisher_fini(&publisher, &node);
        if (fini_rc != RCL_RET_OK)
            printf("publisher fini failed: %d\n", fini_rc);
        fini_rc = rcl_node_fini(&node);
        if (fini_rc != RCL_RET_OK)
            printf("node fini failed: %d\n", fini_rc);
        fini_rc = rclc_executor_fini(&executor);
        if (fini_rc != RCL_RET_OK)
            printf("executor fini failed: %d\n", fini_rc);
        fini_rc = rcl_timer_fini(&timer);
        if (fini_rc != RCL_RET_OK)
            printf("timer fini failed: %d\n", fini_rc);
        fini_rc = rclc_support_fini(&support);
        if (fini_rc != RCL_RET_OK)
            printf("support fini failed: %d\n", fini_rc);

        printf("micro-ROS entities cleaned up, restarting setup...\n");
        k_sleep(K_MSEC(1000));
    }
    return 0;
}

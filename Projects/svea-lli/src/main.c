#include "remote.h"
#include "ros_iface.h"
#include "servo.h"

extern int rc_input_init(void);
extern int control_start(void);
extern int sensor_imu_init(rcl_node_t *);

void main(void) {
    rc_input_init();   /* capture first           */
    servo_init();      /* outputs                 */
    ros_iface_start(); /* starts executor thread  */
    control_start();   /* arbiter thread (prio 0) */

    /* main idle forever – everything else in threads / callbacks     */
    for (;;)
        k_sleep(K_FOREVER);
}

#ifndef SENSORS_H
#define SENSORS_H

void sensors_init(void);
void sensors_thread(void *p1, void *p2, void *p3);

/* Quick, ROS-free IMU diagnostic.
 * Returns 0 on success (sample fetched), <0 on error.
 * Logs one line with accel [m/s^2] and gyro [rad/s]. */
int sensors_imu_quick_check(void);

#endif // SENSORS_H

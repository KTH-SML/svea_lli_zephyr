#ifndef CONTROL_H
#define CONTROL_H

#include "remote.h"
#include "ros_iface.h"
#include <zephyr/kernel.h>

void control_start(void);
void control_thread(void *p1, void *p2, void *p3);
void center_all_servos(void);

#endif // CONTROL_H
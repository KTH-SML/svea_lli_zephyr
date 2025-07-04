#ifndef SENSORS_H
#define SENSORS_H

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

void sensors_init(void);
void sensors_thread(void *p1, void *p2, void *p3);

#endif // SENSORS_H
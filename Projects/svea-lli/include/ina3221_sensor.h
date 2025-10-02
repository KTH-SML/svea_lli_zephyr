/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * SVEA Note.
 *
 * Part of the SVEA Low‑Level Interface (Zephyr) application.
 * Author: Nils Kiefer
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#ifndef INA3221_SENSOR_H_
#define INA3221_SENSOR_H_

#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

struct ina3221_measurement {
    struct sensor_value bus_voltage[3];
    struct sensor_value shunt_current[3];
    struct sensor_value power[3];
};

int ina3221_sensor_init(void);
int ina3221_sensor_get_latest(struct ina3221_measurement *sample);

#ifdef __cplusplus
}
#endif

#endif /* INA3221_SENSOR_H_ */

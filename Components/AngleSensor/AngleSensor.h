#ifndef ANGLE_SENSOR_H
#define ANGLE_SENSOR_H

/**
 * @brief Angle Sensor instance type.
 *
 * This structure provides generic API for angle sensors.
 */
typedef struct
{
    /**
     * @brief Function for initializing the angle sensor.
     */
    ret_code_t (* angle_sensor_init)(const nrfx_twi_t *m_twi);

    /**
     * @brief Function for uninitializing the LCD controller.
     */
    void (* angle_sensor_uninit)(void);

    void (* angle_sensor_sleep)();

    void (* angle_sensor_wakeup)();

    void (* angle_sensor_get_angles)();

}angle_sensor_t;

void angle_sensor_sleep(angle_sensor_t angle_sensor);
void angle_sensor_wakeup(angle_sensor_t angle_sensor);

#endif
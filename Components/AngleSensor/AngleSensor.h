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
    ret_code_t (* init)(const nrfx_twi_t *m_twi);

    /**
     * @brief Function for uninitializing the LCD controller.
     */
    void (* uninit)(void);

    void (* sleep)();

    void (* wakeup)();

    float * (* get_angles)();

    void (* set_orientation)(uint16_t orientation);

}angle_sensor_t;

ret_code_t angle_sensor_init(const nrfx_twi_t *m_twi);

void angle_sensor_log_angles(float *angles);

extern angle_sensor_t angle_sensor;

#endif
#include "Components/AngleSensor/Accelerometers/ADXL355/adxl355.h"
#include "Components/AngleSensor/Accelerometers/MPU6050/mpu6050.h"
#include "Components/AngleSensor/Accelerometers/BMI270/bmi270.h"

float mLastAnglesFromSensor[3] = {0.0, 0.0, 0.0};   // x y z axis

MPU6050 mpu6050Sensor;
ADXL355 adxl355Sensor;
BMI270 bmi270Sensor;
MAX17260 max17260Sensor;

void angle_sensor_init(const nrfx_twi_t *m_twi)
{
    // Try to find an accelerometer sensor on the TWI bus
    if (adxl355_init(&adxl355Sensor, &m_twi))
    {
       //ble_acceleration_service_init(ACCELEROMETER_ADXL355);
       NRF_LOG_INFO("ADXL355 initialised"); // if it failed to initialize then print a message
    }
    else if (mpu6050_init(&mpu6050Sensor, &m_twi))
    {
        //ble_acceleration_service_init(ACCELEROMETER_MPU6050);
        NRF_LOG_INFO("MPU6050 initialised"); // if it failed to initialize then print a message
    }
    
    else if (bmi270_init(&bmi270Sensor, &m_twi))
    {
        //ble_acceleration_service_init(ACCELEROMETER_BMI270);
        NRF_LOG_INFO("BMI270 initialised"); // if it failed to initialize then print a message
    }
}

void angle_sensor_sleep()
{

}

void angle_sensor_wakeup()
{

}
#include "Components/AngleSensor/Accelerometers/ADXL355/adxl355.h"
#include "Components/AngleSensor/Accelerometers/MPU6050/mpu6050.h"
#include "Components/AngleSensor/Accelerometers/BMI270/bmi270.h"

#include "AngleSensor.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "app_timer.h"
#include "nrfx_twi.h"
#include "nrf_gpio.h"

#include <math.h>

APP_TIMER_DEF(m_read_angle_sensor_timer_id);

#define ANGLE_SENSOR_TIMER_INTERVAL_MS              13   // 12ms
#define ANGLE_SENSOR_TIMER_INTERVAL_TICKS           APP_TIMER_TICKS(ANGLE_SENSOR_TIMER_INTERVAL_MS)

#define _RAD_TO_DEG 57.2957795131f  // Constant to convert radians to degrees
#define _PI 3.14159265359f           // Constant for the value of pi

static float mLastAnglesFromSensor[3] = {0.0, 0.0, 0.0};   // x y z axis

static MPU6050 mpu6050Sensor;
static ADXL355 adxl355Sensor;
static BMI270 bmi270Sensor;

static ret_code_t angle_sensor_bmi270_init();
static void angle_sensor_adxl355_sleep();
static void angle_sensor_adxl355_wakeup();
static void angle_sensor_bmi270_sleep();
static void angle_sensor_bmi270_wakeup();
static void angle_sensor_mpu6050_sleep();
static void angle_sensor_mpu6050_wakeup();
static float map(int32_t value, int32_t low1, int32_t high1, int32_t low2, int32_t high2);
static void calculateAnglesFromDeviceOrientation(float angleX, float angleY, float angleZ, float *angles);
static float* angle_sensor_get_angles();
static void angle_sensor_set_sensor_orientation(uint16_t orientation);

angle_sensor_t angle_sensor = 
{
    .init = angle_sensor_init,
    .get_angles = angle_sensor_get_angles,
    .set_orientation = angle_sensor_set_sensor_orientation,
};

static float xGs;
static float yGs;
static float zGs;

static int32_t xoutput = 0;
static int32_t youtput = 0;
static int32_t zoutput = 0;

static int16_t sensorOrientation = 1;

static void start_read_angle_sensor_timer()
{
    ret_code_t err_code = app_timer_start(m_read_angle_sensor_timer_id, ANGLE_SENSOR_TIMER_INTERVAL_TICKS, NULL);
    APP_ERROR_CHECK(err_code);
}

static void stop_read_angle_sensor_timer()
{
    ret_code_t err_code = app_timer_stop(m_read_angle_sensor_timer_id);
    APP_ERROR_CHECK(err_code);
}

static void read_angle_sensor_timer_timeout_handler()
{
        ret_code_t err_code;
    // Increment the value of m_custom_value before nortifing it.
    if (mpu6050Sensor.initialised)
    {
        static int16_t AccValue[3];
        if(mpu6050_ReadAcc(&mpu6050Sensor, &AccValue[0], &AccValue[1], &AccValue[2]) == true) // Read acc value from mpu6050 internal registers and save them in the array
        {
            xoutput = (int32_t)(0.9396f * xoutput + 0.0604 * (float)AccValue[0]);
            youtput = (int32_t)(0.9396f * youtput + 0.0604 * (float)AccValue[1]);
            zoutput = (int32_t)(0.9396f * zoutput + 0.0604 * (float)AccValue[2]);

            xGs = map((uint32_t)xoutput, -32768, 32767, -90, 90);
            yGs = map((uint32_t)youtput, -32768, 32767, -90, 90);
            zGs = map((uint32_t)zoutput, -32768, 32767, -90, 90);

            calculateAnglesFromDeviceOrientation(xGs, yGs, zGs, mLastAnglesFromSensor);
            
            angle_sensor_log_angles(mLastAnglesFromSensor);
        }
        else
        {
            NRF_LOG_RAW_INFO("Reading ACC values Failed!!!\n"); // if reading was unsuccessful then let the user know about it
            NRF_LOG_FLUSH();
        }

        static int16_t temperatureValue;
        if(mpu6050_ReadTemp(&mpu6050Sensor, &temperatureValue))
        {
            float scaledTemperature = ((float)temperatureValue / 340.0) + 36.53;
            //NRF_LOG_RAW_INFO("temp:" NRF_LOG_FLOAT_MARKER "\n", NRF_LOG_FLOAT(scaledTemperature) ); // display the read values

        }
    }
    else if (adxl355Sensor.initialised)
    {
        static int32_t AccValue[3];
        if(adxl355_ReadAcc(&adxl355Sensor, &AccValue[0], &AccValue[1], &AccValue[2]) == true) // Read acc value from adxl355 internal registers and save them in the array
        {
            uint16_t tempValue;
            adxl355_ReadTemp(&adxl355Sensor, &tempValue);

            float temp = -0.11049723765f * ((float)tempValue - 1852.0f) + 25.0f;

            //NRF_LOG_RAW_INFO("Temperature: " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(temp));
            //NRF_LOG_FLUSH();

            xoutput = (int32_t)(0.9396f * (float)xoutput + 0.0604 * (float)AccValue[0]);
            youtput = (int32_t)(0.9396f * (float)youtput + 0.0604 * (float)AccValue[1]);
            zoutput = (int32_t)(0.9396f * (float)zoutput + 0.0604 * (float)AccValue[2]);

            xGs = map(xoutput, -524286, 524286, -90, 90);
            yGs = map(youtput, -524286, 524286, -90, 90);
            zGs = map(zoutput, -524286, 524286, -90, 90);

            calculateAnglesFromDeviceOrientation(xGs, yGs, zGs, mLastAnglesFromSensor);
            
            //angle_sensor_log_angles(mLastAnglesFromSensor);
        }
    }
    else if (bmi270Sensor.initialised)
    {
        static int16_t AccValue[3];
        if(bmi270_ReadAcc(&bmi270Sensor, &AccValue[0], &AccValue[1], &AccValue[2]) == true) // Read acc value from mpu6050 internal registers and save them in the array
        {
            xoutput = (0.9396f * xoutput + 0.0604 * (float)AccValue[0]);
            youtput = (0.9396f * youtput + 0.0604 * (float)AccValue[1]);
            zoutput = (0.9396f * zoutput + 0.0604 * (float)AccValue[2]);

            xGs = map((uint32_t)xoutput, -32768, 32767, -90, 90);
            yGs = map((uint32_t)youtput, -32768, 32767, -90, 90);
            zGs = map((uint32_t)zoutput, -32768, 32767, -90, 90);

            calculateAnglesFromDeviceOrientation(xGs, yGs, zGs, mLastAnglesFromSensor);

            float temp;

            bmi270_ReadTemp(&bmi270Sensor, &temp );   
        }
        else
        {
            NRF_LOG_RAW_INFO("Reading ACC values Failed!!!\n"); // if reading was unsuccessful then let the user know about it
            NRF_LOG_FLUSH();
        }
    }

    start_read_angle_sensor_timer();
}


ret_code_t angle_sensor_init(const nrfx_twi_t *m_twi)
{
    NRF_LOG_INFO("Initialising angleSensor");
    // Try to find an accelerometer sensor on the TWI bus



    if (adxl355_init(&adxl355Sensor, m_twi))
    {
        angle_sensor.sleep = angle_sensor_adxl355_sleep;
        angle_sensor.wakeup = angle_sensor_adxl355_wakeup;
        NRF_LOG_INFO("ADXL355 initialised"); // if it failed to initialize then print a message
    }
    else if (mpu6050_init(&mpu6050Sensor, m_twi))
    {
        angle_sensor.sleep = angle_sensor_mpu6050_sleep;
        angle_sensor.wakeup = angle_sensor_mpu6050_wakeup;
        NRF_LOG_INFO("MPU6050 initialised"); // if it failed to initialize then print a message
    }
    else 
    if (bmi270_init(&bmi270Sensor, m_twi))
    {   
        angle_sensor.sleep = angle_sensor_bmi270_sleep;
        angle_sensor.wakeup = angle_sensor_bmi270_wakeup;
        
        NRF_LOG_INFO("BMI270 initialised"); // if it failed to initialize then print a message
        NRF_LOG_INFO("BMI270 setup complete");
    }

    ret_code_t err_code = app_timer_create(&m_read_angle_sensor_timer_id, APP_TIMER_MODE_SINGLE_SHOT, read_angle_sensor_timer_timeout_handler);

    NRF_LOG_FLUSH();
}

static void angle_sensor_adxl355_sleep()
{
    adxl355_setPowerControl(&adxl355Sensor, ADXL355_POWER_CONTROL_FLAG_STANDBY);
}

static void angle_sensor_adxl355_wakeup()
{
    adxl355_setPowerControl(&adxl355Sensor, ADXL355_POWER_CONTROL_FLAG_MEASUREMENT_MODE);
    adxl355_setFilterSettings(&adxl355Sensor, ADXL355_ODR_LPF_15_625HZ_3_906HZ);
    adxl355_setRange(&adxl355Sensor, ADXL_RANGE_2G);

    start_read_angle_sensor_timer();
}

static void angle_sensor_bmi270_sleep()
{
    bmi270_EnableAccelerometer(&bmi270Sensor, false);
}

static void angle_sensor_bmi270_wakeup()
{
    //bmi270_EnableAdvancedPowerSave(&bmi270Sensor, false);

    bmi270_EnableAccelerometer(&bmi270Sensor, true);

    uint8_t status;

    bmi270_GetInternalStatus(&bmi270Sensor, &status);

    NRF_LOG_INFO("Status %d", status);

    start_read_angle_sensor_timer();
}

static void angle_sensor_mpu6050_sleep()
{
    mpu6050_SetTemperatureDisabled(&mpu6050Sensor, true);
    mpu6050_SetSleepDisabled(&mpu6050Sensor, true);
}

static void angle_sensor_mpu6050_wakeup()
{
    mpu6050_SetSleepDisabled(&mpu6050Sensor, false);        // Enable accelerometer and gyro
    mpu6050_SetTemperatureDisabled(&mpu6050Sensor, false);  // Enable temperature sensor
    mpu6050_SetCycleEnabled(&mpu6050Sensor, false);         // Enable temperature sensor
   
    mpu6050_register_write(&mpu6050Sensor, MPU6050_SAMPLE_RATE_REG , 0x07); // Set sample rate divider to be 7. Sample rate = 1,000 / (1+7) = 125 (Same sample may be received in FIFO twice)
    mpu6050_register_write(&mpu6050Sensor, MPU6050_CONFIG_REG , 0x06);      // Configure the DLPF to 10 Hz, 13.8 ms / 10 Hz, 13.4 ms, 1 kHz
    mpu6050_EnableInterrupt(&mpu6050Sensor, DISABLE_ALL_INTERRUPTS);        // Disable Interrupts
    mpu6050_register_write(&mpu6050Sensor, MPU6050_ACCEL_CONFIG_REG, 0x04); // Configure the DHPF available in the path leading to motion detectors to 0.63Hz

    start_read_angle_sensor_timer();
}

static float* angle_sensor_get_angles()
{
    return mLastAnglesFromSensor;
}

static void angle_sensor_set_sensor_orientation(uint16_t orientation)
{
    sensorOrientation = orientation;
}

static float map(int32_t value, int32_t low1, int32_t high1, int32_t low2, int32_t high2) {
    return low2 + ((float)(high2 - low2) * (value - low1) / (high1 - low1));
}

static void calculateAnglesFromDeviceOrientation(float angleX, float angleY, float angleZ, float *angles) 
{
    switch (sensorOrientation) 
    {
        case 1:
            angles[0] = _RAD_TO_DEG * (atan2(angleZ, -angleY) + _PI);
            angles[1] = _RAD_TO_DEG * (atan2(-angleX, -angleZ) + _PI);
            angles[2] = _RAD_TO_DEG * (atan2(-angleZ, -angleX) + _PI);
            break;
        case 2:
            angles[0] = _RAD_TO_DEG * (atan2(-angleY, -angleZ) + _PI);
            angles[1] = _RAD_TO_DEG * (atan2(-angleX, angleY) + _PI);
            angles[2] = _RAD_TO_DEG * (atan2(-angleY, -angleX) + _PI);
            break;
        case 3:
            angles[0] = _RAD_TO_DEG * (atan2(-angleY, -angleX) + _PI);
            angles[1] = _RAD_TO_DEG * (atan2(angleZ, angleY) + _PI);
            angles[2] = _RAD_TO_DEG * (atan2(-angleY, angleZ) + _PI);
            break;
        case 4:
            angles[0] = _RAD_TO_DEG * (atan2(-angleY, angleX) + _PI);
            angles[1] = _RAD_TO_DEG * (atan2(-angleZ, angleY) + _PI);
            angles[2] = _RAD_TO_DEG * (atan2(-angleY, -angleZ) + _PI);
            break;
        case 5:
            angles[0] = _RAD_TO_DEG * (atan2(-angleY, angleZ) + _PI);
            angles[1] = _RAD_TO_DEG * (atan2(angleX, angleY) + _PI);
            angles[2] = _RAD_TO_DEG * (atan2(-angleY, angleX) + _PI);
            break;
        case 6:
            angles[0] = _RAD_TO_DEG * (atan2(-angleZ, angleY) + _PI);
            angles[1] = _RAD_TO_DEG * (atan2(-angleX, angleZ) + _PI);
            angles[2] = _RAD_TO_DEG * (atan2(-angleZ, -angleX) + _PI);
            break;
    }
}

void angle_sensor_log_angles(float *angles)
{
    NRF_LOG_RAW_INFO("x:" NRF_LOG_FLOAT_MARKER ", ", NRF_LOG_FLOAT(angles[0]) ); // display the read values
    NRF_LOG_RAW_INFO("y:" NRF_LOG_FLOAT_MARKER ", ", NRF_LOG_FLOAT(angles[1]) ); // display the read values
    NRF_LOG_RAW_INFO("z:" NRF_LOG_FLOAT_MARKER " ", NRF_LOG_FLOAT(angles[2]) ); // display the read values

    NRF_LOG_RAW_INFO("\n");
    NRF_LOG_FLUSH();
}
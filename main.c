#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_delay.h"
#include "nrfx_twi.h"

#include "Components/AngleSensor/AngleSensor.h"

#include "Components/FuelGauge/MAX17260/max17260.h"
#include "Components/LED/nrf_buddy_led.h"
#include "Components/Bluetooth/Bluetooth.h"
#include "Components/Bluetooth/Services/AccelerometerService.h"
#include "Components/Bluetooth/Services/BatteryService.h"
#include "Components/Bluetooth/Services/EnvironmentalService.h"
#include "Components/SavedParameters/SavedParameters.h"

APP_TIMER_DEF(m_notification_timer_id);


//Initializing TWI0 instance
#define TWI_INSTANCE_ID     0

//I2C Pins Settings, you change them to any other pins
#define TWI_SCL_M           6         //I2C SCL Pin
#define TWI_SDA_M           8        //I2C SDA Pin

#define NOTIFICATION_INTERVAL           APP_TIMER_TICKS(1000/30) // 30 Hz

// Create a Handle for the twi communication
const nrfx_twi_t m_twi = NRFX_TWI_INSTANCE(TWI_INSTANCE_ID);

static MAX17260 max17260Sensor;



void orientationValueReceivedHandler(uint16_t orientation)
{
    saved_parameters_SaveOrientation(orientation);
    angle_sensor.set_orientation(orientation);
}

void vehicleLengthValueReceivedHandler(uint16_t length)
{
    saved_parameters_SaveVehicleLength(length);
}

void vehicleWidthValueReceivedHandler(uint16_t width)
{
    saved_parameters_SaveVehicleWidth(width);
}

void levelingModeReceivedHandler(uint16_t levelingMode)
{
    saved_parameters_SaveCurrentLevelingMode(levelingMode);
}

void initialise_accelerometer()
{
    ble_accelerometer_service_leveling_mode_update(saved_parameters_getSavedCurrentLevelingMode());
    ble_accelerometer_service_orientation_update((uint8_t)saved_parameters_getSavedOrientation());

    ble_accelerometer_service_saved_hitch_angle_update(saved_parameters_getSavedHitchHeightAngle());

    ble_accelerometer_service_saved_vehicle_length_update(saved_parameters_getSavedVehicleLength());
    ble_accelerometer_service_saved_vehicle_width_update(saved_parameters_getSavedVehicleWidth());
    
    angle_sensor.wakeup();
    ret_code_t err_code = app_timer_start(m_notification_timer_id, NOTIFICATION_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}

void calibration_value_received_callback(uint8_t value)
{
    switch (value)
    {
    case 1: // Calibrate sensor
    {
        NRF_LOG_INFO("Message Received from calibration.");
        saved_parameters_SaveAngleOffsets(angle_sensor.get_angles());
        
        break;
    }

    case 2: // save the hitch height
    {
        float angleToSave = angle_sensor.get_angles()[1];
        saved_parameters_SaveHitchAngle(angleToSave); // save the y angle
        NRF_LOG_RAW_INFO("angle saved: " NRF_LOG_FLOAT_MARKER ", \n", NRF_LOG_FLOAT(angleToSave) );

        angleToSave = saved_parameters_getSavedHitchHeightAngle(); // save the y angle
        NRF_LOG_RAW_INFO("angle read back: " NRF_LOG_FLOAT_MARKER ",\n ", NRF_LOG_FLOAT(angleToSave) );

        float angleOffsets[3];
        saved_parameters_getSavedCalibrationAngles(angleOffsets);

        float hitchAngleToSendTo = angle_sensor.get_angles()[1] - angleOffsets[1];
        saved_parameters_SaveHitchAngle(hitchAngleToSendTo);
        ble_accelerometer_service_saved_hitch_angle_update(hitchAngleToSendTo);
    }
    
    default:
        break;
    }

    uint8_t resetValue = 0;
    ble_accelerometer_service_calibration_update(&m_accelerometer_service, &resetValue, 1);
}

void shutdown_accelerometer()
{
    NRF_LOG_INFO("Shutting down accelerometer");
    NRF_LOG_FLUSH();

    ret_code_t err_code = app_timer_stop(m_notification_timer_id);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Accelerometer measurement timer timeout.
 *
 * @details This function will be called each time the accelerometer level measurement timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void notification_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    ret_code_t err_code;
    // Increment the value of m_custom_value before nortifing it.

    float angles[3];
    memcpy(angles, angle_sensor.get_angles(), sizeof(float) * 3);

    float calibrationAngles[3];
    saved_parameters_getSavedCalibrationAngles(calibrationAngles);

    angles[0] = angles[0] - calibrationAngles[0];
    angles[1] = angles[1] - calibrationAngles[1];
    angles[2] = angles[2] - calibrationAngles[2];

    float widthAxisAdjustment = (tan((angle_sensor.get_angles()[0] - calibrationAngles[0]) * 3.14 / 180.0) *  saved_parameters_getSavedVehicleWidth());
    float lengthAxisAdjustment = (tan((angle_sensor.get_angles()[1] - calibrationAngles[1]) * 3.14 / 180.0) * saved_parameters_getSavedVehicleLength());

    if (saved_parameters_getSavedCurrentLevelingMode() == 1)
    {
        
        lengthAxisAdjustment = (tan(( angle_sensor.get_angles()[1] - saved_parameters_getSavedHitchHeightAngle() - calibrationAngles[1]) * 3.14 / 180.0) * saved_parameters_getSavedVehicleLength());
    }

    ble_accelerometer_service_width_axis_adjustment_update(widthAxisAdjustment);
    ble_accelerometer_service_length_axis_adjustment_update(lengthAxisAdjustment);
    //ble_accelerometer_service_sensor_data_set((uint8_t*)AccValue, (uint8_t)6);

    ble_accelerometer_service_angles_set(angles);

    if (max17260Sensor.initialised)
    {
        float soc;  // state of charge
        max17260_getStateOfCharge(&max17260Sensor, &soc);
        battery_service_battery_level_update((uint8_t)roundf(soc), BLE_CONN_HANDLE_ALL);
    }
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    
    err_code = app_timer_create(&m_notification_timer_id, APP_TIMER_MODE_REPEATED, notification_timeout_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
void bluetooth_advertising_timeout_callback(void)
{
    ret_code_t err_code;

    err_code = nrf_buddy_led_indication(NRF_BUDDY_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    /*if (mpu6050Sensor.initialised)
    {
        // Enable wakeup from pin P0.31
        nrf_gpio_cfg_sense_input(31, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_HIGH);

        mpu6050_SetCycleEnabled(&mpu6050Sensor, true);
        mpu6050_SetCycleFrequency(&mpu6050Sensor, WAKE_UP_1_25_HZ);
        mpu6050_SetTemperatureDisabled(&mpu6050Sensor, true);
        
        mpu6050_register_write(&mpu6050Sensor, MPU6050_PWR_MGMT2_REG , 0x03); // Sets the wake up period to be 5 Hz
        mpu6050_register_write(&mpu6050Sensor, MPU6050_CONFIG_REG , 0x00);      // Configure the DLPF to 10 Hz, 13.8 ms / 10 Hz, 13.4 ms, 1 kHz
        mpu6050_register_write(&mpu6050Sensor, MPU6050_ACCEL_CONFIG_REG , 0x00);      // Configure the DLPF to 10 Hz, 13.8 ms / 10 Hz, 13.4 ms, 1 kHz

        // Setup the wakeup interrupt on the MPU6050 
        mpu6050_SetMotionDetectionThreshold(&mpu6050Sensor, 1);   
        mpu6050_SetMotionDetectionDuration(&mpu6050Sensor, 0x01);
        mpu6050_SetAccelerometerPowerOnDelay(&mpu6050Sensor, 0);
        mpu6050_SetFreefallDetectionCounterDecrement(&mpu6050Sensor, 1);
        mpu6050_SetMotionDetectionCounterDecrement(&mpu6050Sensor, 1);
        mpu6050_EnableInterrupt(&mpu6050Sensor, MOT_EN | FF_EN); 
    }*/

#ifdef DEBUG_NRF
    // When in debug, this will put the device in a simulated sleep mode, still allowing the debugger to work
    sd_power_system_off();
    NRF_LOG_INFO("Powered off");
    while(1);
#else
    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    NRF_LOG_INFO("Powered off");
    APP_ERROR_CHECK(err_code);
#endif

}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

//Event Handler for TWI events
void twi_handler(nrfx_twi_evt_t const * p_event, void * p_context)
{
    //Check the event to see what type of event occurred
    switch (p_event->type)
    {
              //If data transmission or receiving is finished
      	case NRFX_TWI_EVT_DONE:
            switch (p_event->xfer_desc.address)
            {
                    break;

                default:
                    // do nothing
                    break;
            }
            break;

        case NRFX_TWI_EVT_ADDRESS_NACK:
           switch (p_event->xfer_desc.address)
            {
                default:
                    // do nothing
                    break;
            }
            break;

        case NRFX_TWI_EVT_DATA_NACK:
            switch (p_event->xfer_desc.address)
            {             

                default:
                    // do nothing
                    break;
            }
            break;
    }

    NRF_LOG_FLUSH();
}

//Initialize the TWI as Master device
void twi_master_init(void)
{
    ret_code_t err_code;

    // Configure the settings for twi communication
    const nrfx_twi_config_t twi_config = {
       .scl                = TWI_SCL_M,  //SCL Pin
       .sda                = TWI_SDA_M,  //SDA Pin
       .frequency          = NRF_TWI_FREQ_400K, //Communication Speed
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH, //Interrupt Priority(Note: if using Bluetooth then select priority carefully)
       .hold_bus_uninit     = true //automatically clear bus
    };

    //A function to initialize the twi communication
    err_code = nrfx_twi_init(&m_twi, &twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);
    
    //Enable the TWI Communication
    nrfx_twi_enable(&m_twi);
}

/**@brief Function for application main entry.
 */
int main(void)
{
    bool erase_bonds;

    ret_code_t err_code;

    // Initialize the nRF logger. Log messages are sent out the RTT interface
    log_init();
    
    // Start execution.
    NRF_LOG_INFO("Trailer Leveler Started.");
    NRF_LOG_INFO("Initilising Firmware...");
    NRF_LOG_FLUSH();

    twi_master_init();                  // initialize nRF5 the twi library 
    timers_init();                      // Initialise nRF5 timers library
    nrf_buddy_leds_init();              // initialise nRF52 buddy leds library
    power_management_init();            // initialise the nRF5 power management library

    
    saved_parameters_init();

    bluetooth_init();

    bluetooth_advertising_start(erase_bonds);

    ble_accelerometer_service_set_calibration_value_received_callback(calibration_value_received_callback);
    
    if (battery_service_init() != NRF_SUCCESS)
    {
        NRF_LOG_INFO("Error Initialising battery service");
    }

    if (ble_ess_service_init() != NRF_SUCCESS)
    {
        NRF_LOG_INFO("Error Initialising ess service");
    }
    if (ble_acceleration_service_init(ACCELEROMETER_BMI270) != NRF_SUCCESS)
    {
        NRF_LOG_INFO("Error Initialising accelerometer service");
    }
    
    bluetooth_register_connected_callback(initialise_accelerometer);
    bluetooth_register_disconnected_callback(shutdown_accelerometer);

    ble_accelerometer_service_set_orientation_received_handler(orientationValueReceivedHandler);
    ble_accelerometer_service_set_vehicle_length_received_handler(vehicleLengthValueReceivedHandler);
    ble_accelerometer_service_set_vehicle_width_received_handler(vehicleWidthValueReceivedHandler);
    ble_accelerometer_service_set_leveling_mode_received_handler(levelingModeReceivedHandler);

    NRF_LOG_INFO("Bluetooth setup complete");
    NRF_LOG_FLUSH();

    if (max17260_init(&max17260Sensor, &m_twi))
    {
        NRF_LOG_INFO("MAX17260 Initialised");
        
        uint16_t val;
        max17260_register_read(&max17260Sensor, 0x18, (uint8_t*)&val, 2);
        NRF_LOG_INFO("Value: %X", val);
        NRF_LOG_FLUSH();
    }

        // Set ADXL355 to be in I2C mode
    nrf_gpio_cfg_output(40);
    nrf_gpio_pin_clear(40);

    //// Set ADXL355 to have address 0x1D    
    nrf_gpio_cfg_output(7);
    nrf_gpio_pin_clear(7);

    angle_sensor.init(&m_twi);
    angle_sensor.set_orientation(saved_parameters_getSavedOrientation());
    //angle_sensor.wakeup();

    NRF_LOG_FLUSH();

    // Keeping this LED on as a visual indicator that the accelerometer has been found        
    //nrf_buddy_led_on(3);

    //initialise_accelerometer();
    // Enter main loop.
    for (;;)
    {
        bluetooth_idle_state_handle();
    }
}
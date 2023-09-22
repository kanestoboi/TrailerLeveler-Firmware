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
#include "nrf_drv_twi.h"

#include "Components/Accelerometers/ADXL355/adxl355.h"
#include "Components/Accelerometers/MPU6050/mpu6050.h"
#include "Components/Accelerometers/Accelerometers.h"
#include "Components/FuelGauge/MAX17260/max17260.h"
#include "Components/LED/nrf_buddy_led.h"
#include "Components/Bluetooth/Bluetooth.h"
#include "Components/Bluetooth/Services/AccelerometerService.h"
#include "Components/Bluetooth/Services/EnvironmentalService.h"

APP_TIMER_DEF(m_notification_timer_id);

//Initializing TWI0 instance
#define TWI_INSTANCE_ID     0

//I2C Pins Settings, you change them to any other pins
#define TWI_SCL_M           6         //I2C SCL Pin
#define TWI_SDA_M           8        //I2C SDA Pin

#define NOTIFICATION_INTERVAL           APP_TIMER_TICKS(80)

// Create a Handle for the twi communication
const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

MPU6050 mpu6050Sensor;
ADXL355 adxl355Sensor;
MAX17260 max17260Sensor;

int32_t xoutput = 0;
int32_t youtput = 0;
int32_t zoutput = 0;

float map(int32_t value, int32_t low1, int32_t high1, int32_t low2, int32_t high2);

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
    // create arrays which will hold x,y & z co-ordinates values of acc

    

    // Increment the value of m_custom_value before nortifing it.
    if (mpu6050Sensor.initialised)
    {
        static int16_t AccValue[3];
        if(mpu6050_ReadAcc(&mpu6050Sensor, &AccValue[0], &AccValue[1], &AccValue[2]) == true) // Read acc value from mpu6050 internal registers and save them in the array
        {
            xoutput = (0.9396f * xoutput + 0.0604 * (float)AccValue[0]);
            youtput = (0.9396f * youtput + 0.0604 * (float)AccValue[1]);
            zoutput = (0.9396f * zoutput + 0.0604 * (float)AccValue[2]);

            float xGs = map((uint32_t)xoutput, -32768, 32767, -90, 90);
            float yGs = map((uint32_t)youtput, -32768, 32767, -90, 90);
            float zGs = map((uint32_t)zoutput, -32768, 32767, -90, 90);

            float angles[3];

            calculateAnglesFromDeviceOrientation(xGs, yGs, zGs, angles);

            /*NRF_LOG_RAW_INFO("x" NRF_LOG_FLOAT_MARKER ", ", NRF_LOG_FLOAT(angles[0]) ); // display the read values
            NRF_LOG_RAW_INFO("y:" NRF_LOG_FLOAT_MARKER ", ", NRF_LOG_FLOAT(angles[1]) ); // display the read values
            NRF_LOG_RAW_INFO("z:" NRF_LOG_FLOAT_MARKER " ", NRF_LOG_FLOAT(angles[2]) ); // display the read values

            NRF_LOG_RAW_INFO("\n");
            NRF_LOG_FLUSH();*/

            
            uint32_t err_code = ble_accelerometer_service_sensor_data_set((uint8_t*)AccValue, (uint8_t)6);
            ble_accelerometer_service_angles_set((uint8_t*)angles, (uint8_t)12);      
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
            int16_t temperatureScaledToBleChar = (int16_t)(scaledTemperature*100.0);
            //NRF_LOG_RAW_INFO("y:" NRF_LOG_FLOAT_MARKER ", ", NRF_LOG_FLOAT(scaledTemperature) );
            NRF_LOG_RAW_INFO("TEMp: %d\n", temperatureScaledToBleChar); // if reading was unsuccessful then let the user know about it
            NRF_LOG_FLUSH();
            ble_ess_service_temperature_set(&temperatureScaledToBleChar, 2);
        }
    }
    else if (adxl355Sensor.initialised)
    {
        static int32_t AccValue[3];
        if(adxl355_ReadAcc(&adxl355Sensor, &AccValue[0], &AccValue[1], &AccValue[2]) == true) // Read acc value from mpu6050 internal registers and save them in the array
        {
            /*
             * This error code returned from ble_accelerometer_service_sensor_data_set 
             * will not be NRF_SUCCESS if there is no ble device
             * connected to the trailer leveler. The error code is not used.
            */

            /*
            float xGs = 9.81f*0.00000390625f * ((float)AccValue[0]);
            float yGs = 9.81f*0.00000390625f * ((float)AccValue[1]);
            float zGs = 9.81f*0.00000390625f * ((float)AccValue[2]);*/

            float xGs = map(AccValue[0], -262144, 262143, -90, 90);
            float yGs = map(AccValue[1], -262144, 262143, -90, 90);
            float zGs = map(AccValue[2], -262144, 262143, -90, 90);

            float angles[3];

            calculateAnglesFromDeviceOrientation(xGs, yGs, zGs, angles);
/*
            NRF_LOG_RAW_INFO("x:" NRF_LOG_FLOAT_MARKER ", ", NRF_LOG_FLOAT(xGs) ); // display the read values
            NRF_LOG_RAW_INFO("y:" NRF_LOG_FLOAT_MARKER ", ", NRF_LOG_FLOAT(yGs) ); // display the read values
            NRF_LOG_RAW_INFO("z:" NRF_LOG_FLOAT_MARKER " ", NRF_LOG_FLOAT(zGs) ); // display the read values

            NRF_LOG_RAW_INFO("\n");
            NRF_LOG_FLUSH();*/

            ble_accelerometer_service_sensor_data_set((uint8_t*)AccValue, (uint8_t)12);
            ble_accelerometer_service_angles_set((uint8_t*)angles, (uint8_t)12);
        }
    }

    float soc;

    max17260_getStateOfCharge(&max17260Sensor, &soc);

    bluetooth_update_battery_level((uint8_t)roundf(soc));


    /*
      adxl355_ReadTemp(&sensor, &tempValue);

      float temp = -0.11049723765f * ((float)tempValue - 1852.0f) + 25.0f;

      //NRF_LOG_RAW_INFO("Temperature: " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(temp));
*/

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

    if (mpu6050Sensor.initialised)
    {
        // Enable wakeup from pin P0.31
        nrf_gpio_cfg_sense_input(31, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_HIGH);

        mpu6050_register_write(&mpu6050Sensor, MPU6050_PWR_MGMT1_REG , 0x28); // Set MPU6050 cycle between sleep mode and wake up mode
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
    }

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
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    //Check the event to see what type of event occurred
    switch (p_event->type)
    {
              //If data transmission or receiving is finished
      	case NRF_DRV_TWI_EVT_DONE:
            switch (p_event->xfer_desc.address)
            {
                case MPU6050_ADDRESS:
                    mpu6050Sensor.mTransferDone = true;
                    break;

                case ADXL355_ADDRESS:
                    adxl355Sensor.mTransferDone = true;
                    break;
                
                case MAX17260_ADDRESS:
                    max17260Sensor.mTransferDone = true;
                    break;

                default:
                    // do nothing
                    break;
            }
            break;

        case NRF_DRV_TWI_EVT_ADDRESS_NACK:
           switch (p_event->xfer_desc.address)
            {
                case MPU6050_ADDRESS:
                    mpu6050Sensor.mTransferDone = true;
                    break;

                case ADXL355_ADDRESS:
                    adxl355Sensor.mTransferDone = true;
                    break;

                case MAX17260_ADDRESS:
                    max17260Sensor.mTransferDone = true;
                    break;

                default:
                    // do nothing
                    break;
            }
            break;

        case NRF_DRV_TWI_EVT_DATA_NACK:
            switch (p_event->xfer_desc.address)
            {
                case MPU6050_ADDRESS:
                    mpu6050Sensor.mTransferDone = true;
                    break;

                case ADXL355_ADDRESS:
                    adxl355Sensor.mTransferDone = true;
                    break;

                 case MAX17260_ADDRESS:
                    max17260Sensor.mTransferDone = true;
                    break;

                default:
                    // do nothing
                    break;
            }
            break;
    }
}

//Initialize the TWI as Master device
void twi_master_init(void)
{
    ret_code_t err_code;

    // Configure the settings for twi communication
    const nrf_drv_twi_config_t twi_config = {
       .scl                = TWI_SCL_M,  //SCL Pin
       .sda                = TWI_SDA_M,  //SDA Pin
       .frequency          = NRF_DRV_TWI_FREQ_400K, //Communication Speed
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH, //Interrupt Priority(Note: if using Bluetooth then select priority carefully)
       .clear_bus_init     = false //automatically clear bus
    };

    //A function to initialize the twi communication
    err_code = nrf_drv_twi_init(&m_twi, &twi_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);
    
    //Enable the TWI Communication
    nrf_drv_twi_enable(&m_twi);
}

/**@brief Function for application main entry.
 */
int main(void)
{
    bool erase_bonds;

    ret_code_t err_code;

    // Initialize the nRF logger. Log messages are sent out the RTT interface
    log_init();

    // Set ADXL355 to be in I2C mode
    nrf_gpio_cfg_output(40);
    nrf_gpio_pin_clear(40);

    // Set ADXL355 to have address 0x1D    
    nrf_gpio_cfg_output(7);
    nrf_gpio_pin_clear(7);

    // Set the charge state pin to be an input    
    nrf_gpio_cfg_input(41, GPIO_PIN_CNF_PULL_Disabled);

    // Start execution.
    NRF_LOG_INFO("Trailer Leveler Started.");
    NRF_LOG_INFO("Initilising Firmware...");
    NRF_LOG_FLUSH();

    twi_master_init();                  // initialize nRF5 the twi library 
    timers_init();                      // Initialise nRF5 timers library
    nrf_buddy_leds_init();              // initialise nRF52 buddy leds library
    power_management_init();            // initialise the nRF5 power management library

    bluetooth_init();
    bluetooth_advertising_start(erase_bonds);
    NRF_LOG_INFO("Bluetooth setup complete");
    NRF_LOG_FLUSH();

    if (max17260_init(&max17260Sensor, &m_twi))
    {
        NRF_LOG_INFO("MAX17260 Initialised");
        
        uint16_t val;
        max17260_register_read(&max17260Sensor, 0x18, (uint8_t*)&val, 2);
        NRF_LOG_INFO("Value: %X", val);
    }

    // Try to find an accelerometer sensor on the TWI bus
    while(adxl355_init(&adxl355Sensor, &m_twi) == false &&
          mpu6050_init(&mpu6050Sensor, &m_twi) == false) 
    {
        NRF_LOG_INFO("Failed to initialise an IMU...retrying"); // if it failed to initialize then print a message
        nrf_delay_ms(200);
        NRF_LOG_FLUSH();
    }

    NRF_LOG_INFO("Found Sensor"); // Found an accelerometer sensor
    NRF_LOG_FLUSH();

    if (mpu6050Sensor.initialised)
    {
        mpu6050_register_write(&mpu6050Sensor, MPU6050_PWR_MGMT1_REG , 0x00);   // Set accelerometer, gyro, and temperature sensor to be on
        mpu6050_register_write(&mpu6050Sensor, MPU6050_SAMPLE_RATE_REG , 0x07); // Set sample rate divider to be 7. Sample rate = 1,000 / (1+7) = 125 (Same sample may be received in FIFO twice)
        mpu6050_register_write(&mpu6050Sensor, MPU6050_CONFIG_REG , 0x06);      // Configure the DLPF to 10 Hz, 13.8 ms / 10 Hz, 13.4 ms, 1 kHz
        mpu6050_EnableInterrupt(&mpu6050Sensor, DISABLE_ALL_INTERRUPTS);        // Disable Interrupts
        mpu6050_register_write(&mpu6050Sensor, MPU6050_ACCEL_CONFIG_REG, 0x04); // Configure the DHPF available in the path leading to motion detectors to 0.63Hz

        bluetooth_initialise_accelerometer_service(ACCELEROMETER_MPU6050);      // Add accelerometer service to ble

        NRF_LOG_INFO("MPU6050 setup complete");
    }
    else if (adxl355Sensor.initialised)
    {
        adxl355_setPowerControl(&adxl355Sensor, ADXL355_POWER_CONTROL_FLAG_MEASUREMENT_MODE);
        adxl355_setFilterSettings(&adxl355Sensor, ADXL355_ODR_LPF_15_625HZ_3_906HZ);
        adxl355_setRange(&adxl355Sensor, ADXL_RANGE_2G);

        bluetooth_initialise_accelerometer_service(ACCELEROMETER_ADXL355);
        NRF_LOG_INFO("ADXL355 setup complete");
    }
    NRF_LOG_FLUSH();

    err_code = app_timer_start(m_notification_timer_id, NOTIFICATION_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    // Keeping this LED on as a visual indicator that the accelerometer has been found        
    nrf_buddy_led_on(3);

    // Enter main loop.
    for (;;)
    {
        bluetooth_idle_state_handle();
    }
}

float map(int32_t value, int32_t low1, int32_t high1, int32_t low2, int32_t high2) {
    return low2 + ((float)(high2 - low2) * (value - low1) / (high1 - low1));
}


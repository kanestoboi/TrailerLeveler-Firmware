#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "fds.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "sensorsim.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_delay.h"

#include "Components/Accelerometers/ADXL355/adxl355.h"
#include "Components/Accelerometers/MPU6050/mpu6050.h"
#include "Components/Accelerometers/Accelerometers.h"
#include "Components/FuelGauge/MAX17260/max17260.h"

#include "nrf_drv_twi.h"

#include "Components/LED/nrf_buddy_led.h"
#include "Components/Bluetooth/Bluetooth.h"

#include "Components/Bluetooth/Services/AccelerometerService.h"

#include "math.h"

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

void getADXL355AccelerometerData(int32_t *AccValue);


/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void notification_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    ret_code_t err_code;
    // create arrays which will hold x,y & z co-ordinates values of acc

    static uint16_t tempValue;

    // Increment the value of m_custom_value before nortifing it.
    if (mpu6050Sensor.initialised)
    {
        static int16_t AccValue[3];
        if(mpu6050_ReadAcc(&mpu6050Sensor, &AccValue[0], &AccValue[1], &AccValue[2]) == true) // Read acc value from mpu6050 internal registers and save them in the array
        {
            
            float xGs = 9.81f*0.00000390625f * ((float)AccValue[0]);
            float yGs = 9.81f*0.00000390625f * ((float)AccValue[1]);
            float zGs = 9.81f*0.00000390625f * ((float)AccValue[2]);


            NRF_LOG_RAW_INFO("x:" NRF_LOG_FLOAT_MARKER ", ", NRF_LOG_FLOAT(xGs) ); // display the read values
            NRF_LOG_RAW_INFO("y:" NRF_LOG_FLOAT_MARKER ", ", NRF_LOG_FLOAT(yGs) ); // display the read values
            NRF_LOG_RAW_INFO("z:" NRF_LOG_FLOAT_MARKER " ", NRF_LOG_FLOAT(zGs) ); // display the read values


            NRF_LOG_RAW_INFO("\n");

            NRF_LOG_FLUSH();
            //*/

            //if (sendAccelData)
            //{
            //static int16_t sendVal[3];
            //sendVal[2] = AccValue[0] << 8 | AccValue[0] >> 8;
            //sendVal[1] = 0;//AccValue[1] << 8 | AccValue[1] >> 8;
            //sendVal[0] = 0; //AccValue[2] << 8 | AccValue[2] >> 8;
            uint32_t err_code = ble_accelerometer_service_value_set((uint8_t*)AccValue, (uint8_t)6);
            //APP_ERROR_CHECK(err_code);
            //}
        
        }
        else
        {
            NRF_LOG_RAW_INFO("Reading ACC values Failed!!!"); // if reading was unsuccessful then let the user know about it
        }
    }
    else if (adxl355Sensor.initialised)
    {
        static int32_t AccValue[3];
        if(adxl355_ReadAcc(&adxl355Sensor, &AccValue[0], &AccValue[1], &AccValue[2]) == true) // Read acc value from mpu6050 internal registers and save them in the array
        {
            uint32_t err_code = ble_accelerometer_service_value_update(&m_accelerometer, (uint8_t*)AccValue, (uint8_t)12);
            APP_ERROR_CHECK(err_code);
        }
    }
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
static void sleep_mode_enter(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Preparing for sleep!");
    NRF_LOG_FLUSH();

    //if (adxl355Sensor.initialised)
    //{
    //  adxl355_setPowerControl(&adxl355Sensor, ADXL355_POWER_CONTROL_FLAG_STANDBY);
    //}

    err_code = nrf_buddy_led_indication(NRF_BUDDY_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    //app_timer_stop(m_notification_timer_id);

    // Wait for any ongoing transactions to complete
    while (nrf_drv_twi_is_busy(&m_twi));

    // Disable the TWI interface
    nrf_drv_twi_disable(&m_twi);

    // Release any resources associated with the TWI interface
    nrf_drv_twi_uninit(&m_twi);

    // Prepare wakeup buttons.
    //err_code = bsp_btn_ble_sleep_mode_prepare();
    //APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).

    NRF_LOG_INFO("Powering system off!");
    NRF_LOG_FLUSH();
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("This should not be printed!");
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

//Event Handler
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    //Check the event to see what type of event occurred
    switch (p_event->type)
    {
              //If data transmission or receiving is finished
      	case NRF_DRV_TWI_EVT_DONE:
          mpu6050Sensor.mTransferDone = true;
          adxl355Sensor.mTransferDone = true;
          break;

        case NRF_DRV_TWI_EVT_ADDRESS_NACK:
          mpu6050Sensor.mTransferDone = true;
          adxl355Sensor.mTransferDone = true;
          break;

        case NRF_DRV_TWI_EVT_DATA_NACK:
          mpu6050Sensor.mTransferDone = true;
          adxl355Sensor.mTransferDone = true;
          break;
        
        default:
          // do nothing
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

void getADXL355AccelerometerData(int32_t *AccValue)
{
    if(adxl355_ReadAcc(&adxl355Sensor, &AccValue[0], &AccValue[1], &AccValue[2]) == true) // Read acc value from mpu6050 internal registers and save them in the array
    {
    
      //float xGs = 9.81f*0.00000390625f * ((float)AccValue[0]);
      //float yGs = 9.81f*0.00000390625f * ((float)AccValue[1]);
      //float zGs = 9.81f*0.00000390625f * ((float)AccValue[2]);
      
      //NRF_LOG_RAW_INFO("x:" NRF_LOG_FLOAT_MARKER ", ", NRF_LOG_FLOAT(xGs) ); // display the read values
      //NRF_LOG_RAW_INFO("y:" NRF_LOG_FLOAT_MARKER ", ", NRF_LOG_FLOAT(yGs) ); // display the read values
      //NRF_LOG_RAW_INFO("z:" NRF_LOG_FLOAT_MARKER " ", NRF_LOG_FLOAT(zGs) ); // display the read values

      //NRF_LOG_RAW_INFO("\n");
      //NRF_LOG_FLUSH();
    }
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

    // Set the charge state pin to be an imput    
    nrf_gpio_cfg_input(41, GPIO_PIN_CNF_PULL_Pullup);

    nrf_delay_ms(100); // give some delay

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
        (void)mpu6050_register_write(&mpu6050Sensor, MPU6050_PWR_MGMT1_REG , 0x00); 
        (void)mpu6050_register_write(&mpu6050Sensor, MPU6050_SAMPLE_RATE_REG , 0x07); 
        (void)mpu6050_register_write(&mpu6050Sensor, MPU6050_CONFIG_REG , 0x06); 						
        (void)mpu6050_register_write(&mpu6050Sensor, MPU6050_INT_EN_REG, 0x00); 
        (void)mpu6050_register_write(&mpu6050Sensor, MPU6050_GYRO_CONFIG_REG , 0x18); 
        (void)mpu6050_register_write(&mpu6050Sensor, MPU6050_ACCEL_CONFIG_REG,0x00);

        //bluetooth_initialise_accelerometer_service(ACCELEROMETER_MPU6050);

        NRF_LOG_INFO("MPU6050 setup complete");
    }
    else if (adxl355Sensor.initialised)
    {
        adxl355_setPowerControl(&adxl355Sensor, ADXL355_POWER_CONTROL_FLAG_MEASUREMENT_MODE);
        adxl355_setFilterSettings(&adxl355Sensor, ADXL355_ODR_LPF_15_625HZ_3_906HZ);
        adxl_setRange(&adxl355Sensor, ADXL_RANGE_2G);

        bluetooth_initialise_accelerometer_service(ACCELEROMETER_ADXL355);
        NRF_LOG_INFO("ADXL355 setup complete");
    }
    NRF_LOG_FLUSH();

    nrf_delay_ms(1000);
    NRF_LOG_INFO("Starting timers");
    err_code = app_timer_start(m_notification_timer_id, NOTIFICATION_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Timers started");

        // Enter main loop.
    for (;;)
    {
        bluetooth_idle_state_handle();
    }
}


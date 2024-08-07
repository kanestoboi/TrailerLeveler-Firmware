#include "sdk_common.h"
#include "ble_srv_common.h"
#include "AccelerometerService.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_log.h"
#include "math.h"
#include "nrf_log_ctrl.h"


#define _RAD_TO_DEG 57.2957795131f  // Constant to convert radians to degrees
#define _PI 3.14159265359f           // Constant for the value of pi

// Callback functions for when value are received through BLE characteristics
void (*mCalibrationValueReceivedHandler)(uint8_t value) = NULL;
void (*mOrientationValueReceivedHandler)(uint16_t orientation) = NULL;
void (*mVehicleLengthValueReceivedHandler)(uint16_t length) = NULL;
void (*mVehicleWidthValueReceivedHandler)(uint16_t width) = NULL;
void (*mLevelingModeReceivedHandler)(uint16_t levelingMode) = NULL;

static uint32_t accelerometer_sensor_data_char_add(const ble_accelerometer_service_init_t * p_ble_accelerometer_service_init, const accelerometer_t accelerometer);
static uint32_t accelerometer_angles_char_add( const ble_accelerometer_service_init_t * p_ble_accelerometer_service_init);
static uint32_t accelerometer_orientation_char_add(const ble_accelerometer_service_init_t * p_ble_accelerometer_service_init);
static uint32_t accelerometer_calibration_char_add(const ble_accelerometer_service_init_t * p_ble_accelerometer_service_init);
static uint32_t accelerometer_saved_hitch_angle_char_add(const ble_accelerometer_service_init_t * p_ble_accelerometer_service_init);
static uint32_t accelerometer_vehicle_length_char_add(const ble_accelerometer_service_init_t * p_ble_accelerometer_service_init);
static uint32_t accelerometer_vehicle_width_char_add(const ble_accelerometer_service_init_t * p_ble_accelerometer_service_init);
static uint32_t accelerometer_length_axis_adjustment_char_add(const ble_accelerometer_service_init_t * p_ble_accelerometer_service_init);
static uint32_t accelerometer_width_axis_adjustment_char_add(const ble_accelerometer_service_init_t * p_ble_accelerometer_service_init);
static uint32_t accelerometer_current_leveling_mode_char_add(const ble_accelerometer_service_init_t * p_ble_accelerometer_service_init);


BLE_ACCELEROMETER_DEF(m_accelerometer_service);

uint32_t ble_acceleration_service_init(const accelerometer_t accelerometer)
{
    ble_accelerometer_service_init_t accelerometer_service_init;
    
     // Initialize custom Service init structure to zero.
    memset(&accelerometer_service_init, 0, sizeof(accelerometer_service_init));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&accelerometer_service_init.accelerometer_sensor_data_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&accelerometer_service_init.accelerometer_sensor_data_char_attr_md.write_perm);

    // Set the accelerometer event handler
    accelerometer_service_init.evt_handler = ble_accelerometer_on_accelerometer_evt;

    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    m_accelerometer_service.conn_handle = BLE_CONN_HANDLE_INVALID;

    ble_uuid128_t base_uuid = {ACCELEROMETER_SERVICE_UUID_BASE};
    err_code =  sd_ble_uuid_vs_add(&base_uuid, &m_accelerometer_service.uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = m_accelerometer_service.uuid_type;
    ble_uuid.uuid = ACCELEROMETER_SERVICE_UUID;

    // Add the Custom Service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &m_accelerometer_service.service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Initialize service structure
    m_accelerometer_service.evt_handler           = accelerometer_service_init.evt_handler;
    m_accelerometer_service.conn_handle           = BLE_CONN_HANDLE_INVALID;

    // Add accelerometer value characteristic to the accelerometer service
    err_code = accelerometer_sensor_data_char_add( &accelerometer_service_init, accelerometer);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add the angles characteristic to the accelerometer service
    err_code = accelerometer_angles_char_add(&accelerometer_service_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add the orientation characteristic to the accelerometer service
    err_code = accelerometer_orientation_char_add(&accelerometer_service_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add the calibration characteristic to the accelerometer service
    err_code = accelerometer_calibration_char_add(&accelerometer_service_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = accelerometer_saved_hitch_angle_char_add(&accelerometer_service_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = accelerometer_vehicle_length_char_add(&accelerometer_service_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = accelerometer_vehicle_width_char_add(&accelerometer_service_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = accelerometer_current_leveling_mode_char_add(&accelerometer_service_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = accelerometer_length_axis_adjustment_char_add(&accelerometer_service_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = accelerometer_width_axis_adjustment_char_add(&accelerometer_service_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }


    return NRF_SUCCESS;
}    

/**@brief Function for adding the acceleration sensor data characteristic.
 *
 * @param[in]   p_cus        Custom Service structure.
 * @param[in]   p_cus_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t accelerometer_sensor_data_char_add(const ble_accelerometer_service_init_t * p_ble_accelerometer_service_init, const accelerometer_t accelerometer)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    // Read  operation on Cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 0;
    char_md.char_props.notify = 1; 
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md; 
    char_md.p_sccd_md         = NULL;

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_ble_accelerometer_service_init->accelerometer_sensor_data_char_attr_md.read_perm;
    attr_md.write_perm = p_ble_accelerometer_service_init->accelerometer_sensor_data_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    ble_uuid.type = m_accelerometer_service.uuid_type;

    if (accelerometer == ACCELEROMETER_ADXL355)
    {
      ble_uuid.uuid = ACCELEROMETER_ADXL355_VALUE_CHAR_UUID;
    }
    else if (accelerometer == ACCELEROMETER_MPU6050)
    {
      ble_uuid.uuid = ACCELEROMETER_MPU6050_VALUE_CHAR_UUID;
    }
    else if (accelerometer == ACCELEROMETER_BMI270)
    {
      ble_uuid.uuid = ACCELEROMETER_BMI270_VALUE_CHAR_UUID;
    }

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 6*sizeof(uint8_t);
    attr_char_value.init_offs = 0;

    if (accelerometer == ACCELEROMETER_ADXL355)
    {
      attr_char_value.max_len   = 12*sizeof(uint8_t);
    }
    else if (accelerometer == ACCELEROMETER_MPU6050 || accelerometer == ACCELEROMETER_BMI270)
    {
      attr_char_value.max_len   = 6*sizeof(uint8_t);
    }
    

    err_code = sd_ble_gatts_characteristic_add(m_accelerometer_service.service_handle, &char_md,
                                               &attr_char_value,
                                               &m_accelerometer_service.accelerometer_sensor_data_handles);
    
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;

}

/**@brief Function for adding the Custom Value characteristic.
 *
 * @param[in]   p_cus        Custom Service structure.
 * @param[in]   p_cus_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t accelerometer_angles_char_add(const ble_accelerometer_service_init_t * p_ble_accelerometer_service_init)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    // Read  operation on Cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 0;
    char_md.char_props.notify = 1; 
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md; 
    char_md.p_sccd_md         = NULL;

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_ble_accelerometer_service_init->accelerometer_sensor_data_char_attr_md.read_perm;
    attr_md.write_perm = p_ble_accelerometer_service_init->accelerometer_sensor_data_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    ble_uuid.type = m_accelerometer_service.uuid_type;

    ble_uuid.uuid = ACCELEROMETER_ANGLE_CHAR_UUID;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 6*sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    
    attr_char_value.max_len   = 12*sizeof(uint8_t);


    err_code = sd_ble_gatts_characteristic_add(m_accelerometer_service.service_handle, &char_md,
                                               &attr_char_value,
                                               &m_accelerometer_service.accelerometer_angles_handles);
    
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;

}

/**@brief Function for adding the Custom Value characteristic.
 *
 * @param[in]   p_cus        Custom Service structure.
 * @param[in]   p_cus_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t accelerometer_orientation_char_add(const ble_accelerometer_service_init_t * p_ble_accelerometer_service_init)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    // Read  operation on Cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.char_props.notify = 0; 
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md; 
    char_md.p_sccd_md         = NULL;

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_ble_accelerometer_service_init->accelerometer_sensor_data_char_attr_md.read_perm;
    attr_md.write_perm = p_ble_accelerometer_service_init->accelerometer_sensor_data_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    ble_uuid.type = m_accelerometer_service.uuid_type;

    ble_uuid.uuid = ACCELEROMETER_ORIENTATION_CHAR_UUID;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 1*sizeof(uint8_t);
    attr_char_value.init_offs = 0;

    uint8_t initialOrientation = 1;
        
    attr_char_value.p_value   = &initialOrientation; // Pointer to the initial value

    attr_char_value.max_len   = 1*sizeof(uint8_t);

    err_code = sd_ble_gatts_characteristic_add(m_accelerometer_service.service_handle, &char_md,
                                               &attr_char_value,
                                               &m_accelerometer_service.accelerometer_orientation_handles);
    
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;

}

/**@brief Function for adding the Custom Value characteristic.
 *
 * @param[in]   p_cus        Custom Service structure.
 * @param[in]   p_cus_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t accelerometer_calibration_char_add(const ble_accelerometer_service_init_t * p_ble_accelerometer_service_init)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    // Read  operation on Cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.char_props.notify = 0; 
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md; 
    char_md.p_sccd_md         = NULL;

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_ble_accelerometer_service_init->accelerometer_sensor_data_char_attr_md.read_perm;
    attr_md.write_perm = p_ble_accelerometer_service_init->accelerometer_sensor_data_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    ble_uuid.type = m_accelerometer_service.uuid_type;

    ble_uuid.uuid = ACCELEROMETER_CALIBRATION_CHAR_UUID;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 1*sizeof(uint8_t);
    attr_char_value.init_offs = 0;

    uint8_t resetValue = 0;        
    attr_char_value.p_value   = (uint8_t*)&resetValue; // Pointer to the initial value

    attr_char_value.max_len   = 1*sizeof(uint8_t);

    err_code = sd_ble_gatts_characteristic_add(m_accelerometer_service.service_handle, &char_md,
                                               &attr_char_value,
                                               &m_accelerometer_service.accelerometer_calibration_handles);
    
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;

}

/**@brief Function for adding the Saved Hitch Angle characteristic.
 *
 * @param[in]   p_accelerometer_service             Accelerometer Service structure.
 * @param[in]   p_ble_accelerometer_service_init    Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t accelerometer_saved_hitch_angle_char_add(const ble_accelerometer_service_init_t * p_ble_accelerometer_service_init)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    // Read  operation on Cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 0;
    char_md.char_props.notify = 0; 
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md; 
    char_md.p_sccd_md         = NULL;

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_ble_accelerometer_service_init->accelerometer_sensor_data_char_attr_md.read_perm;
    attr_md.write_perm = p_ble_accelerometer_service_init->accelerometer_sensor_data_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    ble_uuid.type = m_accelerometer_service.uuid_type;

    ble_uuid.uuid = ACCELEROMETER_SAVED_HITCH_ANGLE_CHAR_UUID;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 1*sizeof(float);
    attr_char_value.init_offs = 0;

    float initialSavedHitchAngle = 0.0;        
    attr_char_value.p_value   = (uint8_t*)&initialSavedHitchAngle; // Pointer to the initial value

    attr_char_value.max_len   = 1*sizeof(float);

    err_code = sd_ble_gatts_characteristic_add(m_accelerometer_service.service_handle, &char_md,
                                               &attr_char_value,
                                               &m_accelerometer_service.accelerometer_saved_hitch_angle_handles);
    
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}


/**@brief Function for adding the vehicle length characteristic.
 *
 * @param[in]   p_accelerometer_service             Accelerometer Service structure.
 * @param[in]   p_ble_accelerometer_service_init    Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t accelerometer_vehicle_length_char_add(const ble_accelerometer_service_init_t * p_ble_accelerometer_service_init)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    // Read  operation on Cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.char_props.notify = 1; 
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md; 
    char_md.p_sccd_md         = NULL;

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_ble_accelerometer_service_init->accelerometer_sensor_data_char_attr_md.read_perm;
    attr_md.write_perm = p_ble_accelerometer_service_init->accelerometer_sensor_data_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    ble_uuid.type = m_accelerometer_service.uuid_type;

    ble_uuid.uuid = ACCELEROMETER_VEHICLE_LENGTH_VALUE_CHAR_UUID;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 1*sizeof(float);
    attr_char_value.init_offs = 0;

    float initialSavedVehicleLength = 1.0;   
    attr_char_value.p_value   = (uint8_t*)&initialSavedVehicleLength; // Pointer to the initial value

    attr_char_value.max_len   = 1*sizeof(float);

    err_code = sd_ble_gatts_characteristic_add(m_accelerometer_service.service_handle, &char_md,
                                               &attr_char_value,
                                               &m_accelerometer_service.accelerometer_saved_vehicle_length_handles);
    
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

/**@brief Function for adding the vehicle length characteristic.
 *
 * @param[in]   p_accelerometer_service             Accelerometer Service structure.
 * @param[in]   p_ble_accelerometer_service_init    Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t accelerometer_length_axis_adjustment_char_add(const ble_accelerometer_service_init_t * p_ble_accelerometer_service_init)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    // Read  operation on Cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 0;
    char_md.char_props.notify = 1; 
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md; 
    char_md.p_sccd_md         = NULL;

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_ble_accelerometer_service_init->accelerometer_sensor_data_char_attr_md.read_perm;
    attr_md.write_perm = p_ble_accelerometer_service_init->accelerometer_sensor_data_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    ble_uuid.type = m_accelerometer_service.uuid_type;

    ble_uuid.uuid = ACCELEROMETER_LENGTH_AXIS_ADJUSTMENT_VALUE_CHAR_UUID;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 1*sizeof(float);
    attr_char_value.init_offs = 0;

    float initialSavedVehicleLength = 1.0;        
    attr_char_value.p_value   = (uint8_t*)&initialSavedVehicleLength; // Pointer to the initial value

    attr_char_value.max_len   = 1*sizeof(float);

    err_code = sd_ble_gatts_characteristic_add(m_accelerometer_service.service_handle, &char_md,
                                               &attr_char_value,
                                               &m_accelerometer_service.accelerometer_length_axis_adjustment_handles);
    
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

/**@brief Function for adding the vehicle length characteristic.
 *
 * @param[in]   p_accelerometer_service             Accelerometer Service structure.
 * @param[in]   p_ble_accelerometer_service_init    Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t accelerometer_width_axis_adjustment_char_add(const ble_accelerometer_service_init_t * p_ble_accelerometer_service_init)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    // Read  operation on Cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 0;
    char_md.char_props.notify = 1; 
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md; 
    char_md.p_sccd_md         = NULL;

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_ble_accelerometer_service_init->accelerometer_sensor_data_char_attr_md.read_perm;
    attr_md.write_perm = p_ble_accelerometer_service_init->accelerometer_sensor_data_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    ble_uuid.type = m_accelerometer_service.uuid_type;

    ble_uuid.uuid = ACCELEROMETER_WIDTH_AXIS_ADJUSTMENT_VALUE_CHAR_UUID;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 1*sizeof(float);
    attr_char_value.init_offs = 0;

    float initialSavedVehicleLength = 1.0;        
    attr_char_value.p_value   = (uint8_t*)&initialSavedVehicleLength; // Pointer to the initial value

    attr_char_value.max_len   = 1*sizeof(float);

    err_code = sd_ble_gatts_characteristic_add(m_accelerometer_service.service_handle, &char_md,
                                               &attr_char_value,
                                               &m_accelerometer_service.accelerometer_width_axis_adjustment_handles);
    
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

/**@brief Function for adding the vehicle width characteristic.
 *
 * @param[in]   p_accelerometer_service             Accelerometer Service structure.
 * @param[in]   p_ble_accelerometer_service_init    Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t accelerometer_vehicle_width_char_add(const ble_accelerometer_service_init_t * p_ble_accelerometer_service_init)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    // Read  operation on Cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.char_props.notify = 1; 
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md; 
    char_md.p_sccd_md         = NULL;

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_ble_accelerometer_service_init->accelerometer_sensor_data_char_attr_md.read_perm;
    attr_md.write_perm = p_ble_accelerometer_service_init->accelerometer_sensor_data_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    ble_uuid.type = m_accelerometer_service.uuid_type;

    ble_uuid.uuid = ACCELEROMETER_VEHICLE_WIDTH_VALUE_CHAR_UUID;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 1*sizeof(float);
    attr_char_value.init_offs = 0;

    float initialSavedVehicleWidth = 1.0;        
    attr_char_value.p_value   = (uint8_t*)&initialSavedVehicleWidth; // Pointer to the initial value

    attr_char_value.max_len   = 1*sizeof(float);

    err_code = sd_ble_gatts_characteristic_add(m_accelerometer_service.service_handle, &char_md,
                                               &attr_char_value,
                                               &m_accelerometer_service.accelerometer_saved_vehicle_width_handles);
    
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

/**@brief Function for adding the leveling mode characteristic.
 *
 * @param[in]   p_accelerometer_service             Accelerometer Service structure.
 * @param[in]   p_ble_accelerometer_service_init    Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t accelerometer_current_leveling_mode_char_add(const ble_accelerometer_service_init_t * p_ble_accelerometer_service_init)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    // Read  operation on Cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.char_props.notify = 1; 
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md; 
    char_md.p_sccd_md         = NULL;

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_ble_accelerometer_service_init->accelerometer_sensor_data_char_attr_md.read_perm;
    attr_md.write_perm = p_ble_accelerometer_service_init->accelerometer_sensor_data_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    ble_uuid.type = m_accelerometer_service.uuid_type;

    ble_uuid.uuid = ACCELEROMETER_CURRENT_LEVELING_MODE_VALUE_CHAR_UUID;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 1*sizeof(uint8_t);
    attr_char_value.init_offs = 0;

    uint8_t initialSavedCurrentLevelingMode = 1;        
    attr_char_value.p_value   = (uint8_t*)&initialSavedCurrentLevelingMode; // Pointer to the initial value

    attr_char_value.max_len   = 1*sizeof(uint8_t);

    err_code = sd_ble_gatts_characteristic_add(m_accelerometer_service.service_handle, &char_md,
                                               &attr_char_value,
                                               &m_accelerometer_service.accelerometer_vehicle_leveling_mode_handles);
    
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

void ble_accelerometer_on_ble_evt( ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_accelerometer_service_t * p_accelerometer_service = (ble_accelerometer_service_t *) p_context;
    
    if (p_accelerometer_service == NULL || p_ble_evt == NULL)
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_accelerometer_service, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_accelerometer_service, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_accelerometer_service, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_cus       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_accelerometer_service_t * p_accelerometer_service, ble_evt_t const * p_ble_evt)
{
    p_accelerometer_service->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

    ble_accelerometer_evt_t evt;

    evt.evt_type = BLE_ACCELEROMETER_EVT_CONNECTED;
    
    // check if the accelerometer service ble event handler has been initialised before calling it
    if (p_accelerometer_service->evt_handler == NULL)
    {
        return;
    }

    p_accelerometer_service->evt_handler(p_accelerometer_service, &evt);
}

/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_cus       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_accelerometer_service_t * p_accelerometer_service, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_accelerometer_service->conn_handle = BLE_CONN_HANDLE_INVALID;
}

/* This code belongs in ble_cus.c*/

/**@brief Function for handling the Write event.
 *
 * @param[in]   p_cus       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_accelerometer_service_t * p_accelerometer_service, ble_evt_t const * p_ble_evt)
{
   const ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    
    // Check if the handle passed with the event matches the Custom Value Characteristic handle.
    if (p_evt_write->handle == p_accelerometer_service->accelerometer_sensor_data_handles.value_handle)
    {
        // Put specific task here. 
        NRF_LOG_INFO("Message Received.");
    }

    // Check if the Custom value CCCD is written to and that the value is the appropriate length, i.e 2 bytes.
    if ((p_evt_write->handle == p_accelerometer_service->accelerometer_sensor_data_handles.cccd_handle)
        && (p_evt_write->len == 2)
       )
    {
        // CCCD written, call application event handler
        if (p_accelerometer_service->evt_handler != NULL)
        {
            ble_accelerometer_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_ACCELEROMETER_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_ACCELEROMETER_EVT_NOTIFICATION_DISABLED;
            }
            // Call the application event handler.
            p_accelerometer_service->evt_handler(p_accelerometer_service, &evt);
        }
    }

    /// Check if the handle passed with the event matches the Orientation Characteristic handle.
    if ((p_evt_write->handle == p_accelerometer_service->accelerometer_orientation_handles.value_handle)
        && (p_evt_write->len == 1)
       )
    {
        NRF_LOG_INFO("Message Received from orientation.");

        uint32_t valToWrite = (uint32_t)p_evt_write->data[0];

        mOrientationValueReceivedHandler((uint16_t)valToWrite);
    }

    // Check if the handle passed with the event matches the Orientation Characteristic handle.
    if ((p_evt_write->handle == p_accelerometer_service->accelerometer_calibration_handles.value_handle)
        && (p_evt_write->len == 1)
       )
    {
        mCalibrationValueReceivedHandler(p_evt_write->data[0]);
    }

    if (p_evt_write->handle == p_accelerometer_service->accelerometer_saved_vehicle_length_handles.value_handle)
    {
        NRF_LOG_INFO("Message Received from vehicle length.");
        float *lengthReceivedPtr = (float*)(p_evt_write->data);
        float lengthReceived = *lengthReceivedPtr;

        mVehicleLengthValueReceivedHandler(lengthReceived);
    }

    if (p_evt_write->handle == p_accelerometer_service->accelerometer_saved_vehicle_width_handles.value_handle)
    {
        NRF_LOG_INFO("Message Received from vehicle width.");
        float *widthReceivedPtr = (float*)p_evt_write->data;
        float widthReceived = *widthReceivedPtr;
        mVehicleWidthValueReceivedHandler(widthReceived);
    }

    if (p_evt_write->handle == p_accelerometer_service->accelerometer_vehicle_leveling_mode_handles.value_handle)
    {
        NRF_LOG_INFO("Message Received from leveling mode.");
        float *widthReceivedPtr = (float*)p_evt_write->data;
        uint8_t levelingMode = *p_evt_write->data;

        mLevelingModeReceivedHandler(levelingMode);
    }
};

uint32_t ble_accelerometer_service_sensor_data_update(ble_accelerometer_service_t * p_accelerometer_service, uint8_t *custom_value, uint8_t custom_value_length)
{
    if (p_accelerometer_service == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = custom_value_length*sizeof(uint8_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = custom_value;

    // Update database.
    err_code= sd_ble_gatts_value_set(p_accelerometer_service->conn_handle,
                                        p_accelerometer_service->accelerometer_sensor_data_handles.value_handle,
                                        &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Send value if connected and notifying.
    if ((p_accelerometer_service->conn_handle != BLE_CONN_HANDLE_INVALID)) 
    {
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_accelerometer_service->accelerometer_sensor_data_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        err_code = sd_ble_gatts_hvx(p_accelerometer_service->conn_handle, &hvx_params);
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}

uint32_t ble_accelerometer_service_angles_update(ble_accelerometer_service_t * p_accelerometer_service, uint8_t *custom_value, uint8_t custom_value_length)
{
    if (p_accelerometer_service == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = custom_value_length*sizeof(uint8_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = custom_value;

    // Update database.
    err_code= sd_ble_gatts_value_set(p_accelerometer_service->conn_handle,
                                        p_accelerometer_service->accelerometer_angles_handles.value_handle,
                                        &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Send value if connected and notifying.
    if ((p_accelerometer_service->conn_handle != BLE_CONN_HANDLE_INVALID)) 
    {
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_accelerometer_service->accelerometer_angles_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        err_code = sd_ble_gatts_hvx(p_accelerometer_service->conn_handle, &hvx_params);
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}

uint32_t ble_accelerometer_service_orientation_update(uint8_t orientation)
{
    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = sizeof(uint8_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = &orientation;

    // Update database.
    err_code= sd_ble_gatts_value_set(m_accelerometer_service.conn_handle,
                                        m_accelerometer_service.accelerometer_orientation_handles.value_handle,
                                        &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Send value if connected and notifying.
    if ((m_accelerometer_service.conn_handle != BLE_CONN_HANDLE_INVALID)) 
    {
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = m_accelerometer_service.accelerometer_orientation_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        err_code = sd_ble_gatts_hvx(m_accelerometer_service.conn_handle, &hvx_params);
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}

uint32_t ble_accelerometer_service_calibration_update(ble_accelerometer_service_t * p_accelerometer_service, uint8_t *custom_value, uint8_t custom_value_length)
{
    if (p_accelerometer_service == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = custom_value_length*sizeof(uint8_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = custom_value;

    // Update database.
    err_code= sd_ble_gatts_value_set(p_accelerometer_service->conn_handle,
                                        p_accelerometer_service->accelerometer_calibration_handles.value_handle,
                                        &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Send value if connected and notifying.
    if ((p_accelerometer_service->conn_handle != BLE_CONN_HANDLE_INVALID)) 
    {
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_accelerometer_service->accelerometer_calibration_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        err_code = sd_ble_gatts_hvx(p_accelerometer_service->conn_handle, &hvx_params);
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}

uint32_t ble_accelerometer_service_saved_hitch_angle_update(float angle)
{
    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = sizeof(float);
    gatts_value.offset  = 0;
    gatts_value.p_value = (uint8_t*)&angle;

    // Update database.
    err_code= sd_ble_gatts_value_set(m_accelerometer_service.conn_handle,
                                        m_accelerometer_service.accelerometer_saved_hitch_angle_handles.value_handle,
                                        &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Send value if connected and notifying.
    if ((m_accelerometer_service.conn_handle != BLE_CONN_HANDLE_INVALID)) 
    {
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = m_accelerometer_service.accelerometer_saved_hitch_angle_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        err_code = sd_ble_gatts_hvx(m_accelerometer_service.conn_handle, &hvx_params);
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}


uint32_t ble_accelerometer_service_vehicle_length_update(ble_accelerometer_service_t * p_accelerometer_service, float length)
{
    if (p_accelerometer_service == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = sizeof(float);
    gatts_value.offset  = 0;
    gatts_value.p_value = (uint8_t*)&length;

    // Update database.
    err_code= sd_ble_gatts_value_set(p_accelerometer_service->conn_handle,
                                        p_accelerometer_service->accelerometer_saved_hitch_angle_handles.value_handle,
                                        &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Send value if connected and notifying.
    if ((p_accelerometer_service->conn_handle != BLE_CONN_HANDLE_INVALID)) 
    {
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_accelerometer_service->accelerometer_saved_vehicle_length_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        err_code = sd_ble_gatts_hvx(p_accelerometer_service->conn_handle, &hvx_params);
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}


uint32_t ble_accelerometer_service_vehicle_width_update(ble_accelerometer_service_t * p_accelerometer_service, float width)
{
    if (p_accelerometer_service == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = sizeof(float);
    gatts_value.offset  = 0;
    gatts_value.p_value = (uint8_t*)&width;

    // Update database.
    err_code= sd_ble_gatts_value_set(p_accelerometer_service->conn_handle,
                                        p_accelerometer_service->accelerometer_saved_vehicle_width_handles.value_handle,
                                        &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Send value if connected and notifying.
    if ((p_accelerometer_service->conn_handle != BLE_CONN_HANDLE_INVALID)) 
    {
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_accelerometer_service->accelerometer_saved_hitch_angle_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        err_code = sd_ble_gatts_hvx(p_accelerometer_service->conn_handle, &hvx_params);
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}

uint32_t ble_accelerometer_service_leveling_mode_update(uint8_t mode)
{
    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = sizeof(uint8_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = (uint8_t*)&mode;

    // Update database.
    err_code= sd_ble_gatts_value_set(m_accelerometer_service.conn_handle,
                                        m_accelerometer_service.accelerometer_vehicle_leveling_mode_handles.value_handle,
                                        &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Send value if connected and notifying.
    if ((m_accelerometer_service.conn_handle != BLE_CONN_HANDLE_INVALID)) 
    {
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = m_accelerometer_service.accelerometer_saved_hitch_angle_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        err_code = sd_ble_gatts_hvx(m_accelerometer_service.conn_handle, &hvx_params);
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}

uint32_t ble_accelerometer_service_length_axis_adjustment_update(float lengthAxisAdjustment)
{
    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = sizeof(float);
    gatts_value.offset  = 0;
    gatts_value.p_value = (uint8_t*)&lengthAxisAdjustment;

    // Update database.
    err_code= sd_ble_gatts_value_set(m_accelerometer_service.conn_handle,
                                        m_accelerometer_service.accelerometer_length_axis_adjustment_handles.value_handle,
                                        &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Send value if connected and notifying.
    if ((m_accelerometer_service.conn_handle != BLE_CONN_HANDLE_INVALID)) 
    {
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = m_accelerometer_service.accelerometer_length_axis_adjustment_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        err_code = sd_ble_gatts_hvx(m_accelerometer_service.conn_handle, &hvx_params);
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}

uint32_t ble_accelerometer_service_width_axis_adjustment_update(float widthAxisAdjustment)
{
    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = sizeof(float);
    gatts_value.offset  = 0;
    gatts_value.p_value = (uint8_t*)&widthAxisAdjustment;

    // Update database.
    err_code= sd_ble_gatts_value_set(m_accelerometer_service.conn_handle,
                                        m_accelerometer_service.accelerometer_width_axis_adjustment_handles.value_handle,
                                        &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Send value if connected and notifying.
    if ((m_accelerometer_service.conn_handle != BLE_CONN_HANDLE_INVALID)) 
    {
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = m_accelerometer_service.accelerometer_width_axis_adjustment_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        err_code = sd_ble_gatts_hvx(m_accelerometer_service.conn_handle, &hvx_params);
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}

uint32_t ble_accelerometer_service_saved_vehicle_length_update(float length)
{
    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = sizeof(float);
    gatts_value.offset  = 0;
    gatts_value.p_value = (uint8_t*)&length;

    // Update database.
    err_code= sd_ble_gatts_value_set(m_accelerometer_service.conn_handle,
                                        m_accelerometer_service.accelerometer_saved_vehicle_length_handles.value_handle,
                                        &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Send value if connected and notifying.
    if ((m_accelerometer_service.conn_handle != BLE_CONN_HANDLE_INVALID)) 
    {
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = m_accelerometer_service.accelerometer_saved_vehicle_length_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        err_code = sd_ble_gatts_hvx(m_accelerometer_service.conn_handle, &hvx_params);
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}


uint32_t ble_accelerometer_service_saved_vehicle_width_update(float width)
{
    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = sizeof(float);
    gatts_value.offset  = 0;
    gatts_value.p_value = (uint8_t*)&width;

    // Update database.
    err_code= sd_ble_gatts_value_set(m_accelerometer_service.conn_handle,
                                        m_accelerometer_service.accelerometer_saved_vehicle_width_handles.value_handle,
                                        &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Send value if connected and notifying.
    if ((m_accelerometer_service.conn_handle != BLE_CONN_HANDLE_INVALID)) 
    {
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = m_accelerometer_service.accelerometer_saved_vehicle_width_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        err_code = sd_ble_gatts_hvx(m_accelerometer_service.conn_handle, &hvx_params);
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}

/**@brief Function for handling the Accelerometer Service Service events.
 *
 * @details This function will be called for all Accelerometer Service events which are passed to
 *          the application.
 *
 * @param[in]   p_accelerometer_service  Accelerometer Service structure.
 * @param[in]   p_evt          Event received from the Accelerometer Service.
 *
 */
void ble_accelerometer_on_accelerometer_evt(ble_accelerometer_service_t * p_accelerometer_service, ble_accelerometer_evt_t * p_evt)
{
    ret_code_t err_code;
    switch(p_evt->evt_type)
    {
        case BLE_ACCELEROMETER_EVT_NOTIFICATION_ENABLED:
            break;

        case BLE_ACCELEROMETER_EVT_NOTIFICATION_DISABLED:      
            break;

        case BLE_ACCELEROMETER_EVT_CONNECTED:
            break;

        case BLE_ACCELEROMETER_EVT_DISCONNECTED:
              break;

        default:
              // No implementation needed.
              break;
    }
}

uint32_t ble_accelerometer_service_sensor_data_set(uint8_t *custom_value, uint8_t custom_value_length)
{
    return ble_accelerometer_service_sensor_data_update(&m_accelerometer_service, custom_value, custom_value_length); 
}

uint32_t ble_accelerometer_service_angles_set(float *angles)
{
    return ble_accelerometer_service_angles_update(&m_accelerometer_service, (uint8_t *)angles, 12); 
}

void ble_accelerometer_service_set_calibration_value_received_callback(void (*func)(uint8_t value))
{
    mCalibrationValueReceivedHandler = func;
}

void ble_accelerometer_service_set_orientation_received_handler(void (*func)(uint16_t orientation ))
{
    mOrientationValueReceivedHandler = func;
}

void ble_accelerometer_service_set_vehicle_length_received_handler(void (*func)(uint16_t length ))
{
    mVehicleLengthValueReceivedHandler = func;
}

void ble_accelerometer_service_set_vehicle_width_received_handler(void (*func)(uint16_t width ))
{
    mVehicleWidthValueReceivedHandler = func;
}

void ble_accelerometer_service_set_leveling_mode_received_handler(void (*func)(uint16_t levelingMode ))
{
    mLevelingModeReceivedHandler = func;
}



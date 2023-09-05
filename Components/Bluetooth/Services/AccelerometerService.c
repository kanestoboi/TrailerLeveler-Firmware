#include "sdk_common.h"
#include "ble_srv_common.h"
#include "AccelerometerService.h"
#include <string.h>
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_log.h"
#include "math.h"
#include "nrf_fstorage.h"
#include "nrf_fstorage_sd.h"
#include "nrf_log_ctrl.h"

#pragma pack(4)

#define _RAD_TO_DEG 57.2957795131f  // Constant to convert radians to degrees
#define _PI 3.14159265359f           // Constant for the value of pi

float mLastSentAngles[3] = {0.0, 0.0, 0.0};

static uint32_t accelerometer_value_char_add(ble_accelerometer_service_t * p_accelerometer_service, const ble_accelerometer_service_init_t * p_ble_accelerometer_service_init, const accelerometer_t accelerometer);
static uint32_t accelerometer_angles_char_add(ble_accelerometer_service_t * p_accelerometer_service, const ble_accelerometer_service_init_t * p_ble_accelerometer_service_init);
static uint32_t accelerometer_orientation_char_add(ble_accelerometer_service_t * p_accelerometer_service, const ble_accelerometer_service_init_t * p_ble_accelerometer_service_init);
static uint32_t accelerometer_calibration_char_add(ble_accelerometer_service_t * p_accelerometer_service, const ble_accelerometer_service_init_t * p_ble_accelerometer_service_init);
static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt);
void showCurrentSavedParameters();

typedef struct savedParameters_t
{
    float anglesCalibrationOffsets[3];
    uint32_t orientation;
    float savedHitchHeight;
} savedParameters_t;

savedParameters_t savedParameters;

NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage) =
{
    /* Set a handler for fstorage events. */
    .evt_handler = fstorage_evt_handler,

    /* These below are the boundaries of the flash space assigned to this instance of fstorage.
     * You must set these manually, even at runtime, before nrf_fstorage_init() is called.
     * The function nrf5_flash_end_addr_get() can be used to retrieve the last address on the
     * last page of flash available to write data. */
    .start_addr = 0xF7000,
    .end_addr   = 0xF8000,
};


static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt)
{
    if (p_evt->result != NRF_SUCCESS)
    {
        NRF_LOG_INFO("--> Event received: ERROR while executing an fstorage operation.");
        return;
    }

    switch (p_evt->id)
    {
        case NRF_FSTORAGE_EVT_WRITE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: wrote %d bytes at address 0x%x.",
                         p_evt->len, p_evt->addr);

            showCurrentSavedParameters();

        } break;

        case NRF_FSTORAGE_EVT_ERASE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: erased %d page from address 0x%x.",
                         p_evt->len, p_evt->addr);

            //NRF_LOG_INFO("Writing \"%x\" to flash.", savedParameters);
            ret_code_t err_code = nrf_fstorage_write(&fstorage, 0xF7000, &savedParameters, sizeof(savedParameters_t), NULL);
            APP_ERROR_CHECK(err_code);
        } break;

        default:
            break;
    }
}



uint32_t ble_acceleration_service_init(ble_accelerometer_service_t * p_accelerometer_service, const ble_accelerometer_service_init_t * p_ble_accelerometer_service_init, const accelerometer_t accelerometer)
{
    if (p_accelerometer_service == NULL || p_ble_accelerometer_service_init == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    p_accelerometer_service->conn_handle = BLE_CONN_HANDLE_INVALID;

    ble_uuid128_t base_uuid = {ACCELEROMETER_SERVICE_UUID_BASE};
    err_code =  sd_ble_uuid_vs_add(&base_uuid, &p_accelerometer_service->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_accelerometer_service->uuid_type;
    ble_uuid.uuid = ACCELEROMETER_SERVICE_UUID;

    // Add the Custom Service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_accelerometer_service->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Initialize service structure
    p_accelerometer_service->evt_handler           = p_ble_accelerometer_service_init->evt_handler;
    p_accelerometer_service->conn_handle           = BLE_CONN_HANDLE_INVALID;


    nrf_fstorage_api_t * p_fs_api;
    p_fs_api = &nrf_fstorage_sd;

    err_code = nrf_fstorage_init(&fstorage, p_fs_api, NULL);
    APP_ERROR_CHECK(err_code);

    //NRF_LOG_RAW_INFO("\n\nParameters on AccelerometerService Initialisation");

    showCurrentSavedParameters();

    
    uint32_t intValue = *(uint32_t*)&savedParameters.anglesCalibrationOffsets[0];;



    if (intValue == 0xFFFFFFFF) {
    // The values are considered equal within the specified tolerance
    // Do something here
        savedParameters.anglesCalibrationOffsets[0] = 0.0;
        savedParameters.anglesCalibrationOffsets[1] = 0.0;
        savedParameters.anglesCalibrationOffsets[2] = 0.0;
    }

    if (savedParameters.orientation == 0xFFFFFFFF)
    {
        savedParameters.orientation = 1;
    }

    intValue = *(uint32_t*)&savedParameters.savedHitchHeight;
    
    if (intValue == 0xFFFFFFFF)
    {
        savedParameters.savedHitchHeight = 0.0;
    }

    nrf_fstorage_erase(&fstorage, 0xF7000, 1, NULL);


    //NRF_LOG_RAW_INFO("\n\nParameters on Before adding characteristics");
    //showCurrentSavedParameters();


    NRF_LOG_FLUSH();

    // Add accelerometer value characteristic
    err_code = accelerometer_value_char_add(p_accelerometer_service, p_ble_accelerometer_service_init, accelerometer);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    err_code = accelerometer_angles_char_add(p_accelerometer_service, p_ble_accelerometer_service_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = accelerometer_orientation_char_add(p_accelerometer_service, p_ble_accelerometer_service_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = accelerometer_calibration_char_add(p_accelerometer_service, p_ble_accelerometer_service_init);
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
uint32_t accelerometer_value_char_add(ble_accelerometer_service_t * p_accelerometer_service, const ble_accelerometer_service_init_t * p_ble_accelerometer_service_init, const accelerometer_t accelerometer)
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

    attr_md.read_perm  = p_ble_accelerometer_service_init->accelerometer_value_char_attr_md.read_perm;
    attr_md.write_perm = p_ble_accelerometer_service_init->accelerometer_value_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    ble_uuid.type = p_accelerometer_service->uuid_type;

    if (accelerometer == ACCELEROMETER_ADXL355)
    {
      ble_uuid.uuid = ACCELEROMETER_ADXL355_VALUE_CHAR_UUID;
    }
    else if (accelerometer == ACCELEROMETER_MPU6050)
    {
      ble_uuid.uuid = ACCELEROMETER_MPU6050_VALUE_CHAR_UUID;
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
    else if (accelerometer == ACCELEROMETER_MPU6050)
    {
      attr_char_value.max_len   = 6*sizeof(uint8_t);
    }
    

    err_code = sd_ble_gatts_characteristic_add(p_accelerometer_service->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_accelerometer_service->accelerometer_value_handles);
    
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
uint32_t accelerometer_angles_char_add(ble_accelerometer_service_t * p_accelerometer_service, const ble_accelerometer_service_init_t * p_ble_accelerometer_service_init)
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

    attr_md.read_perm  = p_ble_accelerometer_service_init->accelerometer_value_char_attr_md.read_perm;
    attr_md.write_perm = p_ble_accelerometer_service_init->accelerometer_value_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    ble_uuid.type = p_accelerometer_service->uuid_type;

    ble_uuid.uuid = ACCELEROMETER_ANGLE_CHAR_UUID;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 6*sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    
    attr_char_value.max_len   = 12*sizeof(uint8_t);


    err_code = sd_ble_gatts_characteristic_add(p_accelerometer_service->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_accelerometer_service->accelerometer_angles_handles);
    
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
uint32_t accelerometer_orientation_char_add(ble_accelerometer_service_t * p_accelerometer_service, const ble_accelerometer_service_init_t * p_ble_accelerometer_service_init)
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

    attr_md.read_perm  = p_ble_accelerometer_service_init->accelerometer_value_char_attr_md.read_perm;
    attr_md.write_perm = p_ble_accelerometer_service_init->accelerometer_value_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    ble_uuid.type = p_accelerometer_service->uuid_type;

    ble_uuid.uuid = ACCELEROMETER_ORIENTATION_CHAR_UUID;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 1*sizeof(uint8_t);
    attr_char_value.init_offs = 0;

    uint8_t orientationValue = savedParameters.orientation;
        
    attr_char_value.p_value   = &orientationValue; // Pointer to the initial value

    attr_char_value.max_len   = 1*sizeof(uint8_t);

    err_code = sd_ble_gatts_characteristic_add(p_accelerometer_service->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_accelerometer_service->accelerometer_orientation_handles);
    
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
uint32_t accelerometer_calibration_char_add(ble_accelerometer_service_t * p_accelerometer_service, const ble_accelerometer_service_init_t * p_ble_accelerometer_service_init)
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

    attr_md.read_perm  = p_ble_accelerometer_service_init->accelerometer_value_char_attr_md.read_perm;
    attr_md.write_perm = p_ble_accelerometer_service_init->accelerometer_value_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    ble_uuid.type = p_accelerometer_service->uuid_type;

    ble_uuid.uuid = ACCELEROMETER_CALIBRATION_CHAR_UUID;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 1*sizeof(uint8_t);
    attr_char_value.init_offs = 0;
        
    attr_char_value.p_value   = (uint8_t*)&savedParameters.orientation; // Pointer to the initial value

    attr_char_value.max_len   = 1*sizeof(uint8_t);

    err_code = sd_ble_gatts_characteristic_add(p_accelerometer_service->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_accelerometer_service->accelerometer_calibration_handles);
    
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
    if (p_evt_write->handle == p_accelerometer_service->accelerometer_value_handles.value_handle)
    {
        // Put specific task here. 
        NRF_LOG_INFO("Message Received.");
    }

    // Check if the Custom value CCCD is written to and that the value is the appropriate length, i.e 2 bytes.
    if ((p_evt_write->handle == p_accelerometer_service->accelerometer_value_handles.cccd_handle)
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

        //memcpy(&mAccelerometerOrientation, p_evt_write->data, sizeof(uint8_t));

        uint32_t valToWrite = (uint32_t)p_evt_write->data[0];

        NRF_LOG_INFO("Writing orientation \"%x\" to flash.", valToWrite);


        savedParameters.orientation = valToWrite;

        ret_code_t err_code = nrf_fstorage_erase(&fstorage, 0xF7000, 1, NULL);
        APP_ERROR_CHECK(err_code);

    }

        /// Check if the handle passed with the event matches the Orientation Characteristic handle.
    if ((p_evt_write->handle == p_accelerometer_service->accelerometer_calibration_handles.value_handle)
        && (p_evt_write->len == 1)
       )
    {
        NRF_LOG_INFO("Message Received from calibration.");


        NRF_LOG_INFO("mLastSentAngles.");
        NRF_LOG_RAW_INFO("y:" NRF_LOG_FLOAT_MARKER ", ", NRF_LOG_FLOAT(mLastSentAngles[0]) );
        NRF_LOG_RAW_INFO("y:" NRF_LOG_FLOAT_MARKER ", ", NRF_LOG_FLOAT(mLastSentAngles[1]) );
        NRF_LOG_RAW_INFO("y:" NRF_LOG_FLOAT_MARKER ", \n", NRF_LOG_FLOAT(mLastSentAngles[2]) );

        
        //NRF_LOG_FLUSH();

        static uint32_t temp          = 0xBADC0FFE;

        savedParameters.anglesCalibrationOffsets[0] = mLastSentAngles[0];
        savedParameters.anglesCalibrationOffsets[1] = mLastSentAngles[1];
        savedParameters.anglesCalibrationOffsets[2] = mLastSentAngles[2];


        ret_code_t err_code = nrf_fstorage_erase(&fstorage, 0xF7000, 1, NULL);

        uint8_t val = 0;

        ble_accelerometer_service_calibration_update(&m_accelerometer, &val, 1);        
    }
};

uint32_t ble_accelerometer_service_value_update(ble_accelerometer_service_t * p_accelerometer_service, uint8_t *custom_value, uint8_t custom_value_length)
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
                                        p_accelerometer_service->accelerometer_value_handles.value_handle,
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

        hvx_params.handle = p_accelerometer_service->accelerometer_value_handles.value_handle;
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

uint32_t ble_accelerometer_service_orientation_update(ble_accelerometer_service_t * p_accelerometer_service, uint8_t *custom_value, uint8_t custom_value_length)
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
                                        p_accelerometer_service->accelerometer_orientation_handles.value_handle,
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

        hvx_params.handle = p_accelerometer_service->accelerometer_orientation_handles.value_handle;
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

/**@brief Function for handling the Custom Service Service events.
 *
 * @details This function will be called for all Custom Service events which are passed to
 *          the application.
 *
 * @param[in]   p_cus_service  Custom Service structure.
 * @param[in]   p_evt          Event received from the Custom Service.
 *
 */
void on_accelerometer_evt(ble_accelerometer_service_t * p_accelerometer_service, ble_accelerometer_evt_t * p_evt)
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

uint32_t ble_accelerometer_service_value_set(uint8_t *custom_value, uint8_t custom_value_length)
{
    return ble_accelerometer_service_value_update(&m_accelerometer, custom_value, custom_value_length); 
}

uint32_t ble_accelerometer_service_angles_set(uint8_t *custom_value, uint8_t custom_value_length)
{
    memcpy(mLastSentAngles, custom_value, custom_value_length); 

    static float myFloat[3];

    myFloat[0] = mLastSentAngles[0] -  savedParameters.anglesCalibrationOffsets[0];
    myFloat[1] = mLastSentAngles[1] -  savedParameters.anglesCalibrationOffsets[1];
    myFloat[2] = mLastSentAngles[2] -  savedParameters.anglesCalibrationOffsets[2];

    NRF_LOG_RAW_INFO("x" NRF_LOG_FLOAT_MARKER ", ", NRF_LOG_FLOAT(myFloat[0]) ); // display the read values
    NRF_LOG_RAW_INFO("y:" NRF_LOG_FLOAT_MARKER ", ", NRF_LOG_FLOAT(myFloat[1]) ); // display the read values
    NRF_LOG_RAW_INFO("z:" NRF_LOG_FLOAT_MARKER " ", NRF_LOG_FLOAT(myFloat[2]) ); // display the read values

    NRF_LOG_RAW_INFO("\n");
    NRF_LOG_FLUSH();

    return ble_accelerometer_service_angles_update(&m_accelerometer, (uint8_t *)myFloat, custom_value_length); 
}

void calculateAnglesFromDeviceOrientation(float angleX, float angleY, float angleZ, float *angles) {
    switch (savedParameters.orientation) {
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

void showCurrentSavedParameters()
{
    //uint32_t memPageStart = 247; // Assuming this is a valid page number
    uint32_t mrmAddrPtr = (0xF7000);

    // Assuming your struct is saved at memory location 0xF7000
    uint8_t *memoryLocation = (uint8_t *) mrmAddrPtr;

    // Use memcpy to copy the data from memoryLocation into savedParameters
    memcpy(&savedParameters, memoryLocation, sizeof(savedParameters_t));

    /*NRF_LOG_RAW_INFO("x:" NRF_LOG_FLOAT_MARKER ", ", NRF_LOG_FLOAT(0.0) );
    NRF_LOG_RAW_INFO("y:" NRF_LOG_FLOAT_MARKER ", ", NRF_LOG_FLOAT(savedParameters.anglesCalibrationOffsets[1]) );
    NRF_LOG_RAW_INFO("z:" NRF_LOG_FLOAT_MARKER ", \n", NRF_LOG_FLOAT(savedParameters.anglesCalibrationOffsets[2]) );

    NRF_LOG_RAW_INFO("orientation: %d\n", savedParameters.orientation);
    
    NRF_LOG_RAW_INFO("saved hitch height: " NRF_LOG_FLOAT_MARKER ", \n", NRF_LOG_FLOAT(savedParameters.savedHitchHeight) );

    NRF_LOG_FLUSH();*/
}
#include "sdk_common.h"
#include "ble_srv_common.h"
#include "AccelerometerService.h"
#include <string.h>
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_log.h"

static uint32_t accelerometer_value_char_add(ble_accelerometer_service_t * p_accelerometer_service, const ble_accelerometer_service_init_t * p_ble_accelerometer_service_init, const accelerometer_t accelerometer);

/**@brief   Macro for defining a ble_accelerometer instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
                                                                                    

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

    // Add accelerometer value characteristic
    return accelerometer_value_char_add(p_accelerometer_service, p_ble_accelerometer_service_init, accelerometer);
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
                                               &p_accelerometer_service->accerometer_value_handles);
    
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
    if (p_evt_write->handle == p_accelerometer_service->accerometer_value_handles.value_handle)
    {
        // Put specific task here. 
        NRF_LOG_INFO("Message Received.");
    }

    // Check if the Custom value CCCD is written to and that the value is the appropriate length, i.e 2 bytes.
    if ((p_evt_write->handle == p_accelerometer_service->accerometer_value_handles.cccd_handle)
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
};

uint32_t err_codess;
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
    err_codess= sd_ble_gatts_value_set(p_accelerometer_service->conn_handle,
                                        p_accelerometer_service->accerometer_value_handles.value_handle,
                                        &gatts_value);
    if (err_codess != NRF_SUCCESS)
    {
        return err_codess;
    }

    // Send value if connected and notifying.
    if ((p_accelerometer_service->conn_handle != BLE_CONN_HANDLE_INVALID)) 
    {
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_accelerometer_service->accerometer_value_handles.value_handle;
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
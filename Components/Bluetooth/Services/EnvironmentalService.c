#include "sdk_common.h"
#include "ble_srv_common.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_log.h"
#include "math.h"
#include "nrf_log_ctrl.h"

#include "EnvironmentalService.h"

static uint32_t ess_sensor_data_char_add(ble_ess_service_t * p_ess_service, const ble_ess_service_init_t * p_ble_ess_service_init);
static uint32_t ess_temperature_char_add(ble_ess_service_t * p_ess_service, const ble_ess_service_init_t * p_ble_ess_service_init);
static uint32_t ess_orientation_char_add(ble_ess_service_t * p_ess_service, const ble_ess_service_init_t * p_ble_ess_service_init);
static uint32_t ess_calibration_char_add(ble_ess_service_t * p_ess_service, const ble_ess_service_init_t * p_ble_ess_service_init);
static uint32_t ess_saved_hitch_angle_char_add(ble_ess_service_t * p_ess_service, const ble_ess_service_init_t * p_ble_ess_service_init);

BLE_ESS_DEF(m_ess);

uint32_t ble_ess_service_init()
{
    ble_ess_service_init_t ess_service_init;
    
    // Initialize Environmental Sensor Service init structure to zero.
    memset(&ess_service_init, 0, sizeof(ess_service_init));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&ess_service_init.ess_temperature_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&ess_service_init.ess_temperature_char_attr_md.write_perm);

    // Set the ess event handler
    ess_service_init.evt_handler = ble_ess_on_ess_evt;

    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    m_ess.conn_handle = BLE_CONN_HANDLE_INVALID;

    ble_uuid128_t base_uuid = {ENVIRONMENTAL_SENSING_SERVICE_UUID};
    err_code =  sd_ble_uuid_vs_add(&base_uuid, &m_ess.uuid_type);
    VERIFY_SUCCESS(err_code);

    // Assign the ESS UUID to the uuid object
    BLE_UUID_BLE_ASSIGN(ble_uuid, ENVIRONMENTAL_SENSING_SERVICE_UUID);

    // Add the Environmental Sensor Service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &m_ess.service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Initialize service structure
    m_ess.evt_handler           = ble_ess_on_ess_evt;
    m_ess.conn_handle           = BLE_CONN_HANDLE_INVALID;

    // Add ess value characteristic to the ess service
    err_code = ess_temperature_char_add(&m_ess, &ess_service_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}    

/**@brief Function for adding the temperature characteristic.
 *
 * @param[in]   p_ess_service               Environmental Sensor Service structure.
 * @param[in]   p_ble_ess_service_init      Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ess_temperature_char_add(ble_ess_service_t * p_ess_service, const ble_ess_service_init_t * p_ble_ess_service_init)
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

    attr_md.read_perm  = p_ble_ess_service_init->ess_temperature_char_attr_md.read_perm;
    attr_md.write_perm = p_ble_ess_service_init->ess_temperature_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    BLE_UUID_BLE_ASSIGN(ble_uuid, TEMPERATURE_CHAR_UUID);


    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 2*sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = 2*sizeof(uint8_t);
    

    err_code = sd_ble_gatts_characteristic_add(p_ess_service->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_ess_service->ess_temperature_handles);
    
    return err_code;
}

void ble_ess_on_ble_evt( ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_ess_service_t * p_ess_service = (ble_ess_service_t *) p_context;
    
    if (p_ess_service == NULL || p_ble_evt == NULL)
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            ble_ess_on_connect(p_ess_service, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            ble_ess_on_disconnect(p_ess_service, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            ble_ess_on_write(p_ess_service, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_ess_service       Environmental Sensor Service structure.
 * @param[in]   p_ble_evt           Event received from the BLE stack.
 */
static void ble_ess_on_connect(ble_ess_service_t * p_ess_service, ble_evt_t const * p_ble_evt)
{
    p_ess_service->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

    ble_ess_evt_t evt;

    evt.evt_type = BLE_ESS_EVT_CONNECTED;
    
    // check if the ess service ble event handler has been initialised before calling it
    if (p_ess_service->evt_handler == NULL)
    {
        return;
    }

    p_ess_service->evt_handler(p_ess_service, &evt);
}

/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_ess_service       Environmental Sensor Service structure.
 * @param[in]   p_ble_evt           Event received from the BLE stack.
 */
static void ble_ess_on_disconnect(ble_ess_service_t * p_ess_service, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_ess_service->conn_handle = BLE_CONN_HANDLE_INVALID;
}

/**@brief Function for handling the Write event.
 *
 * @param[in]   p_ess_service       Environmental Sensor Service.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void ble_ess_on_write(ble_ess_service_t * p_ess_service, ble_evt_t const * p_ble_evt)
{
    const ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    
    // Check if the handle passed with the event matches the Temperature Characteristic handle.
    if (p_evt_write->handle == p_ess_service->ess_temperature_handles.value_handle)
    {
        // Put specific task here. 
        NRF_LOG_INFO("Message Received.");
    }
};

uint32_t ble_ess_service_temperature_update(ble_ess_service_t * p_ess_service, float *temperture)
{
    if (p_ess_service == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;
    int16_t temperatureToWriteToCharacteristic = (int16_t)(*temperture * 100.0f);

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = sizeof(int16_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = (uint8_t*)&temperatureToWriteToCharacteristic;

    // Update database.
    err_code= sd_ble_gatts_value_set(p_ess_service->conn_handle,
                                        p_ess_service->ess_temperature_handles.value_handle,
                                        &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Send value if connected and notifying.
    if ((p_ess_service->conn_handle != BLE_CONN_HANDLE_INVALID)) 
    {
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_ess_service->ess_temperature_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        err_code = sd_ble_gatts_hvx(p_ess_service->conn_handle, &hvx_params);
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}

/**@brief Function for handling the ess Service Service events.
 *
 * @details This function will be called for all ess Service events which are passed to
 *          the application.
 *
 * @param[in]   p_ess_service  Environmental Sensor Service Service structure.
 * @param[in]   p_evt          Event received from the ess Service.
 *
 */
void ble_ess_on_ess_evt(ble_ess_service_t * p_ess_service, ble_ess_evt_t * p_evt)
{
    ret_code_t err_code;
    switch(p_evt->evt_type)
    {
        case BLE_ESS_EVT_NOTIFICATION_ENABLED:
            break;

        case BLE_ESS_EVT_NOTIFICATION_DISABLED:      
            break;

        case BLE_ESS_EVT_CONNECTED:
            break;

        case BLE_ESS_EVT_DISCONNECTED:
              break;

        default:
              // No implementation needed.
              break;
    }
}

uint32_t ble_ess_service_temperature_set(float *temperature)
{
    return ble_ess_service_temperature_update(&m_ess, temperature); 
}

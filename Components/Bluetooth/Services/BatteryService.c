#include "sdk_common.h"
#include "BatteryService.h"
#include <string.h>
#include "ble_srv_common.h"
#include "ble_conn_state.h"
#include "nrf_log.h"

#define INVALID_BATTERY_LEVEL 255

BATTERY_SERVICE_DEF(m_battery_service);

/**@brief Function for handling the Write event.
 *
 * @param[in]   p_bas       Battery Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(battery_service_t * p_battery_service, ble_evt_t const * p_ble_evt)
{
    if (!p_battery_service->is_notification_supported)
    {
        return;
    }

    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (    (p_evt_write->handle == p_battery_service->battery_level_handles.cccd_handle)
        &&  (p_evt_write->len == 2))
    {
        if (p_battery_service->evt_handler == NULL)
        {
            return;
        }

        battery_service_evt_t evt;

        if (ble_srv_is_notification_enabled(p_evt_write->data))
        {
            evt.evt_type = BATTERY_SERVICE_EVT_NOTIFICATION_ENABLED;
        }
        else
        {
            evt.evt_type = BATTERY_SERVICE_EVT_NOTIFICATION_DISABLED;
        }
        evt.conn_handle = p_ble_evt->evt.gatts_evt.conn_handle;

        // CCCD written, call application event handler.
        p_battery_service->evt_handler(p_battery_service, &evt);
    }
}


void battery_service_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    if ((p_context == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    battery_service_t * p_battery_service = (battery_service_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTS_EVT_WRITE:
            on_write(p_battery_service, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for adding the Battery Level characteristic.
 *
 * @param[in]   p_bas        Battery Service structure.
 * @param[in]   p_bas_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static ret_code_t battery_service_battery_level_char_add(const battery_service_init_t * p_battery_service_init)
{
    ret_code_t             err_code;
    ble_add_char_params_t  add_char_params;
    ble_add_descr_params_t add_descr_params;
    uint8_t                initial_battery_level;
    uint8_t                init_len;
    uint8_t                encoded_report_ref[BLE_SRV_ENCODED_REPORT_REF_LEN];

    // Add battery level characteristic
    initial_battery_level = p_battery_service_init->initial_batt_level;

    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid              = BLE_UUID_BATTERY_LEVEL_CHAR;
    add_char_params.max_len           = sizeof(uint8_t);
    add_char_params.init_len          = sizeof(uint8_t);
    add_char_params.p_init_value      = &initial_battery_level;
    add_char_params.char_props.notify = m_battery_service.is_notification_supported;
    add_char_params.char_props.read   = 1;
    add_char_params.cccd_write_access = p_battery_service_init->bl_cccd_wr_sec;
    add_char_params.read_access       = p_battery_service_init->bl_rd_sec;

    err_code = characteristic_add(m_battery_service.service_handle,
                                  &add_char_params,
                                  &(m_battery_service.battery_level_handles));
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    if (p_battery_service_init->p_report_ref != NULL)
    {
        // Add Report Reference descriptor
        init_len = ble_srv_report_ref_encode(encoded_report_ref, p_battery_service_init->p_report_ref);

        memset(&add_descr_params, 0, sizeof(add_descr_params));
        add_descr_params.uuid        = BLE_UUID_REPORT_REF_DESCR;
        add_descr_params.read_access = p_battery_service_init->bl_report_rd_sec;
        add_descr_params.init_len    = init_len;
        add_descr_params.max_len     = add_descr_params.init_len;
        add_descr_params.p_value     = encoded_report_ref;

        err_code = descriptor_add(m_battery_service.battery_level_handles.value_handle,
                                  &add_descr_params,
                                  &m_battery_service.report_ref_handle);
        return err_code;
    }
    else
    {
        m_battery_service.report_ref_handle = BLE_GATT_HANDLE_INVALID;
    }

    return NRF_SUCCESS;
}


/**@brief Function for adding the Battery Level characteristic.
 *
 * @param[in]   p_bas        Battery Service structure.
 * @param[in]   p_bas_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static ret_code_t battery_service_battery_time_status_char_add(const battery_service_init_t * p_battery_service_init)
{
    ret_code_t             err_code;
    ble_add_char_params_t  add_char_params;
    ble_add_descr_params_t add_descr_params;
    uint8_t                initial_battery_level;
    uint8_t                init_len;
    uint8_t                encoded_report_ref[BLE_SRV_ENCODED_REPORT_REF_LEN];

    // Add battery level characteristic
    initial_battery_level = p_battery_service_init->initial_batt_level;

    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid              = BLE_UUID_BATTERY_TIME_STATUS_CHAR;
    add_char_params.max_len           = sizeof(battery_time_status_t);
    add_char_params.init_len          = sizeof(battery_time_status_t);
    add_char_params.p_init_value      = &initial_battery_level;
    add_char_params.char_props.notify = m_battery_service.is_notification_supported;
    add_char_params.char_props.read   = 1;
    add_char_params.cccd_write_access = p_battery_service_init->bl_cccd_wr_sec;
    add_char_params.read_access       = p_battery_service_init->bl_rd_sec;

    err_code = characteristic_add(m_battery_service.service_handle,
                                  &add_char_params,
                                  &(m_battery_service.battery_time_status_handles));
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    if (p_battery_service_init->p_report_ref != NULL)
    {
        // Add Report Reference descriptor
        init_len = ble_srv_report_ref_encode(encoded_report_ref, p_battery_service_init->p_report_ref);

        memset(&add_descr_params, 0, sizeof(add_descr_params));
        add_descr_params.uuid        = BLE_UUID_REPORT_REF_DESCR;
        add_descr_params.read_access = p_battery_service_init->bl_report_rd_sec;
        add_descr_params.init_len    = init_len;
        add_descr_params.max_len     = add_descr_params.init_len;
        add_descr_params.p_value     = encoded_report_ref;

        err_code = descriptor_add(m_battery_service.battery_time_status_handles.value_handle,
                                  &add_descr_params,
                                  &m_battery_service.report_ref_handle);
        return err_code;
    }
    else
    {
        m_battery_service.report_ref_handle = BLE_GATT_HANDLE_INVALID;
    }

    return NRF_SUCCESS;
}


ret_code_t battery_service_init()
{
    // Initialize Battery Service.
    battery_service_init_t     battery_service_init;
    memset(&battery_service_init, 0, sizeof(battery_service_init));

    // Here the sec level for the Battery Service can be changed/increased.
    battery_service_init.bl_rd_sec        = SEC_OPEN;
    battery_service_init.bl_cccd_wr_sec   = SEC_OPEN;
    battery_service_init.bl_report_rd_sec = SEC_OPEN;

    battery_service_init.evt_handler          = NULL;
    battery_service_init.support_notification = true;
    battery_service_init.p_report_ref         = NULL;
    battery_service_init.initial_batt_level   = 100;

    ret_code_t err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    m_battery_service.evt_handler               = battery_service_init.evt_handler;
    m_battery_service.is_notification_supported = battery_service_init.support_notification;
    m_battery_service.battery_level_last        = INVALID_BATTERY_LEVEL;

    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_BATTERY_SERVICE);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &m_battery_service.service_handle);
    VERIFY_SUCCESS(err_code);

    // Add battery level characteristic
    err_code = battery_service_battery_level_char_add(&battery_service_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    battery_service_battery_time_status_char_add(&battery_service_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return err_code;
}


/**@brief Function for sending notifications with the Battery Level characteristic.
 *
 * @param[in]   p_hvx_params Pointer to structure with notification data.
 * @param[in]   conn_handle  Connection handle.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static ret_code_t battery_service_battery_notification_send(ble_gatts_hvx_params_t * const p_hvx_params,
                                            uint16_t                       conn_handle)
{
    ret_code_t err_code = sd_ble_gatts_hvx(conn_handle, p_hvx_params);
    if (err_code == NRF_SUCCESS)
    {
        NRF_LOG_INFO("Battery notification has been sent using conn_handle: 0x%04X", conn_handle);
    }
    else
    {
        NRF_LOG_DEBUG("Error: 0x%08X while sending notification with conn_handle: 0x%04X",
                      err_code,
                      conn_handle);
    }
    return err_code;
}



ret_code_t battery_service_battery_level_update(uint8_t battery_level, uint16_t conn_handle)
{
    ret_code_t         err_code = NRF_SUCCESS;
    ble_gatts_value_t  gatts_value;

    if (battery_level != m_battery_service.battery_level_last)
    {
        // Initialize value struct.
        memset(&gatts_value, 0, sizeof(gatts_value));

        gatts_value.len     = sizeof(uint8_t);
        gatts_value.offset  = 0;
        gatts_value.p_value = &battery_level;

        // Update database.
        err_code = sd_ble_gatts_value_set(BLE_CONN_HANDLE_INVALID,
                                          m_battery_service.battery_level_handles.value_handle,
                                          &gatts_value);
        if (err_code == NRF_SUCCESS)
        {
            //NRF_LOG_INFO("Battery level has been updated: %d%%", battery_level)

            // Save new battery value.
            m_battery_service.battery_level_last = battery_level;
        }
        else
        {
            NRF_LOG_DEBUG("Error during battery level update: 0x%08X", err_code)

            return err_code;
        }

        // Send value if connected and notifying.
        if (m_battery_service.is_notification_supported)
        {
            ble_gatts_hvx_params_t hvx_params;

            memset(&hvx_params, 0, sizeof(hvx_params));

            hvx_params.handle = m_battery_service.battery_level_handles.value_handle;
            hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset = gatts_value.offset;
            hvx_params.p_len  = &gatts_value.len;
            hvx_params.p_data = gatts_value.p_value;

            if (conn_handle == BLE_CONN_HANDLE_ALL)
            {
                ble_conn_state_conn_handle_list_t conn_handles = ble_conn_state_conn_handles();

                // Try sending notifications to all valid connection handles.
                for (uint32_t i = 0; i < conn_handles.len; i++)
                {
                    if (ble_conn_state_status(conn_handles.conn_handles[i]) == BLE_CONN_STATUS_CONNECTED)
                    {
                        if (err_code == NRF_SUCCESS)
                        {
                            err_code = battery_service_battery_notification_send(&hvx_params,
                                                                 conn_handles.conn_handles[i]);
                        }
                        else
                        {
                            // Preserve the first non-zero error code
                            UNUSED_RETURN_VALUE(battery_service_battery_notification_send(&hvx_params,
                                                                          conn_handles.conn_handles[i]));
                        }
                    }
                }
            }
            else
            {
                err_code = battery_service_battery_notification_send(&hvx_params, conn_handle);
            }
        }
        else
        {
            err_code = NRF_ERROR_INVALID_STATE;
        }
    }

    return err_code;
}


ret_code_t battery_service_battery_time_status_update(battery_time_status_t battery_time_status, uint16_t conn_handle)
{
    ret_code_t         err_code = NRF_SUCCESS;
    ble_gatts_value_t  gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = sizeof(battery_time_status_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = (uint8_t*)&battery_time_status;

    // Update database.
    err_code = sd_ble_gatts_value_set(BLE_CONN_HANDLE_INVALID,
                                      m_battery_service.battery_time_status_handles.value_handle,
                                      &gatts_value);
    if (err_code == NRF_SUCCESS)
    {
        NRF_LOG_INFO(" battery time status has been updated:")
    }
    else
    {
        NRF_LOG_DEBUG("Error during battery time status update: 0x%08X", err_code)

        return err_code;
    }

    // Send value if connected and notifying.
    if (m_battery_service.is_notification_supported)
    {
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = m_battery_service.battery_time_status_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        if (conn_handle == BLE_CONN_HANDLE_ALL)
        {
            ble_conn_state_conn_handle_list_t conn_handles = ble_conn_state_conn_handles();

            // Try sending notifications to all valid connection handles.
            for (uint32_t i = 0; i < conn_handles.len; i++)
            {
                if (ble_conn_state_status(conn_handles.conn_handles[i]) == BLE_CONN_STATUS_CONNECTED)
                {
                    if (err_code == NRF_SUCCESS)
                    {
                        err_code = battery_service_battery_notification_send(&hvx_params,
                                                             conn_handles.conn_handles[i]);
                    }
                    else
                    {
                        // Preserve the first non-zero error code
                        UNUSED_RETURN_VALUE(battery_service_battery_notification_send(&hvx_params,
                                                                      conn_handles.conn_handles[i]));
                    }
                }
            }
        }
        else
        {
            err_code = battery_service_battery_notification_send(&hvx_params, conn_handle);
        }
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }
    

    return err_code;
}


ret_code_t battery_service_battery_lvl_on_reconnection_update(battery_service_t * p_battery_service,
                                                      uint16_t    conn_handle)
{
    if (p_battery_service == NULL)
    {
        return NRF_ERROR_NULL;
    }

    ret_code_t err_code;

    if (p_battery_service->is_notification_supported)
    {
        ble_gatts_hvx_params_t hvx_params;
        uint16_t               len = sizeof(uint8_t);

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_battery_service->battery_level_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &len;
        hvx_params.p_data = &p_battery_service->battery_level_last;

        err_code = battery_service_battery_notification_send(&hvx_params, conn_handle);

        return err_code;
    }

    return NRF_ERROR_INVALID_STATE;
}


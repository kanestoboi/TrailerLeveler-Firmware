#include "sdk_common.h"
#include "ble_srv_common.h"
#include "AccelerometerService.h"
#include <string.h>
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_log.h"


/**@brief   Macro for defining a ble_accelerometer instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
                                                                                    

uint32_t ble_acceleration_service_init(ble_accerometer_service_t * p_accerometer_service, const ble_accerometer_service_init_t * p_ble_accerometer_service_init)
{
    if (p_accerometer_service == NULL || p_ble_accerometer_service_init == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    p_accerometer_service->conn_handle = BLE_CONN_HANDLE_INVALID;

    ble_uuid128_t base_uuid = {ACCELEROMETER_SERVICE_UUID_BASE};
    err_code =  sd_ble_uuid_vs_add(&base_uuid, &p_accerometer_service->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_accerometer_service->uuid_type;
    ble_uuid.uuid = ACCELEROMETER_SERVICE_UUID;

    // Add the Custom Service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_accerometer_service->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
}    

                                                                       
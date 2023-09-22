/*
 * The MIT License (MIT)
 * Copyright (c) 2018 Novel Bits
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 */

#ifndef ENVIRONMENTAL_SERVICE_H
#define ENVIRONMENTAL_SERVICE_H

#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"

#define ENVIRONMENTAL_SENSING_SERVICE_UUID          0x181A  // environmental sensing service uuid
#define TEMPERATURE_CHAR_UUID                       0x2A6E  // temperature characteristic UUID. 
#define BLE_APPEARANCE_TEMPERATURE_SENSOR                1347    // Temperature Sensor  




/**@brief   Macro for defining a ble_accelerometer instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_ESS_DEF(_name)                          \
ble_ess_service_t _name;                                      \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                           \
                     BLE_HRS_BLE_OBSERVER_PRIO,               \
                     ble_ess_on_ble_evt, &_name)

// Forward declaration of the ble_accerometer_service_t type.
typedef struct ble_ess_service_s ble_ess_service_t;


typedef enum
{
    BLE_ESS_EVT_NOTIFICATION_ENABLED,                             /**< Custom value notification enabled event. */
    BLE_ESS_EVT_NOTIFICATION_DISABLED,                            /**< Custom value notification disabled event. */
    BLE_ESS_EVT_DISCONNECTED,
    BLE_ESS_EVT_CONNECTED
} ble_ess_evt_type_t;

/**@brief Custom Service event. */
typedef struct
{
    ble_ess_evt_type_t evt_type;                                  /**< Type of event. */
} ble_ess_evt_t;

/**@brief Custom Service event handler type. */
typedef void (*ble_ess_evt_handler_t) (ble_ess_service_t * p_ess_service, ble_ess_evt_t * p_evt);

/**@brief Custom Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_ess_evt_handler_t         evt_handler;                    /**< Event handler to be called for handling events in the Custom Service. */
    uint8_t                       initial_custom_value;           /**< Initial custom value */
    ble_srv_cccd_security_mode_t  ess_temperature_char_attr_md;     /**< Initial security level for Custom characteristics attribute */
} ble_ess_service_init_t;

/**@brief Accelerometer Service structure. This contains various status information for the service. */
struct ble_ess_service_s
{
    ble_ess_evt_handler_t         evt_handler;                    /**< Event handler to be called for handling events in the Custom Service. */
    uint16_t                        service_handle;                 /**< Handle of Accelerometer Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t      ess_temperature_handles;      /**< Handles related to the Accelerometer Value characteristic. */
    uint16_t                      conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint8_t                       uuid_type; 
};


/**@brief Function for initializing the Custom Service.
 *
 * @param[out]  p_cus       Custom Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_cus_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_ess_service_init();

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Battery Service.
 *
 * @note 
 *
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 * @param[in]   p_context  Custom Service structure.
 */
void ble_ess_on_ble_evt( ble_evt_t const * p_ble_evt, void * p_context);

static void ble_ess_on_connect(ble_ess_service_t * p_ess_service, ble_evt_t const * p_ble_evt);

static void ble_ess_on_disconnect(ble_ess_service_t * p_ess_service, ble_evt_t const * p_ble_evt);

static void ble_ess_on_write(ble_ess_service_t * p_ess_service, ble_evt_t const * p_ble_evt);

void ble_ess_on_ess_evt(ble_ess_service_t * p_ess_service, ble_ess_evt_t * p_evt);

/**@brief Function for updating the custom value.
 *
 * @details The application calls this function when the cutom value should be updated. If
 *          notification has been enabled, the custom value characteristic is sent to the client.
 *
 * @note 
 *       
 * @param[in]   p_cus          Custom Service structure.
 * @param[in]   Custom value 
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_ess_service_sensor_temperature_update(ble_ess_service_t * p_ess_service, uint8_t *custom_value, uint8_t custom_value_length);

void ble_ess_on_temperature_evt(ble_ess_service_t * p_ess_service, ble_ess_evt_t * p_evt);

uint32_t ble_ess_service_temperature_set(float *temperature);

extern ble_ess_service_t m_ess;

#endif /* ACCELEROMETER_SERVICE_H */
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

#ifndef ACCELEROMETER_SERVICE_H
#define ACCELEROMETER_SERVICE_H

#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"



#define ACCELEROMETER_SERVICE_UUID_BASE {0x02, 0x00, 0x12, 0xAC, 0x42, 0x02, 0xEB, 0xA1, 0xED, 0x11, 0xD9, 0x7D, 0x02, 0xF7, 0x49, 0x76}


#define ACCELEROMETER_SERVICE_UUID               0x1400
#define ACCELEROMETER_VALUE_CHAR_UUID            0x1401

#define BLE_ACCELEROMETER_DEF(_name)                        \
static ble_accerometer_service_t _name;                     \


/**@brief Custom Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    uint8_t                       initial_custom_value;           /**< Initial custom value */
    ble_srv_cccd_security_mode_t  accelerometer_value_char_attr_md;     /**< Initial security level for Custom characteristics attribute */
} ble_accerometer_service_init_t;

/**@brief Accelerometer Service structure. This contains various status information for the service. */
struct ble_accerometer_service_s
{
    uint16_t                      service_handle;                 /**< Handle of Accelerometer Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t      accerometer_value_handles;      /**< Handles related to the Accelerometer Value characteristic. */
    uint16_t                      conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint8_t                       uuid_type; 
};

// Forward declaration of the ble_accerometer_service_t type.
typedef struct ble_accerometer_service_s ble_accerometer_service_t;

/**@brief Function for initializing the Custom Service.
 *
 * @param[out]  p_cus       Custom Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_cus_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_acceleration_service_init(ble_accerometer_service_t * p_accerometer_service, const ble_accerometer_service_init_t * p_ble_accerometer_service_init);

#endif /* ACCELEROMETER_SERVICE_H */
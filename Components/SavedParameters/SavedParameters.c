#include "SavedParameters.h"

#include "sdk_common.h"
#include "ble_srv_common.h"
#include <string.h>
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_log.h"
#include "math.h"
#include "nrf_fstorage.h"
#include "nrf_fstorage_sd.h"
#include "nrf_log_ctrl.h"


static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt);

typedef struct SavedParameters_t
{
    float anglesCalibrationOffsets[3];
    uint32_t orientation;
    float savedHitchHeight;
} SavedParameters_t;

SavedParameters_t mSavedParameters;

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

            saved_parameters_copy_saved_parameters_from_flash();

        } break;

        case NRF_FSTORAGE_EVT_ERASE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: erased %d page from address 0x%x.",
                         p_evt->len, p_evt->addr);

            //NRF_LOG_INFO("Writing \"%x\" to flash.", savedParameters);
            ret_code_t err_code = nrf_fstorage_write(&fstorage, 0xF7000, &mSavedParameters, sizeof(SavedParameters_t), NULL);
            APP_ERROR_CHECK(err_code);
        } break;

        default:
            break;
    }
}

void saved_parameters_init()
{
    nrf_fstorage_api_t * p_fs_api;
    p_fs_api = &nrf_fstorage_sd;

    ret_code_t err_code = nrf_fstorage_init(&fstorage, p_fs_api, NULL);
    APP_ERROR_CHECK(err_code);

    //NRF_LOG_RAW_INFO("\n\nParameters on AccelerometerService Initialisation");

    saved_parameters_copy_saved_parameters_from_flash();

    uint32_t intValue = *(uint32_t*)&mSavedParameters.anglesCalibrationOffsets[0];;

    if (intValue == 0xFFFFFFFF) {
    // The values are considered equal within the specified tolerance
    // Do something here
        mSavedParameters.anglesCalibrationOffsets[0] = 0.0;
        mSavedParameters.anglesCalibrationOffsets[1] = 0.0;
        mSavedParameters.anglesCalibrationOffsets[2] = 0.0;
    }

    if (mSavedParameters.orientation == 0xFFFFFFFF)
    {
        mSavedParameters.orientation = 1;
    }

    intValue = *(uint32_t*)&mSavedParameters.savedHitchHeight;
    
    if (intValue == 0xFFFFFFFF)
    {
        mSavedParameters.savedHitchHeight = 0.0;
    }

    nrf_fstorage_erase(&fstorage, 0xF7000, 1, NULL);
}

void saved_parameters_getSavedCalibrationAngles(float * angles)
{
    angles[0] = mSavedParameters.anglesCalibrationOffsets[0];
    angles[1] = mSavedParameters.anglesCalibrationOffsets[1];
    angles[2] = mSavedParameters.anglesCalibrationOffsets[2];
}

uint32_t saved_parameters_getSavedOrientation()
{
    return mSavedParameters.orientation;
}

float saved_parameters_getSavedHitchHeightAngle()
{
    return mSavedParameters.savedHitchHeight;
}


void saved_parameters_SaveAngleOffsets(float * angles)
{
    mSavedParameters.anglesCalibrationOffsets[0] = angles[0];
    mSavedParameters.anglesCalibrationOffsets[1] = angles[1];
    mSavedParameters.anglesCalibrationOffsets[2] = angles[2];

    ret_code_t err_code = nrf_fstorage_erase(&fstorage, 0xF7000, 1, NULL);
    APP_ERROR_CHECK(err_code);
}

void saved_parameters_SaveOrientation(uint32_t orientation)
{
    mSavedParameters.orientation = orientation;

    ret_code_t err_code = nrf_fstorage_erase(&fstorage, 0xF7000, 1, NULL);
    APP_ERROR_CHECK(err_code);
}

void saved_parameters_SaveHitchAngle(float angle)
{
    mSavedParameters.savedHitchHeight = angle;

    ret_code_t err_code = nrf_fstorage_erase(&fstorage, 0xF7000, 1, NULL);
    APP_ERROR_CHECK(err_code);
}

void saved_parameters_copy_saved_parameters_from_flash()
{
    //uint32_t memPageStart = 247; // Assuming this is a valid page number
    uint32_t mrmAddrPtr = (0xF7000);

    // Assuming your struct is saved at memory location 0xF7000
    uint8_t *memoryLocation = (uint8_t *) mrmAddrPtr;

    // Use memcpy to copy the data from memoryLocation into savedParameters
    memcpy(&mSavedParameters, memoryLocation, sizeof(SavedParameters_t));

    /*NRF_LOG_RAW_INFO("x:" NRF_LOG_FLOAT_MARKER ", ", NRF_LOG_FLOAT(0.0) );
    NRF_LOG_RAW_INFO("y:" NRF_LOG_FLOAT_MARKER ", ", NRF_LOG_FLOAT(savedParameters.anglesCalibrationOffsets[1]) );
    NRF_LOG_RAW_INFO("z:" NRF_LOG_FLOAT_MARKER ", \n", NRF_LOG_FLOAT(savedParameters.anglesCalibrationOffsets[2]) );

    NRF_LOG_RAW_INFO("orientation: %d\n", savedParameters.orientation);
    
    NRF_LOG_RAW_INFO("saved hitch height: " NRF_LOG_FLOAT_MARKER ", \n", NRF_LOG_FLOAT(savedParameters.savedHitchHeight) );

    NRF_LOG_FLUSH();*/
}
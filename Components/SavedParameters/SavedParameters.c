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
#include "fds.h"
#include "nrf_log_ctrl.h"


/* Array to map FDS events to strings. */
static char const * fds_evt_str[] =
{
    "FDS_EVT_INIT",
    "FDS_EVT_WRITE",
    "FDS_EVT_UPDATE",
    "FDS_EVT_DEL_RECORD",
    "FDS_EVT_DEL_FILE",
    "FDS_EVT_GC",
};


/* Keep track of the progress of a delete_all operation. */
static struct
{
    bool delete_next;   //!< Delete next record.
    bool pending;       //!< Waiting for an fds FDS_EVT_DEL_RECORD event, to delete the next record.
} m_delete_all;

typedef struct SavedParameters_t
{
    float anglesCalibrationOffsets[3];
    uint32_t orientation;
    float savedHitchHeight;
    float vehicleLength;
    float vehicleWidth;
} SavedParameters_t;

static SavedParameters_t mSavedParameters = 
{
    .anglesCalibrationOffsets[0] = 0.0,
    .anglesCalibrationOffsets[1] = 0.0,
    .anglesCalibrationOffsets[2] = 0.0,
    .orientation = 1,
    .savedHitchHeight = 0.0,
    .vehicleLength = 1.0,
    .vehicleWidth = 1.0,
};

/* A record containing dummy configuration data. */
static fds_record_t const m_dummy_record =
{
    .file_id           = CONFIG_FILE,
    .key               = CONFIG_REC_KEY,
    .data.p_data       = &mSavedParameters,
    /* The length of a record is always expressed in 4-byte units (words). */
    .data.length_words = (sizeof(mSavedParameters) + 3) / sizeof(uint32_t),
};

static void record_update();

const char *fds_err_str(ret_code_t ret)
{
    /* Array to map FDS return values to strings. */
    static char const * err_str[] =
    {
        "FDS_ERR_OPERATION_TIMEOUT",
        "FDS_ERR_NOT_INITIALIZED",
        "FDS_ERR_UNALIGNED_ADDR",
        "FDS_ERR_INVALID_ARG",
        "FDS_ERR_NULL_ARG",
        "FDS_ERR_NO_OPEN_RECORDS",
        "FDS_ERR_NO_SPACE_IN_FLASH",
        "FDS_ERR_NO_SPACE_IN_QUEUES",
        "FDS_ERR_RECORD_TOO_LARGE",
        "FDS_ERR_NOT_FOUND",
        "FDS_ERR_NO_PAGES",
        "FDS_ERR_USER_LIMIT_REACHED",
        "FDS_ERR_CRC_CHECK_FAILED",
        "FDS_ERR_BUSY",
        "FDS_ERR_INTERNAL",
    };

    return err_str[ret - NRF_ERROR_FDS_ERR_BASE];
}

/* Flag to check fds initialization. */
static bool volatile m_fds_initialized;

static void fds_evt_handler(fds_evt_t const * p_evt)
{
    if (p_evt->result == NRF_SUCCESS)
    {
        NRF_LOG_INFO("Event: %s received (NRF_SUCCESS)",
                      fds_evt_str[p_evt->id]);
    }
    else
    {
        NRF_LOG_INFO("Event: %s received (%s)",
                      fds_evt_str[p_evt->id],
                      fds_err_str(p_evt->result));
    }

    switch (p_evt->id)
    {
        case FDS_EVT_INIT:
            if (p_evt->result == NRF_SUCCESS)
            {
                m_fds_initialized = true;
            }
            break;

        case FDS_EVT_WRITE:
        {
            if (p_evt->result == NRF_SUCCESS)
            {
                NRF_LOG_INFO("Record ID:\t0x%04x",  p_evt->write.record_id);
                NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->write.file_id);
                NRF_LOG_INFO("Record key:\t0x%04x", p_evt->write.record_key);
            }
        } break;

        case FDS_EVT_DEL_RECORD:
        {
            if (p_evt->result == NRF_SUCCESS)
            {
                NRF_LOG_INFO("Record ID:\t0x%04x",  p_evt->del.record_id);
                NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->del.file_id);
                NRF_LOG_INFO("Record key:\t0x%04x", p_evt->del.record_key);
            }
            m_delete_all.pending = false;
        } break;

        default:
            break;
    }
}

void saved_parameters_init()
{
    /* Register first to receive an event when initialization is complete. */
    (void) fds_register(fds_evt_handler);

    // Initialise the nRF Flash Data Storage module
    ret_code_t err_code = fds_init();
    APP_ERROR_CHECK(err_code);

    // wait for the nRF Flash Data Storeage module to be initialised
    while (!m_fds_initialized)
    {
    }

    fds_stat_t stat = {0};

    err_code = fds_stat(&stat);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Found %d valid records.", stat.valid_records);
    NRF_LOG_INFO("Found %d dirty records (ready to be garbage collected).", stat.dirty_records);
    NRF_LOG_INFO("%d Pages available.", stat.pages_available);

    fds_record_desc_t desc = {0};
    fds_find_token_t  tok  = {0};

    err_code = fds_record_find(CONFIG_FILE, CONFIG_REC_KEY, &desc, &tok);

    if (err_code == NRF_SUCCESS)
    {
        /* A config file is in flash. Let's update it. */
        fds_flash_record_t config = {0};

        /* Open the record and read its contents. */
        err_code = fds_record_open(&desc, &config);
        APP_ERROR_CHECK(err_code);

        /* Copy the saved parameters from flash into mSavedParameters. */
        memcpy(&mSavedParameters, config.p_data, sizeof(SavedParameters_t));

        NRF_LOG_INFO("Config file found");

        /* Close the record when done reading. */
        err_code = fds_record_close(&desc);
        APP_ERROR_CHECK(err_code);
    }
    else
    {
        /* Saved parameter not found; write a new one. */
        NRF_LOG_INFO("Writing config file...");

        err_code = fds_record_write(&desc, &m_dummy_record);
        if ((err_code != NRF_SUCCESS) && (err_code == FDS_ERR_NO_SPACE_IN_FLASH))
        {
            err_code = fds_gc();
            switch (err_code)
            {
                case NRF_SUCCESS:
                    break;

                default:
                    NRF_LOG_INFO(
                                    "error: garbage collection returned %s\n", fds_err_str(err_code));
                    break;
            }
            NRF_LOG_INFO("No space in flash, delete some records to update the config file.");
        }
        else
        {
            APP_ERROR_CHECK(err_code);
        }
    }

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

    //err_code = nrf_fstorage_erase(&fstorage, START_ADDRESS, 1, NULL);
    //APP_ERROR_CHECK(err_code);
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

float saved_parameters_getSavedVehicleLength()
{
    return mSavedParameters.vehicleLength;
}

float saved_parameters_getSavedVehicleWidth()
{
    return mSavedParameters.vehicleWidth;
}

void saved_parameters_SaveAngleOffsets(float * angles)
{
    mSavedParameters.anglesCalibrationOffsets[0] = angles[0];
    mSavedParameters.anglesCalibrationOffsets[1] = angles[1];
    mSavedParameters.anglesCalibrationOffsets[2] = angles[2];
    record_update();
}

void saved_parameters_SaveOrientation(uint32_t orientation)
{
    mSavedParameters.orientation = orientation;
    record_update();
}

void saved_parameters_SaveHitchAngle(float angle)
{
    mSavedParameters.savedHitchHeight = angle;
    record_update();
}

void saved_parameters_SaveVehicleLength(float length)
{
    mSavedParameters.vehicleLength = length;
    record_update();
}

void saved_parameters_SaveVehicleWidth(float width)
{
    mSavedParameters.vehicleWidth = width;
    record_update();
}

static void record_update()
{
    fds_record_desc_t desc = {0};
    fds_find_token_t  ftok = {0};

    if (fds_record_find(CONFIG_FILE, CONFIG_REC_KEY, &desc, &ftok) == NRF_SUCCESS)
    {
        fds_record_t const rec =
        {
            .file_id           = CONFIG_FILE,
            .key               = CONFIG_REC_KEY,
            .data.p_data       = &mSavedParameters,
            .data.length_words = (sizeof(SavedParameters_t) + 3) / sizeof(uint32_t)
        };

        ret_code_t rc = fds_record_update(&desc, &rec);
        if (rc != NRF_SUCCESS)
        {
            NRF_LOG_INFO(
                            "error: fds_record_update() returned %s.\n",
                            fds_err_str(rc));
        }
    }
    else
    {
        NRF_LOG_INFO("error: could not find config file.\n");
    }
}
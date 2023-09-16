#ifndef SAVED_PARAMETERS_H
#define SAVED_PARAMETERS_H

#include "nrf_fstorage.h"
#include "nrf_fstorage_sd.h"
#include "fds.h"

/* File ID and Key used for the configuration record. */
#define CONFIG_FILE     (0x8010)
#define CONFIG_REC_KEY  (0x7010)


void saved_parameters_init();

void saved_parameters_getSavedCalibrationAngles(float * angles);
uint32_t saved_parameters_getSavedOrientation();
float saved_parameters_getSavedHitchHeightAngle();

void saved_parameters_SaveAngleOffsets(float * angles);
void saved_parameters_SaveOrientation(uint32_t orientation);
void saved_parameters_SaveHitchAngle(float angle);

#endif
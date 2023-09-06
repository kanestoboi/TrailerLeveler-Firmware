#ifndef SAVED_PARAMETERS_H
#define SAVED_PARAMETERS_H

#include "nrf_fstorage.h"
#include "nrf_fstorage_sd.h"

void saved_parameters_init();

void saved_parameters_getSavedCalibrationAngles(float * angles);
uint32_t saved_parameters_getSavedOrientation();
float saved_parameters_getSavedHitchHeightAngle();

void saved_parameters_SaveAngleOffsets(float * angles);
void saved_parameters_SaveOrientation(uint32_t orientation);
void saved_parameters_SaveHitchAngle(float angle);

void saved_parameters_copy_saved_parameters_from_flash();

#endif
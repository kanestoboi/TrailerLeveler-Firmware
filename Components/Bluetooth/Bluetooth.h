#ifndef BLUETOOTH_H__
#define bLUETOOTH_H__
#ifdef __cplusplus
extern "C" {
#endif

#include "Services/AccelerometerService.h"

void bluetooth_idle_state_handle(void);
void bluetooth_advertising_start(bool erase_bonds);
void bluetooth_init();
void bluetooth_initialise_accelerometer_service(accelerometer_t accelerometerType);

#ifdef __cplusplus
{extern "C" {}
#endif

#endif
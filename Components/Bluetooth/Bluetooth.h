#ifndef BLUETOOTH_H__
#define bLUETOOTH_H__
#ifdef __cplusplus
extern "C" {
#endif

#include "Services/AccelerometerService.h"

// Define a function pointer type for the functions you want to register.
typedef void (*ConnectedCallbackFunctionPointer)();
typedef void (*DisconnectedCallbackFunctionPointer)();

void bluetooth_idle_state_handle(void);
void bluetooth_advertising_start(bool erase_bonds);
void bluetooth_init();
void bluetooth_initialise_accelerometer_service(accelerometer_t accelerometerType);
void bluetooth_initialise_ess_service();
void bluetooth_update_battery_level(uint8_t batteryLevel);
void bluetooth_register_connected_callback(ConnectedCallbackFunctionPointer func);
void bluetooth_register_disconnected_callback(ConnectedCallbackFunctionPointer func);

extern void bluetooth_advertising_timeout_callback();

#ifdef __cplusplus
{extern "C" {}
#endif

#endif
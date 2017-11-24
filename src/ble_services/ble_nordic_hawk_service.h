/*
 * ble_nordic_hawk_service.h
 *
 *  Created on: 19 nov. 2017
 *      Author: diego.aguirre
 */

#ifndef _BLE_NORDIC_HAWK_SERVICE_H_
#define _BLE_NORDIC_HAWK_SERVICE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"

/**@brief   Macro for defining a ble_nordic_hawk_service(ble_nhs) instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_NHS_DEF(_name)                                               \
static ble_nhs_t _name;                                                  \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                	\
					 BLE_NHS_BLE_OBSERVER_PRIO,                          			\
                     ble_nhs_on_ble_evt, &_name)

#define NORDIC_HAWK_SERVICE_UUID_BASE        {0xC2, 0x42, 0x57, 0x27, 0x68, 0xDC, 0x48, 0x89,            \
											0xAA, 0xF0, 0x17, 0x0C, 0x00, 0x00, 0x96, 0xEB}
#define NORDIC_HAWK_SERVICE_UUID_SERVICE     					0x0A46
#define NORDIC_HAWK_SERVICE_UUID_ALL_MOTORS_CHAR 				0x0A47

// Length of Characteristic in bytes
#define ALL_MOTORS_CHAR_LEN         		2

// Forward declaration of the ble_hrs_t type.
typedef struct ble_nhs_s ble_nhs_t;

/**@brief Nordic Hawk Service event handler type. */
typedef void (*ble_nhs_all_motors_write_handler_t) (uint16_t conn_handle, ble_nhs_t * p_nhs, uint8_t const *new_state);

/**@brief Nordic Hawk Service init structure. This contains all options and data needed for
 *        initialization of the service. */
typedef struct
{
	ble_nhs_all_motors_write_handler_t      all_motors_write_handler;/**< Write handler to be called for handling write events to all motors char */                                    /**< Initial security level for all motors attribute */
} ble_nhs_init_t;


/**@brief Nordic Hawk Service structure. This contains various status information for the service. */
struct ble_nhs_s
{
    uint16_t                     			service_handle;      /**< Handle of Nordic Hawk Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t     			all_motors_handles;  /**< Handles related to the All Motors characteristic. */
    uint16_t                     			conn_handle;         /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint8_t                      			all_motors_char_len; /**< Current all motors char length */
    uint8_t                      			uuid_type;           /**< UUID type for the Nordic Hawk Service. */
    ble_nhs_all_motors_write_handler_t      all_motors_write_handler;
};

/**@brief Function for initializing the Nordic Hawk Service.
 *
 * @param[out] p_ss      Nordic Hawk Service structure. This structure must be supplied by
 *                        the application. It is initialized by this function and will later
 *                        be used to identify this particular service instance.
 * @param[in] p_ss_init  Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was initialized successfully. Otherwise, an error code is returned.
 */
uint32_t ble_nordic_hawk_service_init(ble_nhs_t * p_nhs, const ble_nhs_init_t *p_nhs_init);


/**@brief Function for handling the application's BLE stack events.
 *
 * @details This function handles all events from the BLE stack that are of interest to the Sensor Service.
 *
 * @param[in] p_ble_evt  Event received from the BLE stack.
 * @param[in] p_context  Sensor Service structure.
 */
void ble_nhs_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


#ifdef __cplusplus
}
#endif

#endif /* BLE_SERVICES_BLE_NORDIC_HAWK_SERVICE_H_ */

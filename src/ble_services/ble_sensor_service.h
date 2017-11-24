/*
 * ble_sensor_service.h
 *
 *  Created on: 17 sept. 2017
 *      Author: Diego Aguirre
 */

#ifndef _BLE_SENSOR_SERVICE_H_
#define _BLE_SENSOR_SERVICE_H_

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"

#ifdef __cplusplus
extern "C" {
#endif

/**@brief   Macro for defining a ble_sensor_service(ble_ss) instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_SS_DEF(_name)                                               \
static ble_ss_t _name;                                                  \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                	\
					 BLE_SS_BLE_OBSERVER_PRIO,                          			\
                     ble_ss_on_ble_evt, &_name)

#define SENSOR_SERVICE_UUID_BASE        {0xD6, 0xA7, 0xFF, 0xC9, 0x7F, 0xFD, 0x7D, 0x99,            \
											0x53, 0x4A, 0x21, 0xCB, 0x00, 0x00, 0xF0, 0x16}
#define SENSOR_SERVICE_UUID_SERVICE     		0xDA45
#define SENSOR_SERVICE_UUID_ACC_CHAR 			0xB700
#define SENSOR_SERVICE_UUID_GYRO_CHAR    		0xB701
#define SENSOR_SERVICE_UUID_MAG_CHAR            0xB702
#define SENSOR_SERVICE_UUID_QUATERNION_CHAR          0xB703

// Length of Characteristic in bytes
//#define ACC_LEN           6
//#define GYRO_LEN          6
//#define MAG_LEN        	  6
#define AXES3_CHAR_LEN         		6
#define QUATERNION_CHAR_LEN         16

/**@brief Heart Rate Service event type. */
typedef enum
{
    BLE_SS_EVT_NOTIFICATION_ENABLED,   /**< Sensor service char value notification enabled event. */
    BLE_SS_EVT_NOTIFICATION_DISABLED   /**< Sensor service char value notification disabled event. */
} ble_ss_evt_type_t;

/**@brief Heart Rate Service event. */
typedef struct
{
    ble_ss_evt_type_t evt_type;    /**< Type of event. */
} ble_ss_evt_t;

// Forward declaration of the ble_hrs_t type.
typedef struct ble_ss_s ble_ss_t;

/**@brief Heart Rate Service event handler type. */
typedef void (*ble_ss_evt_handler_t) (ble_ss_t * p_hrs, ble_ss_evt_t * p_evt);

/**@brief Heart Rate Service init structure. This contains all options and data needed for
 *        initialization of the service. */
typedef struct
{
    ble_ss_evt_handler_t        	evt_handler;                                          /**< Event handler to be called for handling events in the Heart Rate Service. */
    ble_srv_cccd_security_mode_t 	ss_acc_attr_md;                                      /**< Initial security level for acc measurement attribute */
} ble_ss_init_t;


/**@brief Heart Rate Service structure. This contains various status information for the service. */
struct ble_ss_s
{
    ble_ss_evt_handler_t         evt_handler;                                          /**< Event handler to be called for handling events in the Heart Rate Service. */
    uint16_t                     service_handle;                                       /**< Handle of Heart Rate Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t     acc_handles;                                          /**< Handles related to the ACC characteristic. */
    ble_gatts_char_handles_t     gyro_handles;                                         /**< Handles related to the GYRO characteristic. */
    ble_gatts_char_handles_t     mag_handles;                                          /**< Handles related to the MAG characteristic. */
    ble_gatts_char_handles_t     quaternion_meas_handles;                                        /**< Handles related to the AXIS3 characteristic. */
    uint16_t                     conn_handle;                                          /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint8_t                      axes3_char_len;                                          /**< Current maximum HR measurement length, adjusted according to the current ATT MTU. */
    uint8_t                      uuid_type;           /**< UUID type for the Sensor Service. */
};

/**@brief Function for initializing the LED Button Service.
 *
 * @param[out] p_ss      Sensor Service structure. This structure must be supplied by
 *                        the application. It is initialized by this function and will later
 *                        be used to identify this particular service instance.
 * @param[in] p_ss_init  Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was initialized successfully. Otherwise, an error code is returned.
 */
uint32_t ble_sensor_service_init(ble_ss_t * p_ss);


/**@brief Function for handling the application's BLE stack events.
 *
 * @details This function handles all events from the BLE stack that are of interest to the Sensor Service.
 *
 * @param[in] p_ble_evt  Event received from the BLE stack.
 * @param[in] p_context  Sensor Service structure.
 */
void ble_ss_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


/**@brief Function for sending heart rate measurement if notification has been enabled.
 *
 * @details The application calls this function after having performed a heart rate measurement.
 *          If notification has been enabled, the heart rate measurement data is encoded and sent to
 *          the client.
 *
 * @param[in]   p_ss                    Sensor Service structure.
 * @param[in]   acc_meas                New Acc measurement.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_ss_axes3_measurement_send(uint16_t conn_handle, uint16_t char_handle, int16_t *axes3_meas);
uint32_t ble_ss_quaternion_send(uint16_t conn_handle, uint16_t char_handle, float *quaternion);

#ifdef __cplusplus
}
#endif


#endif /* _BLE_SENSOR_SERVICE_H_ */

/*
 * ble_sensor_service.c
 *
 *  Created on: 17 sept. 2017
 *      Author: Diego Aguirre
 */

#include "sdk_common.h"
#if NRF_MODULE_ENABLED(BLE_SENSOR_SERVICE)
#include "ble_sensor_service.h"
#include "ble_srv_common.h"
#include "real_time_debugger.h"

//#define OPCODE_LENGTH 1                                                              /**< Length of opcode inside ACC Measurement packet. */
//#define HANDLE_LENGTH 2                                                              /**< Length of handle inside ACC Measurement packet. */
//#define MAX_HRM_LEN      (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH) /**< Maximum size of a transmitted ACC Measurement. */

//#define INITIAL_VALUE_HRM                       0                                    /**< Initial ACC Measurement value. */

static void sensor_service_encoder(int16_t *data_meas, uint8_t * p_encoded_buffer);
static void float_encoder(float value, uint8_t *p_encoded_value);

/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_ss        Sensor Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_ss_t * p_ss, ble_evt_t const * p_ble_evt)
{
    p_ss->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_ss        Sensor Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_ss_t * p_ss, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_ss->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling write events to the ACC Measurement characteristic.
 *
 * @param[in]   p_ss         Sensor Service structure.
 * @param[in]   p_evt_write   Write event received from the BLE stack.
 */
static void on_acc_cccd_write(ble_ss_t *p_ss, ble_gatts_evt_write_t const * p_evt_write)
{
    if (p_evt_write->len == 2)
    {
        // CCCD written, update notification state
        if (p_ss->evt_handler != NULL)
        {
            ble_ss_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_SS_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_SS_EVT_NOTIFICATION_DISABLED;
            }

            p_ss->evt_handler(p_ss, &evt);
        }
    }
}

/**@brief Function for handling the Write event.
 *
 * @param[in]   p_ss        Sensor Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_ss_t * p_ss, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (p_evt_write->handle == p_ss->acc_handles.cccd_handle)
    {
        on_acc_cccd_write(p_ss, p_evt_write);
    }
}

void ble_ss_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_ss_t *p_ss = (ble_ss_t *) p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_ss, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_ss, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_ss, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


///**@brief Function for setting characteristic metadata with notification
// *        configuration.
// *
// * @param[out]  p_char_md   Pointer to char metadata to be set as notification
// *
// * @return     none.
// */
//static void set_char_metadata_as_notification(ble_gatts_char_md_t *p_char_md, ble_gatts_attr_md_t cccd_md){
//
//	memset(&cccd_md, 0, sizeof(cccd_md));
//
//    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
//    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
//    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;
//
//    memset(p_char_md, 0, sizeof(*p_char_md));
//
//    p_char_md->char_props.notify = 1;
//    p_char_md->p_char_user_desc  = NULL;
//    p_char_md->p_char_pf         = NULL;
//    p_char_md->p_user_desc_md    = NULL;
//    p_char_md->p_cccd_md         = &cccd_md;
//    p_char_md->p_sccd_md         = NULL;
//}
//
///**@brief Function for setting attribute characteristic with axes3 type configuration.
// *		  Axes3 type characteristic is a characteristic that contains inertial sensor data
// *		  in X, y and Z axis format. Data len is 6 bytes (2 bytes for each axis)
// * @param[out]  p_attr_char_value   Pointer to attribute char to be set as Axes3 type
// * @param[in]   ble_uuid			Ble uuid data
// * @return     none.
// */
//static void set_attr_char_as_axes3_type(ble_gatts_attr_t *p_attr_char_value, ble_uuid_t ble_uuid)
//{
//	ble_gatts_attr_md_t attr_md;
//
//	memset(&attr_md, 0, sizeof(attr_md));
//
//	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
//	BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
//	attr_md.vloc       = BLE_GATTS_VLOC_STACK;
//	attr_md.rd_auth    = 0;
//	attr_md.wr_auth    = 0;
//	attr_md.vlen       = 1;
//
//	memset(p_attr_char_value, 0, sizeof(*p_attr_char_value));
//
//	p_attr_char_value->p_uuid    = &ble_uuid;
//	p_attr_char_value->p_attr_md = &attr_md;
//	p_attr_char_value->init_len  = AXES3_CHAR_LEN;
//	p_attr_char_value->init_offs = 0;
//	p_attr_char_value->max_len   = AXES3_CHAR_LEN;
//	p_attr_char_value->p_value   = NULL;
//}

/**@brief Function for adding the ACC Measurement characteristic.
 *
 * @param[in]   p_ss         Sensor Service structure.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t acc_measurement_char_add(ble_ss_t * p_ss)
{
	ble_gatts_attr_md_t cccd_md;
    ble_gatts_char_md_t char_md;
	ble_gatts_attr_md_t attr_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;

	memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_ss->uuid_type;
    ble_uuid.uuid = SENSOR_SERVICE_UUID_ACC_CHAR;

	memset(&attr_md, 0, sizeof(attr_md));

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
	attr_md.vloc       = BLE_GATTS_VLOC_STACK;
	attr_md.rd_auth    = 0;
	attr_md.wr_auth    = 0;
	attr_md.vlen       = 1;

	memset(&attr_char_value, 0, sizeof(attr_char_value));

	attr_char_value.p_uuid    = &ble_uuid;
	attr_char_value.p_attr_md = &attr_md;
	attr_char_value.init_len  = AXES3_CHAR_LEN;
	attr_char_value.init_offs = 0;
	attr_char_value.max_len   = AXES3_CHAR_LEN;
	attr_char_value.p_value   = NULL;

    return sd_ble_gatts_characteristic_add(p_ss->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_ss->acc_handles);
}

/**@brief Function for adding the Gyro Measurement characteristic.
 *
 * @param[in]   p_ss  Sensor Service structure.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t gyro_measurement_char_add(ble_ss_t * p_ss)
{
	ble_gatts_attr_md_t cccd_md;
    ble_gatts_char_md_t char_md;
	ble_gatts_attr_md_t attr_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;

	memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_ss->uuid_type;
    ble_uuid.uuid = SENSOR_SERVICE_UUID_GYRO_CHAR;

	memset(&attr_md, 0, sizeof(attr_md));

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
	attr_md.vloc       = BLE_GATTS_VLOC_STACK;
	attr_md.rd_auth    = 0;
	attr_md.wr_auth    = 0;
	attr_md.vlen       = 1;

	memset(&attr_char_value, 0, sizeof(attr_char_value));

	attr_char_value.p_uuid    = &ble_uuid;
	attr_char_value.p_attr_md = &attr_md;
	attr_char_value.init_len  = AXES3_CHAR_LEN;
	attr_char_value.init_offs = 0;
	attr_char_value.max_len   = AXES3_CHAR_LEN;
	attr_char_value.p_value   = NULL;

    return sd_ble_gatts_characteristic_add(p_ss->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_ss->gyro_handles);
}

/**@brief Function for adding the Mag Measurement characteristic.
 *
 * @param[in]   p_ss  Sensor Service structure.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t mag_measurement_char_add(ble_ss_t * p_ss)
{
	ble_gatts_attr_md_t cccd_md;
    ble_gatts_char_md_t char_md;
	ble_gatts_attr_md_t attr_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;

	memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_ss->uuid_type;
    ble_uuid.uuid = SENSOR_SERVICE_UUID_MAG_CHAR;

	memset(&attr_md, 0, sizeof(attr_md));

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
	attr_md.vloc       = BLE_GATTS_VLOC_STACK;
	attr_md.rd_auth    = 0;
	attr_md.wr_auth    = 0;
	attr_md.vlen       = 1;

	memset(&attr_char_value, 0, sizeof(attr_char_value));

	attr_char_value.p_uuid    = &ble_uuid;
	attr_char_value.p_attr_md = &attr_md;
	attr_char_value.init_len  = AXES3_CHAR_LEN;
	attr_char_value.init_offs = 0;
	attr_char_value.max_len   = AXES3_CHAR_LEN;
	attr_char_value.p_value   = NULL;

    return sd_ble_gatts_characteristic_add(p_ss->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_ss->mag_handles);
}

/**@brief Function for adding the Axes3 Measurement characteristic.
 *
 * @param[in]   p_ss  Sensor Service structure.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t quaternion_char_add(ble_ss_t * p_ss)
{
	ble_gatts_attr_md_t cccd_md;
    ble_gatts_char_md_t char_md;
	ble_gatts_attr_md_t attr_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;

	memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = (uint8_t *)"Quaternion data(float)";
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_ss->uuid_type;
    ble_uuid.uuid = SENSOR_SERVICE_UUID_QUATERNION_CHAR;

	memset(&attr_md, 0, sizeof(attr_md));

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
	attr_md.vloc       = BLE_GATTS_VLOC_STACK;
	attr_md.rd_auth    = 0;
	attr_md.wr_auth    = 0;
	attr_md.vlen       = 1;

	memset(&attr_char_value, 0, sizeof(attr_char_value));

	attr_char_value.p_uuid    = &ble_uuid;
	attr_char_value.p_attr_md = &attr_md;
	attr_char_value.init_len  = QUATERNION_CHAR_LEN;
	attr_char_value.init_offs = 0;
	attr_char_value.max_len   = QUATERNION_CHAR_LEN;
	attr_char_value.p_value   = NULL;

    return sd_ble_gatts_characteristic_add(p_ss->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_ss->quaternion_meas_handles);
}

uint32_t ble_sensor_service_init(ble_ss_t * p_ss)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    //p_ss->evt_handler                 = p_ss_init->evt_handler;
    p_ss->conn_handle                 = BLE_CONN_HANDLE_INVALID;
    p_ss->axes3_char_len              = AXES3_CHAR_LEN;

    // Add service.
    ble_uuid128_t base_uuid = {SENSOR_SERVICE_UUID_BASE};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_ss->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_ss->uuid_type;
    ble_uuid.uuid = SENSOR_SERVICE_UUID_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_ss->service_handle);

    VERIFY_SUCCESS(err_code);

    // Add acc measurement characteristic
    err_code = acc_measurement_char_add(p_ss);
    VERIFY_SUCCESS(err_code);

    err_code = gyro_measurement_char_add(p_ss);
    VERIFY_SUCCESS(err_code);

    err_code = mag_measurement_char_add(p_ss);
    VERIFY_SUCCESS(err_code);

    err_code = quaternion_char_add(p_ss);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

uint32_t ble_ss_axes3_measurement_send(uint16_t conn_handle, uint16_t char_handle, int16_t *axes3_meas)
{
    uint32_t err_code;

    // Send value if connected and notifying
    if (conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint8_t                axes3_meas_encoded[AXES3_CHAR_LEN];
        uint16_t               hvx_len;
        ble_gatts_hvx_params_t hvx_params;

        hvx_len = AXES3_CHAR_LEN;

        sensor_service_encoder(axes3_meas, axes3_meas_encoded);
        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = char_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = axes3_meas_encoded;

        err_code = sd_ble_gatts_hvx(conn_handle, &hvx_params);
        if ((err_code == NRF_SUCCESS) && (hvx_len != AXES3_CHAR_LEN))
        {
            err_code = NRF_ERROR_DATA_SIZE;
        }

//        char data[30];
//        sprintf(data, "0: %d", axes3_meas[0]);
//        rtt_println(data);
//        sprintf(data, "1: %d", axes3_meas[1]);
//        rtt_println(data);
//        sprintf(data, "2: %d", axes3_meas[2]);
//        rtt_println(data);
//        sprintf(data, "3: %d", axes3_meas[3]);
//        rtt_println(data);
//        sprintf(data, "4: %d", axes3_meas[4]);
//        rtt_println(data);
//        sprintf(data, "5: %d", axes3_meas[5]);
//        rtt_println(data);

    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}

uint32_t ble_ss_quaternion_send(uint16_t conn_handle, uint16_t char_handle, float *quaternion)
{
    uint32_t err_code;

    // Send value if connected and notifying
    if (conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint8_t                quaternion_encoded[QUATERNION_CHAR_LEN];
        uint16_t               hvx_len;
        ble_gatts_hvx_params_t hvx_params;

        hvx_len = QUATERNION_CHAR_LEN;

        int k = 0;
        for(int i=0; i < 4; i++){
        	uint8_t encoded_value[4];
        	float_encoder(quaternion[i], encoded_value);
        	for(int j = 0; j < 4; j++){
        		quaternion_encoded[k] = encoded_value[j];
        		k++;
        	}
        }

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = char_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = quaternion_encoded;

        err_code = sd_ble_gatts_hvx(conn_handle, &hvx_params);
        if ((err_code == NRF_SUCCESS) && (hvx_len != AXES3_CHAR_LEN))
        {
            err_code = NRF_ERROR_DATA_SIZE;
        }

    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}

static void sensor_service_encoder(int16_t *data_meas, uint8_t * p_encoded_buffer)
{
	int j=0;
	for(int i = 0; i < 3; i++){
		p_encoded_buffer[j++] = (uint8_t)(data_meas[i] >> 8);
		p_encoded_buffer[j++] = (uint8_t)data_meas[i];
	}
}

static void float_encoder(float value, uint8_t *p_encoded_value)
{
	union{
		float value;
		uint8_t encoded_value[4];
	}encoder;

	encoder.value = value;

	for(int i=0; i < 4; i++){
		p_encoded_value[i] = encoder.encoded_value[i];
	}
}

//void ble_ss_on_gatt_evt(ble_ss_t * p_ss, nrf_ble_gatt_evt_t const * p_gatt_evt)
//{
//    if (    (p_ss->conn_handle == p_gatt_evt->conn_handle)
//        &&  (p_gatt_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
//    {
//        p_ss->acc_len = ACC_LEN;
//    }
//}

#endif //NRF_MODULE_ENABLED(BLE_SENSOR_SERVICE)

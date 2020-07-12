#include "sdk_common.h"
#include "ble_doseIO_s.h"
#include "ble_srv_common.h"



/**@brief Function for handling the Write event.
 *
 * @param[in] p_doseIO      doseIO Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_write(ble_doseIO_t * p_doseIO, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (p_evt_write->handle == p_doseIO->synch_time_char_handles.value_handle)
    {
        p_doseIO->synch_time_handler(p_ble_evt->evt.gap_evt.conn_handle, p_doseIO, p_evt_write->data[0]);
    }
		else if (p_evt_write->handle == p_doseIO->set_notif_char_handles.value_handle)
    {
        p_doseIO->set_notif_handler(p_ble_evt->evt.gap_evt.conn_handle, p_doseIO, p_evt_write->data[0]);
    }
}


void ble_doseIO_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_doseIO_t * p_doseIO = (ble_doseIO_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTS_EVT_WRITE:
            on_write(p_doseIO, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


uint32_t ble_doseIO_init(ble_doseIO_t * p_doseIO, const ble_doseIO_init_t * p_doseIO_init)
{
    volatile uint32_t              err_code;
    ble_uuid_t            ble_uuid;
    ble_add_char_params_t add_char_params;

    // Initialize service structure.
    p_doseIO->synch_time_handler = p_doseIO_init->synch_time_handler;

    // Add service.
    ble_uuid128_t base_uuid = {doseIO_UUID_BASE};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_doseIO->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_doseIO->uuid_type;
    ble_uuid.uuid = doseIO_UUID_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_doseIO->service_handle);
    VERIFY_SUCCESS(err_code);

    // Add Button characteristic.
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid              = doseIO_UUID_BUTTON_CHAR;
    add_char_params.uuid_type         = p_doseIO->uuid_type;
    add_char_params.init_len          = sizeof(uint16_t);
    add_char_params.max_len           = sizeof(uint16_t);
    add_char_params.char_props.read   = 1;
    add_char_params.char_props.notify = 1;

    add_char_params.read_access       = SEC_OPEN;
    add_char_params.cccd_write_access = SEC_OPEN;

    err_code = characteristic_add(p_doseIO->service_handle,
                                  &add_char_params,
                                  &p_doseIO->button_char_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
		
		memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid              = doseIO_UUID_BUTTON_CHAR2;
    add_char_params.uuid_type         = p_doseIO->uuid_type;
    add_char_params.init_len          = sizeof(uint16_t);
    add_char_params.max_len           = sizeof(uint16_t);
    add_char_params.char_props.read   = 1;
    add_char_params.char_props.notify = 1;

    add_char_params.read_access       = SEC_OPEN;
    add_char_params.cccd_write_access = SEC_OPEN;

    err_code = characteristic_add(p_doseIO->service_handle,
                                  &add_char_params,
                                  &p_doseIO->button_char_handles2);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
		
		memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid              = doseIO_UUID_BUTTON_CHAR2+5;
    add_char_params.uuid_type         = p_doseIO->uuid_type;
    add_char_params.init_len          = sizeof(uint16_t);
    add_char_params.max_len           = sizeof(uint16_t);
    add_char_params.char_props.read   = 1;
    add_char_params.char_props.notify = 1;

    add_char_params.read_access       = SEC_OPEN;
    add_char_params.cccd_write_access = SEC_OPEN;

    err_code = characteristic_add(p_doseIO->service_handle,
                                  &add_char_params,
                                  &p_doseIO->button_char_handles2);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add synch_time characteristic.
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid             = doseIO_UUID_SYNCH_TIME_CHAR;
    add_char_params.uuid_type        = p_doseIO->uuid_type;
    add_char_params.init_len         = sizeof(uint32_t);
    add_char_params.max_len          = sizeof(uint32_t);
    add_char_params.char_props.read  = 1;
    add_char_params.char_props.write = 1;

    add_char_params.read_access  = SEC_OPEN;
    add_char_params.write_access = SEC_OPEN;

    err_code = characteristic_add(p_doseIO->service_handle, &add_char_params, &p_doseIO->synch_time_char_handles);
		
		if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
//		
//		// Add set_notif characteristic.
//    memset(&add_char_params, 0, sizeof(add_char_params));
//    add_char_params.uuid             = doseIO_UUID_SET_NOTIF_CHAR;
//    add_char_params.uuid_type        = p_doseIO->uuid_type;
//    add_char_params.init_len         = sizeof(uint32_t);
//    add_char_params.max_len          = sizeof(uint32_t);
//    add_char_params.char_props.read  = 1;
//    add_char_params.char_props.write = 1;

//    add_char_params.read_access  = SEC_OPEN;
//    add_char_params.write_access = SEC_OPEN;

//    err_code = characteristic_add(p_doseIO->service_handle, &add_char_params, &p_doseIO->set_notif_char_handles);
		
		return err_code;
}


uint32_t ble_doseIO_on_button_change(uint16_t conn_handle, ble_doseIO_t * p_doseIO, uint8_t button_state)
{
    ble_gatts_hvx_params_t params;
    uint16_t len = sizeof(button_state);

    memset(&params, 0, sizeof(params));
    params.type   = BLE_GATT_HVX_NOTIFICATION;
    params.handle = p_doseIO->button_char_handles.value_handle;
    params.p_data = &button_state;
    params.p_len  = &len;

    return sd_ble_gatts_hvx(conn_handle, &params);
}
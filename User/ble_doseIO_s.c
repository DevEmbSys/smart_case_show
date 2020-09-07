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

static void on_write_Journal(ble_doseIO_Journal_t * p_doseIO_Journal, ble_evt_t const * p_ble_evt)
{
		static ble_gatts_evt_write_t p_evt_write;
		memcpy((uint8_t*)&p_evt_write,&p_ble_evt->evt.gatts_evt.params.write,sizeof(ble_gatts_evt_write_t));
	
		if (p_evt_write.handle == p_doseIO_Journal->read_handles.value_handle)
    {
				static uint32_t dataTimeSynch;
			
				dataTimeSynch = 0;
			
				for(uint8_t i = 0; i < p_evt_write.len; i++)
				{
					((uint8_t*)&dataTimeSynch)[p_evt_write.len - 1 - i] = p_evt_write.data[i];
				}
				
        p_doseIO_Journal->read_handler(p_ble_evt->evt.gap_evt.conn_handle, p_doseIO_Journal, dataTimeSynch);
    }
		else if (p_evt_write.handle == p_doseIO_Journal->clear_handles.value_handle)
    {
				static uint32_t dataTimeSynch;
			
				dataTimeSynch = 0;
			
				for(uint8_t i = 0; i < p_evt_write.len; i++)
				{
					((uint8_t*)&dataTimeSynch)[p_evt_write.len - 1 - i] = p_evt_write.data[i];
				}
				
        p_doseIO_Journal->clear_handler(p_ble_evt->evt.gap_evt.conn_handle, p_doseIO_Journal, dataTimeSynch);
    }
}

static void on_write_Calendare(ble_doseIO_Calendare_t * p_doseIO_Calendare, ble_evt_t const * p_ble_evt)
{
    static uint8_t p_evt_write_data[sizeof(ble_gatts_evt_write_t)+100];
		volatile ble_gatts_evt_write_t* p_evt_write = (ble_gatts_evt_write_t*)p_evt_write_data;
		memcpy((uint8_t*)p_evt_write,&p_ble_evt->evt.gatts_evt.params.write,sizeof(ble_gatts_evt_write_t)+p_ble_evt->evt.gatts_evt.params.write.len);
	
		if (p_evt_write->handle == p_doseIO_Calendare->time_synch_handles.value_handle)
    {
				static uint32_t dataTimeSynch;
			
				dataTimeSynch = 0;
			
				for(uint8_t i = 0; i < p_evt_write->len; i++)
				{
					((uint8_t*)&dataTimeSynch)[p_evt_write->len - 1 - i] = p_evt_write->data[i];
				}
				
        p_doseIO_Calendare->time_synch_handler(p_ble_evt->evt.gap_evt.conn_handle, p_doseIO_Calendare, dataTimeSynch);
    }
		else if (p_evt_write->handle == p_doseIO_Calendare->write_notif_handles.value_handle)
    {
				static uint32_t dataWriteNotif;
			
				dataWriteNotif = 0;
			
				for(uint8_t i = 0; i < p_evt_write->len; i++)
				{
					((uint8_t*)&dataWriteNotif)[p_evt_write->len - 1 - i] = p_evt_write->data[i];
				}
				
        p_doseIO_Calendare->write_notif_handler(p_ble_evt->evt.gap_evt.conn_handle, p_doseIO_Calendare, dataWriteNotif);
    }
		else if (p_evt_write->handle == p_doseIO_Calendare->clear_notif_handles.value_handle)
    {
				static uint32_t dataClearNotif;
			
				dataClearNotif = 0;
			
				for(uint8_t i = 0; i < p_evt_write->len; i++)
				{
					((uint8_t*)&dataClearNotif)[p_evt_write->len - 1 - i] = p_evt_write->data[i];
				}
				
        p_doseIO_Calendare->clear_notif_handler(p_ble_evt->evt.gap_evt.conn_handle, p_doseIO_Calendare, dataClearNotif);
    }
		else if (p_evt_write->handle == p_doseIO_Calendare->list_notif_handles.value_handle)
    {
				static uint32_t dataListNotif;
			
				dataListNotif = 0;
			
				for(uint8_t i = 0; i < p_evt_write->len; i++)
				{
					((uint8_t*)&dataListNotif)[p_evt_write->len - 1 - i] = p_evt_write->data[i];
				}
				
        p_doseIO_Calendare->list_notif_handler(p_ble_evt->evt.gap_evt.conn_handle, p_doseIO_Calendare, dataListNotif);
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

/**@brief Function for handling the @ref BLE_GATTS_EVT_HVN_TX_COMPLETE event from the SoftDevice.
 *
 * @param[in] p_nus     Nordic UART Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_hvx_tx_complete(ble_doseIO_Calendare_t * doseIO_Calendare, ble_evt_t const * p_ble_evt)
{
    volatile static ble_gatts_evt_write_t p_evt_write;
		memcpy((uint8_t*)&p_evt_write,&p_ble_evt->evt.gatts_evt.params.write,sizeof(ble_gatts_evt_write_t));

    if (p_evt_write.handle == doseIO_Calendare->list_notif_handles.value_handle)
    {
        doseIO_Calendare->list_notif_handler(p_ble_evt->evt.gap_evt.conn_handle, doseIO_Calendare, p_evt_write.data[0]);
    }
}

void ble_doseIO_Journal_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_doseIO_Journal_t * p_doseIO_Journal = (ble_doseIO_Journal_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
				case BLE_GAP_EVT_CONNECTED:
            //on_connect(p_nus, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write_Journal(p_doseIO_Journal, p_ble_evt);
            break;

//        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
//            on_hvx_tx_complete(p_nus, p_ble_evt);
//            break;

        default:
            // No implementation needed.
            break;
    }
}

void ble_doseIO_Calendare_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_doseIO_Calendare_t * p_doseIO_Calendare = (ble_doseIO_Calendare_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTS_EVT_WRITE:
            on_write_Calendare(p_doseIO_Calendare, p_ble_evt);
            break;
							
				case BLE_GATTS_EVT_HVN_TX_COMPLETE:
            on_hvx_tx_complete(p_doseIO_Calendare, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


uint32_t ble_doseIO_init_s_settings(ble_doseIO_t * p_doseIO, const ble_doseIO_init_t * p_doseIO_init)
{
    volatile uint32_t              err_code;
    ble_uuid_t            ble_uuid;
    ble_add_char_params_t add_char_params;

    // Initialize service structure.
    p_doseIO->synch_time_handler = p_doseIO_init->synch_time_handler;

    // Add service.
    ble_uuid128_t base_uuid = {doseIO_UUID_SETTINGS};
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

    // Add set_notif characteristic.
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid             = doseIO_UUID_SET_NOTIF_CHAR;
    add_char_params.uuid_type        = p_doseIO->uuid_type;
    add_char_params.init_len         = sizeof(uint32_t);
    add_char_params.max_len          = sizeof(uint32_t);
    add_char_params.char_props.read  = 1;
    add_char_params.char_props.write = 1;

    add_char_params.read_access  = SEC_OPEN;
    add_char_params.write_access = SEC_OPEN;

    err_code = characteristic_add(p_doseIO->service_handle, &add_char_params, &p_doseIO->set_notif_char_handles);
		
		return err_code;
}

//jurnal service init
uint32_t ble_doseIO_init_s_journal(ble_doseIO_Journal_t * p_doseIO_Journal, const ble_doseIO_init_t * p_doseIO_init)
{
    volatile uint32_t              err_code;
    ble_uuid_t            ble_uuid;
    ble_add_char_params_t add_char_params;

    // Initialize service structure.
    p_doseIO_Journal->clear_handler = p_doseIO_init->Journal_clear_handler;
		p_doseIO_Journal->read_handler = p_doseIO_init->Journal_read_handler;

    // Add service.
    ble_uuid128_t base_uuid = {doseIO_UUID_JOURNAL};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_doseIO_Journal->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_doseIO_Journal->uuid_type;
    ble_uuid.uuid = doseIO_UUID_J_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_doseIO_Journal->service_handle);
    VERIFY_SUCCESS(err_code);

    // Add Button characteristic.
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid              = doseIO_UUID_J_READ;
    add_char_params.uuid_type         = p_doseIO_Journal->uuid_type;
    add_char_params.max_len           = BLE_NUS_MAX_DATA_LEN;
    add_char_params.init_len          = sizeof(uint8_t);
		add_char_params.char_props.read 	= 1;
		add_char_params.char_props.write 	= 1;
    add_char_params.char_props.notify = 1;

    add_char_params.read_access       = SEC_OPEN;
		add_char_params.write_access      = SEC_OPEN;
    add_char_params.cccd_write_access = SEC_OPEN;

    err_code = characteristic_add(p_doseIO_Journal->service_handle,
                                  &add_char_params,
                                  &p_doseIO_Journal->read_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
		
		// Add Button characteristic.
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid              = doseIO_UUID_J_CLEAR;
    add_char_params.uuid_type         = p_doseIO_Journal->uuid_type;
    add_char_params.max_len           = sizeof(uint32_t);
    add_char_params.init_len          = sizeof(uint32_t);
		add_char_params.char_props.write 	= 1;

    add_char_params.write_access      = SEC_OPEN;

    err_code = characteristic_add(p_doseIO_Journal->service_handle,
                                  &add_char_params,
                                  &p_doseIO_Journal->clear_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
}

//calendare service init
uint32_t ble_doseIO_init_s_calendare(ble_doseIO_Calendare_t * p_doseIO_Calendare, const ble_doseIO_init_t * p_doseIO_init)
{
	volatile uint32_t              err_code;
	ble_uuid_t            ble_uuid;
	ble_add_char_params_t add_char_params;

	// Initialize service structure.
	p_doseIO_Calendare->time_synch_handler = p_doseIO_init->Calendare_time_synch_handler;
	p_doseIO_Calendare->write_notif_handler = p_doseIO_init->Calendare_write_notif_handler;
	p_doseIO_Calendare->clear_notif_handler = p_doseIO_init->Calendare_clear_notif_handler;
	p_doseIO_Calendare->list_notif_handler = p_doseIO_init->Calendare_list_notif_handler;

	// Add service.
	ble_uuid128_t base_uuid = {doseIO_UUID_CALENDARE};
	err_code = sd_ble_uuid_vs_add(&base_uuid, &p_doseIO_Calendare->uuid_type);
	VERIFY_SUCCESS(err_code);

	ble_uuid.type = p_doseIO_Calendare->uuid_type;
	ble_uuid.uuid = doseIO_UUID_C_SERVICE;

	err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_doseIO_Calendare->service_handle);
	VERIFY_SUCCESS(err_code);

	// Add Button characteristic.
	memset(&add_char_params, 0, sizeof(add_char_params));
	add_char_params.uuid             = doseIO_UUID_C_SYNCHR;
	add_char_params.uuid_type        = p_doseIO_Calendare->uuid_type;
	add_char_params.init_len         = sizeof(uint32_t);
	add_char_params.max_len          = sizeof(uint32_t);
	add_char_params.char_props.read  = 1;
	add_char_params.char_props.write = 1;

	add_char_params.read_access  = SEC_OPEN;
	add_char_params.write_access = SEC_OPEN;

	err_code = characteristic_add(p_doseIO_Calendare->service_handle,
																&add_char_params,
																&p_doseIO_Calendare->time_synch_handles);
	if (err_code != NRF_SUCCESS)
	{
			return err_code;
	}
	
	// Add Button characteristic.
	memset(&add_char_params, 0, sizeof(add_char_params));
	add_char_params.uuid             = doseIO_UUID_C_WRITE_NOTIF;
	add_char_params.uuid_type        = p_doseIO_Calendare->uuid_type;
	add_char_params.init_len         = sizeof(uint32_t);
	add_char_params.max_len          = sizeof(uint32_t);
	add_char_params.char_props.write = 1;

	add_char_params.write_access = SEC_OPEN;

	err_code = characteristic_add(p_doseIO_Calendare->service_handle,
																&add_char_params,
																&p_doseIO_Calendare->write_notif_handles);
	
	if (err_code != NRF_SUCCESS)
	{
			return err_code;
	}
	
	// Add Button characteristic.
	memset(&add_char_params, 0, sizeof(add_char_params));
	add_char_params.uuid             = doseIO_UUID_C_CLEAR_NOTIF;
	add_char_params.uuid_type        = p_doseIO_Calendare->uuid_type;
	add_char_params.init_len         = sizeof(uint32_t);
	add_char_params.max_len          = sizeof(uint32_t);
	add_char_params.char_props.read  = 0;
	add_char_params.char_props.write = 1;

	add_char_params.read_access  = SEC_OPEN;
	add_char_params.write_access = SEC_OPEN;

	err_code = characteristic_add(p_doseIO_Calendare->service_handle,
																&add_char_params,
																&p_doseIO_Calendare->clear_notif_handles);
	
	if (err_code != NRF_SUCCESS)
	{
			return err_code;
	}
	
	// Add Button characteristic.
	memset(&add_char_params, 0, sizeof(add_char_params));
	add_char_params.uuid             = doseIO_UUID_C_LIST_NOTIF;
	add_char_params.uuid_type        = p_doseIO_Calendare->uuid_type;
	add_char_params.max_len           = BLE_NUS_MAX_DATA_LEN;
	add_char_params.init_len          = sizeof(uint8_t);
	add_char_params.is_var_len        = true;
	add_char_params.char_props.notify = 1;
	add_char_params.char_props.write  = 1;

	add_char_params.read_access  = SEC_OPEN;
	add_char_params.write_access = SEC_OPEN;
	add_char_params.cccd_write_access = SEC_OPEN;

	err_code = characteristic_add(p_doseIO_Calendare->service_handle,
																&add_char_params,
																&p_doseIO_Calendare->list_notif_handles);
	
	if (err_code != NRF_SUCCESS)
	{
			return err_code;
	}
	
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

uint32_t ble_list_notif_data_send(ble_doseIO_Calendare_t * p_doseIO_Calendare, uint8_t   * p_data, uint16_t NotifId, uint16_t NotifIdMax, uint16_t    conn_handle)
{
		volatile uint16_t dataLengthCopy = 4;
    volatile ret_code_t                 err_code;
		volatile uint32_t count = 0;
    ble_gatts_hvx_params_t     hvx_params;

    memset(&hvx_params, 0, sizeof(hvx_params));

		if(NotifId < NotifIdMax)
		{
			hvx_params.handle = p_doseIO_Calendare->list_notif_handles.value_handle;
			hvx_params.p_data = p_data+(NotifId*4);
			hvx_params.p_len  = (uint16_t*)&dataLengthCopy;
			hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
			
			err_code = sd_ble_gatts_hvx(conn_handle, &hvx_params);
			
			count++;
		}
		else if(NotifId >= NotifIdMax)
		{
			dataLengthCopy = NotifIdMax*4;
			for(int16_t i = dataLengthCopy, j = 0; i > 0 ; i -= BLE_NUS_MAX_DATA_LEN, j += BLE_NUS_MAX_DATA_LEN)
			{
				uint16_t p_length_current = BLE_NUS_MAX_DATA_LEN;
				hvx_params.handle = p_doseIO_Calendare->list_notif_handles.value_handle;
				hvx_params.p_data = p_data+j;
				if(i > BLE_NUS_MAX_DATA_LEN) p_length_current = BLE_NUS_MAX_DATA_LEN;
				else p_length_current = i;
				hvx_params.p_len  = (uint16_t*)&p_length_current;
				hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
				
				err_code = sd_ble_gatts_hvx(conn_handle, &hvx_params);
				if(err_code != NRF_SUCCESS)
				{
					volatile uint8_t a;
					a++;
					return err_code;
				}
				count++;
			}
		}
		
		return count;
}

uint32_t ble_J_read_data_send(ble_doseIO_Journal_t * p_doseIO_Journal, uint8_t* p_data, uint16_t EventId, uint16_t EventIdMax, uint16_t    conn_handle)
{
		ret_code_t                 err_code;
    ble_gatts_hvx_params_t     hvx_params;
		volatile uint32_t count = 0;
		volatile uint16_t lengthData = 3*4;

    memset(&hvx_params, 0, sizeof(hvx_params));

		if(EventId < EventIdMax)
		{
			hvx_params.handle = p_doseIO_Journal->read_handles.value_handle;
			hvx_params.p_data = p_data+(EventId*3*4);
			hvx_params.p_len  = (uint16_t*)&lengthData;
			hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
			
			err_code = sd_ble_gatts_hvx(conn_handle, &hvx_params);
			count++;
		}
		else if(EventId >= EventIdMax)
		{
			lengthData = EventIdMax*3*4;
			for(int16_t i = lengthData, j = 0; i > 0 ; i -= BLE_NUS_MAX_DATA_LEN, j += BLE_NUS_MAX_DATA_LEN)
			{
				uint16_t p_length_current = BLE_NUS_MAX_DATA_LEN;
				hvx_params.handle = p_doseIO_Journal->read_handles.value_handle;
				hvx_params.p_data = p_data+j;
				if(i > BLE_NUS_MAX_DATA_LEN) p_length_current = BLE_NUS_MAX_DATA_LEN;
				else p_length_current = i;
				hvx_params.p_len  = (uint16_t*)&p_length_current;
				hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
				
				err_code = sd_ble_gatts_hvx(conn_handle, &hvx_params);
				count++;
			}
		}
		
		return count;
}
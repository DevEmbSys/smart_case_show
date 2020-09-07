#ifndef BLE_DOSEIO_S_H__
#define BLE_DOSEIO_S_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"

#ifdef __cplusplus
extern "C" {
#endif
	
	/**@brief   Macro for defining a ble_doseIO instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_doseIO_DEF(_name)                                                                          \
static ble_doseIO_t _name;                                                                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 	 \
                     BLE_doseIO_BLE_OBSERVER_PRIO,                                                     \
                     ble_doseIO_on_ble_evt, &_name)
	
#define BLE_doseIO_Journal_DEF(_name)                                                                          \
static ble_doseIO_Journal_t _name;                                                                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 	 \
                     BLE_doseIO_BLE_OBSERVER_PRIO,                                                     \
                     ble_doseIO_Journal_on_ble_evt, &_name)

#define BLE_doseIO_Calendare_DEF(_name)                                                                          \
static ble_doseIO_Calendare_t _name;                                                                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 	 \
                     BLE_doseIO_BLE_OBSERVER_PRIO,                                                     \
                     ble_doseIO_Calendare_on_ble_evt, &_name)
										 
typedef struct{
	uint8_t Data;
}s_Journal_data;	

#define OPCODE_LENGTH        1
#define HANDLE_LENGTH        2

#define	JOURNAL_QUEQUE_MAX 20

#define BLE_NUS_MAX_DATA_LEN (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH)
	
/*
UUID services:
	1)Settings device
		1c410584-ec7c-11ea-adc1-0242ac120002

	2)Journal Event
		8179fa96-ec7c-11ea-adc1-0242ac120002

	3)Calendare notification
		93ec9724-ec7c-11ea-adc1-0242ac120002

*/

#define doseIO_UUID_SETTINGS			{0x02, 0x00, 0x12, 0xac, 0x42, 0x02, 0xc1, 0xad, \
																	 0xea, 0x11, 0x7c, 0xec, 0x84, 0x05, 0x41, 0x1c}

#define doseIO_UUID_JOURNAL				{0x02, 0x00, 0x12, 0xac, 0x42, 0x02, 0xc1, 0xad, \
																	 0xea, 0x11, 0x7c, 0xec, 0x96, 0xfa, 0x79, 0x81}

#define doseIO_UUID_J_SERVICE						0xa000
#define doseIO_UUID_J_READ     					0xa001
#define doseIO_UUID_J_CLEAR							0xa002

#define doseIO_UUID_CALENDARE			{0x02, 0x00, 0x12, 0xac, 0x42, 0x02, 0xc1, 0xad, \
																	 0xea, 0x11, 0x7c, 0xec, 0x24, 0x97, 0xec, 0x93}

#define doseIO_UUID_C_SERVICE						0xb000
#define doseIO_UUID_C_SYNCHR						0xb001
#define doseIO_UUID_C_WRITE_NOTIF				0xb002
#define doseIO_UUID_C_CLEAR_NOTIF				0xb003
#define doseIO_UUID_C_LIST_NOTIF				0xb004
																	 
//15f2e42e-c26d-11ea-b3de-0242ac130004
#define doseIO_UUID_BASE					{0x04, 0x00, 0x13, 0xac, 0x42, 0x02, 0xde, 0xb3, \
																	 0xea, 0x11, 0x6d, 0xc2, 0x2e, 0xe4, 0xf2, 0x15}
																	 
#define doseIO_UUID_SERVICE     				0xc000
#define doseIO_UUID_SYNCH_TIME_CHAR    	0xc001
#define doseIO_UUID_SET_NOTIF_CHAR    	0xc002
#define doseIO_UUID_BUTTON_CHAR 				0xc003
#define doseIO_UUID_BUTTON_CHAR2 				0xc004

// Forward declaration of the ble_doseIO_t type.
typedef struct ble_doseIO_s ble_doseIO_t;
typedef struct ble_doseIO_Journal_s ble_doseIO_Journal_t;
typedef struct ble_doseIO_Calendare_s ble_doseIO_Calendare_t;

typedef void (*ble_doseIO_synch_time_handler_t) (uint16_t conn_handle, ble_doseIO_t * p_doseIO, uint32_t data);
typedef void (*ble_doseIO_set_notif_handler_t) (uint16_t conn_handle, ble_doseIO_t * p_doseIO, uint32_t data);
																	 
typedef void (*ble_doseIO_J_read_handler_t) (uint16_t conn_handle, ble_doseIO_Journal_t * p_doseIO_Journal, uint32_t data);
typedef void (*ble_doseIO_J_clear_handler_t) (uint16_t conn_handle, ble_doseIO_Journal_t * p_doseIO_Journal, uint32_t data);
																	 
typedef void (*ble_doseIO_C_time_synch_handler_t) (uint16_t conn_handle, ble_doseIO_Calendare_t * p_doseIO_Calendare, uint32_t data);
typedef void (*ble_doseIO_C_write_notif_handler_t) (uint16_t conn_handle, ble_doseIO_Calendare_t * p_doseIO_Calendare, uint32_t data);
typedef void (*ble_doseIO_C_clear_notif_handler_t) (uint16_t conn_handle, ble_doseIO_Calendare_t * p_doseIO_Calendare, uint32_t data);
typedef void (*ble_doseIO_C_list_notif_handler_t) (uint16_t conn_handle, ble_doseIO_Calendare_t * p_doseIO_Calendare, uint32_t data);

/** @brief doseIO Service init structure. This structure contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_doseIO_synch_time_handler_t			synch_time_handler; /**< Event handler to be called when the synch_time Characteristic is written. */
		ble_doseIO_set_notif_handler_t			set_notif_handler; /**< Event handler to be called when the set_notif Characteristic is written. */
		
		ble_doseIO_J_read_handler_t 				Journal_read_handler;
		ble_doseIO_J_clear_handler_t 				Journal_clear_handler;
	
		ble_doseIO_C_time_synch_handler_t 	Calendare_time_synch_handler;
		ble_doseIO_C_write_notif_handler_t	Calendare_write_notif_handler;
		ble_doseIO_C_clear_notif_handler_t	Calendare_clear_notif_handler;
		ble_doseIO_C_list_notif_handler_t		Calendare_list_notif_handler;
} ble_doseIO_init_t;

/**@brief doseIO Service structure. This structure contains various status information for the service. */
struct ble_doseIO_s
{
    uint16_t                    		service_handle;      			/**< Handle of doseIO Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t    		synch_time_char_handles;	/**< Handles related to the synch_time Characteristic. */
		ble_gatts_char_handles_t    		set_notif_char_handles;		/**< Handles related to the set_notif Characteristic. */
    ble_gatts_char_handles_t    		button_char_handles; 			/**< Handles related to the Button Characteristic. */
		ble_gatts_char_handles_t    		button_char_handles2; 		/**< Handles related to the Button Characteristic. */
    uint8_t                     		uuid_type;           			/**< UUID type for the doseIO Service. */
    ble_doseIO_synch_time_handler_t synch_time_handler;   		/**< Event handler to be called when the synch_time Characteristic is written. */
		ble_doseIO_set_notif_handler_t 	set_notif_handler;   			/**< Event handler to be called when the set_notif Characteristic is written. */
};

struct ble_doseIO_Journal_s
{
	uint16_t                    				service_handle;
	uint8_t                     				uuid_type;
	ble_doseIO_J_read_handler_t  				read_handler;
	ble_doseIO_J_clear_handler_t  			clear_handler;
	ble_gatts_char_handles_t 						read_handles;
	ble_gatts_char_handles_t 						clear_handles;
};

struct ble_doseIO_Calendare_s
{
	uint16_t                    					service_handle;
	uint8_t                     					uuid_type;
	ble_doseIO_C_time_synch_handler_t    	time_synch_handler; 
	ble_doseIO_C_write_notif_handler_t    write_notif_handler; 
	ble_doseIO_C_clear_notif_handler_t    clear_notif_handler;
	ble_doseIO_C_list_notif_handler_t    	list_notif_handler;
	ble_gatts_char_handles_t    					time_synch_handles;
	ble_gatts_char_handles_t    					write_notif_handles;
	ble_gatts_char_handles_t    					clear_notif_handles;
	ble_gatts_char_handles_t    					list_notif_handles;
};


/**@brief Function for initializing the doseIO Service.
 *
 * @param[out] p_doseIO      doseIO Service structure. This structure must be supplied by
 *                        the application. It is initialized by this function and will later
 *                        be used to identify this particular service instance.
 * @param[in] p_doseIO_init  Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was initialized successfully. Otherwise, an error code is returned.
 */
uint32_t ble_doseIO_init_s_settings(ble_doseIO_t * p_doseIO, const ble_doseIO_init_t * p_doseIO_init);

uint32_t ble_doseIO_init_s_journal(ble_doseIO_Journal_t * p_doseIO_Journal, const ble_doseIO_init_t * p_doseIO_init);

uint32_t ble_doseIO_init_s_calendare(ble_doseIO_Calendare_t * p_doseIO, const ble_doseIO_init_t * p_doseIO_init);


/**@brief Function for handling the application's BLE stack events.
 *
 * @details This function handles all events from the BLE stack that are of interest to the doseIO Service.
 *
 * @param[in] p_ble_evt  Event received from the BLE stack.
 * @param[in] p_context  doseIO Service structure.
 */
void ble_doseIO_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);
void ble_doseIO_Journal_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);
void ble_doseIO_Calendare_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


/**@brief Function for sending a button state notification.
 *
 ' @param[in] conn_handle   Handle of the peripheral connection to which the button state notification will be sent.
 * @param[in] p_doseIO         doseIO Service structure.
 * @param[in] button_state  New button state.
 *
 * @retval NRF_SUCCESS If the notification was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_doseIO_on_button_change(uint16_t conn_handle, ble_doseIO_t * p_doseIO, uint8_t button_state);

uint32_t ble_list_notif_data_send(ble_doseIO_Calendare_t * p_doseIO_Calendare, uint8_t   * p_data, uint16_t NotifId, uint16_t NotifIdMax, uint16_t    conn_handle);

uint32_t ble_J_read_data_send(ble_doseIO_Journal_t * p_doseIO_Journal, uint8_t* p_data, uint16_t EventId, uint16_t EventIdMax, uint16_t    conn_handle);


#ifdef __cplusplus
}
#endif

#endif // BLE_DOSEIO_S_H__

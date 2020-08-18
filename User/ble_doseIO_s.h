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
	
//15f2e42e-c26d-11ea-b3de-0242ac130004
#define doseIO_UUID_BASE					{0x04, 0x00, 0x13, 0xac, 0x42, 0x02, 0xde, 0xb3, \
																	 0xea, 0x11, 0x6d, 0xc2, 0x2e, 0xe4, 0xf2, 0x15}
																	 
#define doseIO_UUID_SERVICE     				0xab00
#define doseIO_UUID_SYNCH_TIME_CHAR    	0xab01
#define doseIO_UUID_SET_NOTIF_CHAR    	0xab02
#define doseIO_UUID_BUTTON_CHAR 				0xab09
#define doseIO_UUID_BUTTON_CHAR2 				0xab0a


// Forward declaration of the ble_doseIO_t type.
typedef struct ble_doseIO_s ble_doseIO_t;

typedef void (*ble_doseIO_synch_time_handler_t) (uint16_t conn_handle, ble_doseIO_t * p_doseIO, uint32_t data);
typedef void (*ble_doseIO_set_notif_handler_t) (uint16_t conn_handle, ble_doseIO_t * p_doseIO, uint32_t data);

/** @brief doseIO Service init structure. This structure contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_doseIO_synch_time_handler_t		synch_time_handler; /**< Event handler to be called when the synch_time Characteristic is written. */
		ble_doseIO_set_notif_handler_t		set_notif_handler; /**< Event handler to be called when the set_notif Characteristic is written. */
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


/**@brief Function for initializing the doseIO Service.
 *
 * @param[out] p_doseIO      doseIO Service structure. This structure must be supplied by
 *                        the application. It is initialized by this function and will later
 *                        be used to identify this particular service instance.
 * @param[in] p_doseIO_init  Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was initialized successfully. Otherwise, an error code is returned.
 */
uint32_t ble_doseIO_init(ble_doseIO_t * p_doseIO, const ble_doseIO_init_t * p_doseIO_init);


/**@brief Function for handling the application's BLE stack events.
 *
 * @details This function handles all events from the BLE stack that are of interest to the doseIO Service.
 *
 * @param[in] p_ble_evt  Event received from the BLE stack.
 * @param[in] p_context  doseIO Service structure.
 */
void ble_doseIO_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


/**@brief Function for sending a button state notification.
 *
 ' @param[in] conn_handle   Handle of the peripheral connection to which the button state notification will be sent.
 * @param[in] p_doseIO         doseIO Service structure.
 * @param[in] button_state  New button state.
 *
 * @retval NRF_SUCCESS If the notification was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_doseIO_on_button_change(uint16_t conn_handle, ble_doseIO_t * p_doseIO, uint8_t button_state);


#ifdef __cplusplus
}
#endif

#endif // BLE_DOSEIO_S_H__

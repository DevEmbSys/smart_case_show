/**
 * Copyright (c) 2015 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/**
 * @brief Blinky Sample Application main file.
 *
 * This file contains the source code for a sample server application using the doseIO service.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
//#include "boards.h"
#include "app_timer.h"
#include "app_button.h"
//#include "ble_doseIO.h"
#include "ble_doseIO_s.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

//#include "nrf_log.h"
//#include "nrf_log_ctrl.h"
//#include "NRF_LOG_DEBUG_backends.h"
#include "sdk_errors.h"

//#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"

#include "ble_dfu.h"

#include "nrf_dfu_ble_svci_bond_sharing.h"
#include "nrf_svci_async_function.h"
#include "nrf_svci_async_handler.h"

#include "nrf_calendar.h"

#define ADVERTISING_LED                 BSP_BOARD_LED_0                         /**< Is on when device is advertising. */
#define CONNECTED_LED                   BSP_BOARD_LED_1                         /**< Is on when device has connected. */
#define LEDBUTTON_LED                   BSP_BOARD_LED_2                         /**< LED to be toggled with the help of the doseIO Service. */
#define LEDBUTTON_BUTTON                BSP_BUTTON_0                            /**< Button that will trigger the notification event with the doseIO Service */

#define DEVICE_NAME                     "doseIO_02"                         /**< Name of device. Will be included in the advertising data. */

#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL                800                                      /**< The advertising interval ( 60 in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_DURATION                BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED   /**< The advertising time-out (in units of seconds). When set to 0, we will never time out. */


#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000)                  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000)                   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                     /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define SAMPLES_IN_BUFFER 2
volatile uint8_t state = 1;

//static const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(0);
static int16_t     ADC_DATA_RAW[2][SAMPLES_IN_BUFFER];
//static nrf_ppi_channel_t     m_ppi_channel;
static uint32_t              m_adc_evt_counter;

BLE_doseIO_DEF(m_doseIO);                                                             /**< doseIO Service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */

static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;                   /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];                    /**< Buffer for storing an encoded advertising set. */
static uint8_t m_enc_scan_response_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX];         /**< Buffer for storing an encoded scan data. */

uint8_t ConectedFlag = 0;

uint16_t OPEN_DATA_SET_THRESHOLD = 0x300;

#define BatVoltag_RawArr_MAX 9
uint16_t BatVoltag_RawArr[BatVoltag_RawArr_MAX];
uint8_t BatVoltag_RawArr_count = 0;

#define ADC_DATA_Arr_MAX 20
uint8_t ADC_DATA_Arr_count = 0;
uint16_t ADC_DATA_Arr[ADC_DATA_Arr_MAX] = {0};
uint16_t ADC_DATA_AVERAGE = 0, ADC_DATA_AVERAGE_LOW = 0, ADC_DATA_AVERAGE_HIGH = 0;
uint32_t ADC_DATA_SUMM = 0;
uint16_t OPEN_DATA_HYST = 2;

struct tm* tm_data = 0;

#define JournalLengthMax 0x1000

#define JournalADDR 			0x6C000
#define JournalCopyADDR 	0x6D000

#define ListNotifADDR			0x6E000
#define ListNotifCopyADDR	0x6F000

#define ListNotifMAX 0x1000
typedef enum{
	EVENT_SET_NOTIFICATION = 0,	// установка оповещения
	EVENT_GET_NEW_EVENT,		// получение новых событий
	EVENT_CASE_OPEN,
	EVENT_CASE_CLOSE,
	EVENT_NOTIFICATION_ACTIVE,
	EVENT_NOTIFICATION_DONE,
	EVENT_NONE = 255
}e_TypeEvent;

typedef struct __attribute__ ((aligned(32))){
	time_t 				Time;
	uint32_t			Value:16;
	e_TypeEvent		TypeEvent;
}s_JournalData;

typedef struct __attribute__ ((aligned(32))){
	s_JournalData Data[128];
}s_Journal;

typedef struct __attribute__ ((aligned(32))){
	time_t Time[128];
}s_ListNotif;

#define Journal ((s_Journal*)JournalADDR)
uint32_t JournalPointer = 0;

#define ListNotif ((s_ListNotif*)ListNotifADDR)
uint32_t ListNotifPointer = 0;

/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = m_enc_scan_response_data,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX

    }
};

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    //bsp_board_init(BSP_INIT_LEDS);
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    // Initialize timer module, making it use the scheduler
    uint32_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    uint32_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t    err_code;
    ble_advdata_t advdata;
    ble_advdata_t srdata;

    ble_uuid_t adv_uuids[] = {{doseIO_UUID_SERVICE, m_doseIO.uuid_type}};

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = true;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;


    memset(&srdata, 0, sizeof(srdata));
    srdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    srdata.uuids_complete.p_uuids  = adv_uuids;

    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = ble_advdata_encode(&srdata, m_adv_data.scan_rsp_data.p_data, &m_adv_data.scan_rsp_data.len);
    APP_ERROR_CHECK(err_code);

    ble_gap_adv_params_t adv_params;

    // Set advertising parameters.
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.primary_phy     = BLE_GAP_PHY_1MBPS;
    adv_params.duration        = APP_ADV_DURATION;
    adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    adv_params.p_peer_addr     = NULL;
    adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    adv_params.interval        = APP_ADV_INTERVAL;

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &adv_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling write events to the synch_time characteristic.
 *
 * @param[in] p_doseIO			Instance of doseIO Service to which the write applies.
 * @param[in] data					Writed in format Unix (start 1 January 1970 00:00:00 GMT) of the synch_time.
 */
static void synch_time_handler(uint16_t conn_handle, ble_doseIO_t * p_doseIO, uint32_t data)
{
	//nrf_cal_set_time_Unix(data);
	if(ListNotifPointer < 128)
	{
		// Enable write.
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
    }

    /* Only full 32-bit words can be written to Flash. */
		*(uint32_t*)(ListNotif->Time[ListNotifPointer]) = data;
		while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
		{
		}

    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
    }
		ListNotifPointer++;
	}
}

/**@brief Function for handling write events to the synch_time characteristic.
 *
 * @param[in] p_doseIO			Instance of doseIO Service to which the write applies.
 * @param[in] data					Writed in format Unix (start 1 January 1970 00:00:00 GMT) of the synch_time.
 */
static void set_notif_handler(uint16_t conn_handle, ble_doseIO_t * p_doseIO, uint32_t data)
{
	if(ListNotifPointer < 128)
	{
		// Enable write.
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
    }

    /* Only full 32-bit words can be written to Flash. */
		*(uint32_t*)(ListNotif->Time[ListNotifPointer]) = data;
		while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
		{
		}

    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
    }
		ListNotifPointer++;
	}
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t         err_code;
    ble_doseIO_init_t     init     = {0};
    nrf_ble_qwr_init_t qwr_init = {0};
		ble_dfu_buttonless_init_t dfus_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize doseIO.
    init.synch_time_handler = synch_time_handler;
		init.set_notif_handler = set_notif_handler;

    err_code = ble_doseIO_init(&m_doseIO, &init);
    APP_ERROR_CHECK(err_code);
		
		err_code = ble_dfu_buttonless_init(&dfus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module that
 *          are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply
 *       setting the disconnect_on_fail config parameter, but instead we use the event
 *       handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t           err_code;

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);

    //bsp_board_led_on(ADVERTISING_LED);
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
						ConectedFlag = 1;
            //NRF_LOG_INFO("Connected");
            //bsp_board_led_on(CONNECTED_LED);
            //bsp_board_led_off(ADVERTISING_LED);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            //err_code = app_button_enable();
            APP_ERROR_CHECK(err_code);
						nrf_gpio_pin_set(8);
						NRFX_DELAY_US(100000);
						nrf_gpio_pin_clear(8);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
						ConectedFlag = 0;
            //NRF_LOG_INFO("Disconnected");
            //bsp_board_led_off(CONNECTED_LED);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            //err_code = app_button_disable();
            APP_ERROR_CHECK(err_code);
            advertising_start();
						nrf_gpio_pin_set(7);
						NRFX_DELAY_US(100000);
						nrf_gpio_pin_clear(7);
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                                   NULL,
                                                   NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            //NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            //NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            //NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
//    uint32_t err_code;

//    switch (pin_no)
//    {
//        case LEDBUTTON_BUTTON:
//            //NRF_LOG_INFO("Send button state change.");
//            err_code = ble_doseIO_on_button_change(m_conn_handle, &m_doseIO, button_action);
//            if (err_code != NRF_SUCCESS &&
//                err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
//                err_code != NRF_ERROR_INVALID_STATE &&
//                err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
//            {
//                APP_ERROR_CHECK(err_code);
//            }
//            break;

//        default:
//            APP_ERROR_HANDLER(pin_no);
//            break;
//    }
}


/**@brief Function for initializing the button handler module.
 */
static void buttons_init(void)
{
//    uint32_t err_code;

//    //The array must be static because a pointer to it will be saved in the button handler module.
//    static app_button_cfg_t buttons[] =
//    {
//        {LEDBUTTON_BUTTON, false, BUTTON_PULL, button_event_handler}
//    };

//    err_code = app_button_init(buttons, ARRAY_SIZE(buttons),
//                               BUTTON_DETECTION_DELAY);
//    APP_ERROR_CHECK(err_code);
}


static void log_init(void)
{
    //uint32_t err_code = NRF_LOG_INIT(NULL);
    //APP_ERROR_CHECK(err_code);

    //NRF_LOG_DEBUG_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    uint32_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */

static void idle_state_handle(void)
{
	uint16_t CaseOpen = nrf_gpio_pin_read(27);
	/*
	3/2.045
	*/
		uint16_t BatVoltag = 0;
		nrf_gpio_pin_set(3);
		//nrf_drv_saadc_sample();
		
		BatVoltag_RawArr[BatVoltag_RawArr_count] = (uint16_t)(((uint32_t)(ADC_DATA_RAW[0][1]*720))/1024);
		BatVoltag_RawArr_count++;
		if(BatVoltag_RawArr_count >= BatVoltag_RawArr_MAX)BatVoltag_RawArr_count = 0;
	
		*((volatile uint32_t *)((uint8_t *)NRF_SAADC + (uint32_t)offsetof(NRF_SAADC_Type, TASKS_SAMPLE))) = 0x1UL; // nrf_drv_saadc_sample
	
		//nrf_gpio_pin_clear(3);
	
//		if(m_buffer_pool[0][0] < 0x300)
//		{
//			nrf_gpio_pin_set(7);
//		}
//		else
//		{
//			nrf_gpio_pin_clear(7);
//		}
	
		ADC_DATA_Arr[ADC_DATA_Arr_count] = ADC_DATA_RAW[0][0];
		ADC_DATA_SUMM += ADC_DATA_RAW[0][0];
		ADC_DATA_Arr_count++;
		if(ADC_DATA_Arr_count >= ADC_DATA_Arr_MAX)
		{
			ADC_DATA_AVERAGE = ADC_DATA_SUMM / ADC_DATA_Arr_count; 
			ADC_DATA_SUMM = 0;
			ADC_DATA_Arr_count = 0;
		}
		
		if(CaseOpen)
		{
			nrf_gpio_pin_set(7);
			NRFX_DELAY_US(100000);
			nrf_gpio_pin_clear(7);
			NRFX_DELAY_US(1000000);
		}

		if(ConectedFlag == 1)
		{
			ble_gatts_hvx_params_t params;
			uint16_t len = 0;
			tm_data = nrf_cal_get_time();
			
			len = sizeof(ADC_DATA_RAW[0][0]);
			memset(&params, 0, sizeof(params));
			params.type   = BLE_GATT_HVX_NOTIFICATION;
			params.handle = m_doseIO.button_char_handles.value_handle;
			params.p_data = (uint8_t*)&ADC_DATA_RAW[0][1];
			params.p_len  = &len;

			sd_ble_gatts_hvx(m_conn_handle, &params);
			
			for(uint8_t i = 0; i < 5; i++)
			{
				uint16_t BatVoltagMinCount = BatVoltag_RawArr[0];
				for(uint8_t j = 0; j < BatVoltag_RawArr_MAX; j++)
				{
					if(BatVoltag_RawArr[j] >= BatVoltag && BatVoltagMinCount < BatVoltag_RawArr[j] && BatVoltag_RawArr[j] != 0)
					{
						BatVoltagMinCount = BatVoltag_RawArr[j];
					}
				}
				BatVoltag = BatVoltagMinCount;
			}
			len = sizeof(BatVoltag);
			memset(&params, 0, sizeof(params));
			params.type   = BLE_GATT_HVX_NOTIFICATION;
			params.handle = m_doseIO.button_char_handles2.value_handle;
			params.p_data = (uint8_t*)&CaseOpen;
			params.p_len  = &len;
			
			sd_ble_gatts_hvx(m_conn_handle, &params);
		}
		
    nrf_pwr_mgmt_run();
}


//void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
//{
//    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
//    {
//        ret_code_t err_code;

//        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
//        APP_ERROR_CHECK(err_code);

//        int i;
//        m_adc_evt_counter++;
//    }
//}


void saadc_init(void)
{
		uint32_t event = 0; 
		//Step 1 - INIT
    //err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    //APP_ERROR_CHECK(err_code);
	
		NRF_SAADC->RESOLUTION = NRFX_SAADC_CONFIG_RESOLUTION;
		NRF_SAADC->OVERSAMPLE = NRFX_SAADC_CONFIG_OVERSAMPLE;
	
		NRF_SAADC->INTENCLR = 0x7FFFFFFFUL;
	
		event = offsetof(NRF_SAADC_Type, EVENTS_END);
		*((volatile uint32_t *)((uint8_t *)NRF_SAADC + (uint32_t)event)) = 0x0UL;
    (void)*((volatile uint32_t *)((uint8_t *)NRF_SAADC + (uint32_t)event));
	
		event = offsetof(NRF_SAADC_Type, EVENTS_STARTED);
		*((volatile uint32_t *)((uint8_t *)NRF_SAADC + (uint32_t)event)) = 0x0UL;
    (void)*((volatile uint32_t *)((uint8_t *)NRF_SAADC + (uint32_t)event));
	
		event = offsetof(NRF_SAADC_Type, EVENTS_STOPPED);
		*((volatile uint32_t *)((uint8_t *)NRF_SAADC + (uint32_t)event)) = 0x0UL;
    (void)*((volatile uint32_t *)((uint8_t *)NRF_SAADC + (uint32_t)event));
	
		NRFX_IRQ_PRIORITY_SET(SAADC_IRQn, NRFX_SAADC_CONFIG_IRQ_PRIORITY);
    NRFX_IRQ_ENABLE(SAADC_IRQn);
		
		NRF_SAADC->INTENSET = SAADC_INTENSET_END_Msk;
	
		if(NRFX_SAADC_CONFIG_LP_MODE)
		{
			NRF_SAADC->INTENSET = SAADC_INTENSET_STARTED_Msk;
		}
		
		NRF_SAADC->ENABLE = (SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos);
		
		//Step 2
    //err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    //APP_ERROR_CHECK(err_code);
		
		NRF_SAADC->ENABLE = (SAADC_ENABLE_ENABLE_Disabled << SAADC_ENABLE_ENABLE_Pos);
		
		NRF_SAADC->CH[0].CONFIG =
            ((SAADC_CH_CONFIG_RESP_Bypass   << SAADC_CH_CONFIG_RESP_Pos)   & SAADC_CH_CONFIG_RESP_Msk)
            | ((SAADC_CH_CONFIG_RESP_Bypass << SAADC_CH_CONFIG_RESN_Pos)   & SAADC_CH_CONFIG_RESN_Msk)
            | ((SAADC_CH_CONFIG_GAIN_Gain1_6       << SAADC_CH_CONFIG_GAIN_Pos)   & SAADC_CH_CONFIG_GAIN_Msk)
            | ((SAADC_CH_CONFIG_REFSEL_Internal  << SAADC_CH_CONFIG_REFSEL_Pos) & SAADC_CH_CONFIG_REFSEL_Msk)
            | ((SAADC_CH_CONFIG_TACQ_10us   << SAADC_CH_CONFIG_TACQ_Pos)   & SAADC_CH_CONFIG_TACQ_Msk)
            | ((SAADC_CH_CONFIG_MODE_SE       << SAADC_CH_CONFIG_MODE_Pos)   & SAADC_CH_CONFIG_MODE_Msk)
            | ((SAADC_CH_CONFIG_BURST_Disabled      << SAADC_CH_CONFIG_BURST_Pos)  & SAADC_CH_CONFIG_BURST_Msk);
		
		NRF_SAADC->CH[0].PSELN = 0;
    NRF_SAADC->CH[0].PSELP = SAADC_CH_PSELP_PSELP_AnalogInput0;
		
		NRF_SAADC->CH[1].CONFIG =
            ((SAADC_CH_CONFIG_RESP_Bypass   << SAADC_CH_CONFIG_RESP_Pos)   & SAADC_CH_CONFIG_RESP_Msk)
            | ((SAADC_CH_CONFIG_RESP_Bypass << SAADC_CH_CONFIG_RESN_Pos)   & SAADC_CH_CONFIG_RESN_Msk)
            | ((SAADC_CH_CONFIG_GAIN_Gain1_6       << SAADC_CH_CONFIG_GAIN_Pos)   & SAADC_CH_CONFIG_GAIN_Msk)
            | ((SAADC_CH_CONFIG_REFSEL_Internal  << SAADC_CH_CONFIG_REFSEL_Pos) & SAADC_CH_CONFIG_REFSEL_Msk)
            | ((SAADC_CH_CONFIG_TACQ_10us   << SAADC_CH_CONFIG_TACQ_Pos)   & SAADC_CH_CONFIG_TACQ_Msk)
            | ((SAADC_CH_CONFIG_MODE_SE       << SAADC_CH_CONFIG_MODE_Pos)   & SAADC_CH_CONFIG_MODE_Msk)
            | ((SAADC_CH_CONFIG_BURST_Disabled      << SAADC_CH_CONFIG_BURST_Pos)  & SAADC_CH_CONFIG_BURST_Msk);
		
		NRF_SAADC->CH[1].PSELN = 0;
    NRF_SAADC->CH[1].PSELP = SAADC_CH_PSELP_PSELP_AnalogInput7;
		
		NRF_SAADC->ENABLE = (SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos);

		//Step 3
//    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
//    APP_ERROR_CHECK(err_code);

		NRF_SAADC->RESULT.PTR = (uint32_t)ADC_DATA_RAW[0];
    NRF_SAADC->RESULT.MAXCNT = SAMPLES_IN_BUFFER;
		
		event = offsetof(NRF_SAADC_Type, EVENTS_STARTED);
		*((volatile uint32_t *)((uint8_t *)NRF_SAADC + (uint32_t)event)) = 0x0UL;
    (void)*((volatile uint32_t *)((uint8_t *)NRF_SAADC + (uint32_t)event));
		
		*((volatile uint32_t *)((uint8_t *)NRF_SAADC + (uint32_t)offsetof(NRF_SAADC_Type, TASKS_START))) = 0x1UL;
}

// This function initializes timer 3 with the following configuration:
// 24-bit, base frequency 16 MHz, auto clear on COMPARE5 match (CC5 = TIMER_RELOAD)
void timer_init()
{
    NRF_TIMER3->BITMODE                 = TIMER_BITMODE_BITMODE_24Bit << TIMER_BITMODE_BITMODE_Pos;
    NRF_TIMER3->PRESCALER               = 8000;
    NRF_TIMER3->SHORTS                  = TIMER_SHORTS_COMPARE0_CLEAR_Msk << 5;
    NRF_TIMER3->MODE                    = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;
    NRF_TIMER3->CC[5] = 1024;    
}

// Starts TIMER3
void timer_start()
{
    NRF_TIMER3->TASKS_START = 1;
}

// This function sets up TIMER3, the PPI and the GPIOTE modules to configure a single PWM channel
// Timer CC num, PPI channel nums and GPIOTE channel num is defined at the top of this file
void pwm_init(uint32_t pinselect)
{  
    NRF_GPIOTE->CONFIG[0] = GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos | 
                                         GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos | 
                                         pinselect << GPIOTE_CONFIG_PSEL_Pos | 
                                         GPIOTE_CONFIG_OUTINIT_High << GPIOTE_CONFIG_OUTINIT_Pos;

    NRF_PPI->CH[0].EEP = (uint32_t)&NRF_TIMER3->EVENTS_COMPARE[0];
    NRF_PPI->CH[0].TEP = (uint32_t)&NRF_GPIOTE->TASKS_CLR[0];
    NRF_PPI->CH[1].EEP = (uint32_t)&NRF_TIMER3->EVENTS_COMPARE[0];
    NRF_PPI->CH[1].TEP = (uint32_t)&NRF_GPIOTE->TASKS_SET[0];
    
    NRF_PPI->CHENSET               = (1 << 0) | (1 << 1);
}

void calendar_updated()
{
	
}

/**@brief Function for application main entry.
 */
int main(void)
{
		volatile uint8_t EraseJournalFlag = 0;
		volatile uint8_t EraseListNotifFlag = 0;
		if(EraseListNotifFlag == 1)
		{
			// Enable erase.
			NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een;
			__ISB();
			__DSB();

			// Erase the page
			NRF_NVMC->ERASEPAGE = ListNotifADDR;
			while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {;}

			NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
			__ISB();
			__DSB();
		}
		if(EraseJournalFlag == 1)
		{
			// Enable erase.
			NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een;
			__ISB();
			__DSB();

			// Erase the page
			NRF_NVMC->ERASEPAGE = JournalADDR;
			while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {;}

			NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
			__ISB();
			__DSB();
		}
		
		if(JournalPointer == 0)
		{
			for(uint32_t i = 0; i < (0x1000/32); i++)
			{
				if(Journal->Data[i].TypeEvent == EVENT_NONE)
				{
					JournalPointer = i;
					break;
				}
			}
		}
		
		if(ListNotifPointer == 0)
		{
			for(uint32_t i = 0; i < (0x1000/32); i++)
			{
				if(ListNotif->Time[i] == 0xffffffff)
				{
					ListNotifPointer = i;
					break;
				}
			}
		}
		
		NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while(NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
	
		nrf_cal_init();
		nrf_cal_set_callback(calendar_updated, 4);
    // Initialize.
    log_init();
    leds_init();
    timers_init();
    buttons_init();
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();

    // Start execution.
    //NRF_LOG_INFO("Blinky example started.");
    advertising_start();
	
		saadc_init();
	
		nrf_gpio_cfg_output(3);
		nrf_gpio_cfg_output(7);
		nrf_gpio_cfg_output(8);
		
		NRF_GPIO->PIN_CNF[29] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)		
                                  | (GPIO_PIN_CNF_DRIVE_S0D1 << GPIO_PIN_CNF_DRIVE_Pos)
				  				  | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
								  | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
								  | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
									
		NRF_GPIO->PIN_CNF[27] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)		
                                  | (GPIO_PIN_CNF_DRIVE_S0D1 << GPIO_PIN_CNF_DRIVE_Pos)
				  				  | (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos)
								  | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
								  | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
									
		nrf_gpio_pin_set(27);
									
		nrf_gpio_pin_clear(29);
		
		nrf_gpio_cfg_input(27,NRF_GPIO_PIN_PULLUP);
		
		timer_init();
		pwm_init(12);
		timer_start();
		
		//NRF_TIMER3->CC[0] = 512;
		
		nrf_cal_set_time(2020,07,06,18,00,00);

    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}

void nrfx_saadc_irq_handler(void)
{
	if ((bool)*(volatile uint32_t *)((uint8_t *)NRF_SAADC + (uint32_t)offsetof(NRF_SAADC_Type, EVENTS_END)))
	{
		*((volatile uint32_t *)((uint8_t *)NRF_SAADC + (uint32_t)offsetof(NRF_SAADC_Type, EVENTS_END))) = 0x0UL;
    (void)*((volatile uint32_t *)((uint8_t *)NRF_SAADC + (uint32_t)offsetof(NRF_SAADC_Type, EVENTS_END)));
		
		*((volatile uint32_t *)((uint8_t *)NRF_SAADC + (uint32_t)offsetof(NRF_SAADC_Type, TASKS_START))) = 0x1UL;
	}
	if (NRFX_SAADC_CONFIG_LP_MODE && (bool)*(volatile uint32_t *)((uint8_t *)NRF_SAADC + (uint32_t)offsetof(NRF_SAADC_Type, EVENTS_STARTED)))
	{
		
	}
}


/**
 * @}
 */

#include <stdint.h>
#include <string.h>
#include "sys_battery.h"
#include "ble_bas.h"
#include "app_error.h"
#include "sys_battery.h"
#include "nrf_drv_saadc.h"
#include "sdk_macros.h"
#include "app_timer.h"

#define ADC_REF_VOLTAGE_IN_MILLIVOLTS  600  //!< Reference voltage (in milli volts) used by ADC while doing conversion.
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS 270  //!< Typical forward voltage drop of the diode (Part no: SD103ATW-7-F) that is connected in series with the voltage supply. This is the voltage drop when the forward current is 1mA. Source: Data sheet of 'SURFACE MOUNT SCHOTTKY BARRIER DIODE ARRAY' available at www.diodes.com.
#define ADC_RES_10BIT                  1024 //!< Maximum digital value for 10-bit ADC conversion.
#define ADC_PRE_SCALING_COMPENSATION   6    //!< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE) \
    ((((ADC_VALUE) *ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_PRE_SCALING_COMPENSATION)

static nrf_saadc_value_t adc_buf;                   //!< Buffer used for storing ADC value.
static uint16_t          m_batt_lvl_in_milli_volts; //!< Current battery level.

#define BATTERY_LEVEL_MEAS_INTERVAL APP_TIMER_TICKS(3000)

BLE_BAS_DEF(m_bas);                                                 /**< Structure used to identify the battery service. */
APP_TIMER_DEF(m_adc_timer_id);                                                      /**< Timer ID used with the timer module. */

static void saadc_event_handler(nrf_drv_saadc_evt_t const * p_evt)
{
    if (p_evt->type == NRF_DRV_SAADC_EVT_DONE)
    {
        nrf_saadc_value_t adc_result;
        uint8_t percentage_batt_level;
        uint32_t err_code;
		
        adc_result = p_evt->data.done.p_buffer[0];
        err_code = nrf_drv_saadc_buffer_convert(p_evt->data.done.p_buffer,1);
        APP_ERROR_CHECK(err_code);
        m_batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_result);

        percentage_batt_level = battery_level_in_percent(m_batt_lvl_in_milli_volts);
		
        err_code = ble_bas_battery_level_update(&m_bas,percentage_batt_level,BLE_CONN_HANDLE_ALL);
        if((err_code != NRF_SUCCESS)&&
            (err_code != NRF_ERROR_INVALID_STATE)&&
            (err_code != NRF_ERROR_RESOURCES)&&
            (err_code != NRF_ERROR_BUSY)&&
            (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
          )
        {
            APP_ERROR_HANDLER(err_code);
        }
    }
}

void battery_level_meas_timeout_handler(void *p_context)
{
    UNUSED_PARAMETER(p_context);

	ret_code_t err_code;
	
	battery_voltage_init();

	err_code =  nrf_drv_saadc_sample();
	APP_ERROR_CHECK(err_code);
}
void battery_voltage_init(void)
{
    ret_code_t err_code = nrf_drv_saadc_init(NULL, saadc_event_handler);

    APP_ERROR_CHECK(err_code);

    nrf_saadc_channel_config_t config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN3);
    err_code = nrf_drv_saadc_channel_init(0, &config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(&adc_buf, 1);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_sample();
    APP_ERROR_CHECK(err_code);
}


 void battery_voltage_get(uint16_t * p_vbatt)
{
    VERIFY_PARAM_NOT_NULL_VOID(p_vbatt);

    *p_vbatt = m_batt_lvl_in_milli_volts;
    if (!nrf_drv_saadc_is_busy())
    {
        ret_code_t err_code = nrf_drv_saadc_buffer_convert(&adc_buf, 1);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_drv_saadc_sample();
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for handling the Battery Service events.
 *
 * @details This function will be called for all Battery Service events which are passed to the
 |          application.
 *
 * @param[in] p_bas  Battery Service structure.
 * @param[in] p_evt  Event received from the Battery Service.
 */
static void on_bas_evt(ble_bas_t * p_bas, ble_bas_evt_t * p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_type)
    {
        case BLE_BAS_EVT_NOTIFICATION_ENABLED:
            // Start battery timer
            err_code = app_timer_start(m_adc_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_BAS_EVT_NOTIFICATION_ENABLED

        case BLE_BAS_EVT_NOTIFICATION_DISABLED:
            err_code = app_timer_stop(m_adc_timer_id);
            APP_ERROR_CHECK(err_code);
            break; // BLE_BAS_EVT_NOTIFICATION_DISABLED

        default:
            // No implementation needed.
            break;
    }
}

void bas_timer_init(void)
{
    ret_code_t err_code;
	
    err_code = app_timer_create(&m_adc_timer_id, APP_TIMER_MODE_REPEATED, battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
}

//battery service init
void bas_init(void)
{
    uint32_t       err_code;
	ble_bas_init_t     bas_init;
	
    // Here the sec level for the Battery Service can be changed/increased.
    bas_init.bl_rd_sec        = SEC_OPEN;
    bas_init.bl_cccd_wr_sec   = SEC_OPEN;
    bas_init.bl_report_rd_sec = SEC_OPEN;

    bas_init.evt_handler          = on_bas_evt;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);
}


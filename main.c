#include "sdk_config.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app_button.h"
#include "app_error.h"
#include "app_scheduler.h"
#include "app_timer.h"
#include "ble.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "fds.h"
#include "nrf.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"

#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_bootloader_info.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_drv_clock.h"
#include "nrf_drv_power.h"
#include "nrf_drv_saadc.h"

#include "nrfx_saadc.h"

#include "ble_dis.h"

#include "nrf_delay.h"
#include "nrf_gpio.h"

#include "ble_bas.h"

#include "nrf_crypto.h"

#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE
#define SCHED_QUEUE_SIZE 10

#define DEVICE_NAME "remote_fob"
#define MANUFACTURER_NAME "company"
#define MODEL_NUMBER "fob_gen_1"

#define BUTTON_DETECTION_DELAY APP_TIMER_TICKS(50)
#define BUTTON_PULL NRF_GPIO_PIN_PULLUP
#define LED_PIN 10
#define BUTTON_PIN 13

#define LED_OFF 1
#define LED_ON 0

#define VOLTAGE_READ_SAMPLES 5
#define ADC_REF_VOLTAGE_IN_MILLIVOLTS 600
#define ADC_PRE_SCALING_COMPENSATION 6
#define ADC_RES_10_BIT 1024

#define VBAT_MV_MAX 3000
#define VBAT_MV_MIN 2000

#define ADVERTISING_TIMEOUT_SEC 10
#define APP_BLE_CONN_CFG_TAG 1
#define APP_BLE_OBSERVER_PRIO 3

#define APP_ADV_INTERVAL 300

#define MIN_CONN_INTERVAL MSEC_TO_UNITS(100, UNIT_1_25_MS)
#define MAX_CONN_INTERVAL MSEC_TO_UNITS(200, UNIT_1_25_MS)
#define SLAVE_LATENCY 0
#define CONN_SUP_TIMEOUT MSEC_TO_UNITS(4000, UNIT_10_MS)

#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(5000)
#define NEXT_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(30000)
#define MAX_CONN_PARAMS_UPDATE_COUNT 3

#define SERVICE_UUID 0x0001
#define BUTTON_CHAR_UUID 0x0002
#define VBAT_CHAR_UUID 0x0003

#define ADV_TIMER_TIMEOUT APP_TIMER_TICKS(ADVERTISING_TIMEOUT_SEC * 1000)

NRF_BLE_GATT_DEF(m_gatt);
NRF_BLE_QWR_DEF(m_qwr);
BLE_ADVERTISING_DEF(m_advertising);

BLE_BAS_DEF(m_bas);

APP_TIMER_DEF(m_battery_timer_id);
APP_TIMER_DEF(m_adv_timeout_timer_id);

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;

static ble_uuid_t m_adv_uuids[] = {
    {SERVICE_UUID, BLE_UUID_TYPE_BLE},
    {BLE_UUID_BATTERY_SERVICE, BLE_UUID_TYPE_BLE},
};

static uint16_t m_service_handle;
static ble_gatts_char_handles_t m_button_char_handles;
static ble_gatts_char_handles_t m_vbat_char_handles;

static nrf_saadc_value_t saadc_val;

static uint8_t m_battery_level = 0;

static void advertising_start(void);

static void shutdown_on_error(ret_code_t err_code) {
    if (err_code != NRF_SUCCESS) {
        NRF_LOG_FINAL_FLUSH();
        nrf_gpio_pin_write(LED_PIN, LED_OFF);
        nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
    }
}

static void saadc_callback(nrf_drv_saadc_evt_t const *p_event) {
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE) {
        saadc_val = p_event->data.done.p_buffer[0];
    }
}

static uint16_t read_battery_voltage(void) {
    ret_code_t err_code;
    nrf_saadc_value_t val = 0;

    nrf_drv_saadc_config_t saadc_config = NRF_DRV_SAADC_DEFAULT_CONFIG;
    saadc_config.resolution = NRF_SAADC_RESOLUTION_10BIT;

    err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback);
    if (err_code != NRF_SUCCESS) return 0;

    nrf_saadc_channel_config_t channel_config = NRF_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_VDD);
    channel_config.gain = NRF_SAADC_GAIN1_6;
    channel_config.reference = NRF_SAADC_REFERENCE_INTERNAL;

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    if (err_code != NRF_SUCCESS) {
        nrf_drv_saadc_uninit();
        return 0;
    }

    uint32_t sum = 0;
    for (int i = 0; i < VOLTAGE_READ_SAMPLES; i++) {
        err_code = nrf_drv_saadc_sample_convert(0, &val);
        if (err_code == NRF_SUCCESS) {
            sum += val;
        }
        nrf_delay_us(100);
    }

    nrf_drv_saadc_channel_uninit(0);
    nrf_drv_saadc_uninit();

    uint16_t avg_val = sum / VOLTAGE_READ_SAMPLES;
    uint16_t vbat_mv = (avg_val * ADC_REF_VOLTAGE_IN_MILLIVOLTS * ADC_PRE_SCALING_COMPENSATION) / ADC_RES_10_BIT;

    return vbat_mv;
}

static uint8_t vbat_to_percentage(uint16_t mvolts) {
    if (mvolts >= VBAT_MV_MAX) return 100;
    if (mvolts <= VBAT_MV_MIN) return 0;

    return (uint8_t)(((mvolts - VBAT_MV_MIN) * 100) / (VBAT_MV_MAX - VBAT_MV_MIN));
}

static void battery_timer_handler(void *p_context) {
    uint16_t vbat_mv = read_battery_voltage();
    uint16_t len = sizeof(vbat_mv);
    m_battery_level = vbat_to_percentage(vbat_mv);

    ble_gatts_hvx_params_t hvx_params;
    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = m_vbat_char_handles.value_handle;
    hvx_params.type = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.offset = 0;
    hvx_params.p_len = &len;
    hvx_params.p_data = (uint8_t *)&vbat_mv;

    if (m_conn_handle != BLE_CONN_HANDLE_INVALID) {
        sd_ble_gatts_hvx(m_conn_handle, &hvx_params);
        ble_bas_battery_level_update(&m_bas, m_battery_level, m_conn_handle);
    }
}

static void adv_timeout_handler(void *p_context) {
    nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
}

static void create_timers(void) {
    ret_code_t err_code;

    err_code = app_timer_create(&m_battery_timer_id, APP_TIMER_MODE_REPEATED, battery_timer_handler);
    shutdown_on_error(err_code);

    err_code = app_timer_create(&m_adv_timeout_timer_id, APP_TIMER_MODE_SINGLE_SHOT, adv_timeout_handler);
    shutdown_on_error(err_code);
}

static void start_battery_timer(void) {
    ret_code_t err_code = app_timer_start(m_battery_timer_id, APP_TIMER_TICKS(1000), NULL);
    shutdown_on_error(err_code);
}

static void stop_battery_timer(void) {
    app_timer_stop(m_battery_timer_id);
}

static void start_adv_timer(void) {
    ret_code_t err_code = app_timer_start(m_adv_timeout_timer_id, ADV_TIMER_TIMEOUT, NULL);
    shutdown_on_error(err_code);
}

static void stop_adv_timer(void) {
    app_timer_stop(m_adv_timeout_timer_id);
}

static void button_handler_callback(uint8_t pin_no, uint8_t button_action) {
    static uint8_t last_pin = 0xFF;
    uint16_t len = sizeof(pin_no);

    if (button_action == APP_BUTTON_PUSH) {
        nrf_gpio_pin_write(LED_PIN, LED_ON);
        last_pin = pin_no;

        ble_gatts_hvx_params_t hvx_params;
        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = m_button_char_handles.value_handle;
        hvx_params.type = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len = &len;
        hvx_params.p_data = &pin_no;

        if (m_conn_handle != BLE_CONN_HANDLE_INVALID) {
            sd_ble_gatts_hvx(m_conn_handle, &hvx_params);
        }
    } else if (button_action == APP_BUTTON_RELEASE) {
        if (pin_no == last_pin) {
            nrf_gpio_pin_write(LED_PIN, LED_OFF);
            last_pin = 0xFF;
        }
    }
}

static void init_services(void) {
    ret_code_t err_code;
    ble_uuid_t ble_uuid;
    ble_uuid128_t base_uuid = {
        {0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15,
         0xDE, 0xEF, 0x12, 0x34, 0x00, 0x00, 0x00, 0x00}};

    err_code = sd_ble_uuid_vs_add(&base_uuid, &ble_uuid.type);
    shutdown_on_error(err_code);

    ble_uuid.uuid = SERVICE_UUID;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &m_service_handle);
    shutdown_on_error(err_code);

    // Button Characteristic
    ble_add_char_params_t add_char_params;
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid = BUTTON_CHAR_UUID;
    add_char_params.uuid_type = ble_uuid.type;
    add_char_params.max_len = sizeof(uint8_t);
    add_char_params.init_len = sizeof(uint8_t);
    add_char_params.char_props.read = 1;
    add_char_params.char_props.notify = 1;
    add_char_params.cccd_write_access = SEC_OPEN;
    add_char_params.read_access = SEC_OPEN;

    err_code = characteristic_add(m_service_handle, &add_char_params, &m_button_char_handles);
    shutdown_on_error(err_code);

    // VBAT Characteristic
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid = VBAT_CHAR_UUID;
    add_char_params.uuid_type = ble_uuid.type;
    add_char_params.max_len = sizeof(uint16_t);
    add_char_params.init_len = sizeof(uint16_t);
    add_char_params.char_props.read = 1;
    add_char_params.char_props.notify = 1;
    add_char_params.cccd_write_access = SEC_OPEN;
    add_char_params.read_access = SEC_OPEN;

    err_code = characteristic_add(m_service_handle, &add_char_params, &m_vbat_char_handles);
    shutdown_on_error(err_code);

    // BAS Service
    ble_bas_init_t bas_init;
    memset(&bas_init, 0, sizeof(bas_init));

    bas_init.evt_handler = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref = NULL;
    bas_init.initial_batt_level = 100;

    bas_init.bl_rd_sec = SEC_OPEN;
    bas_init.bl_cccd_write_sec = SEC_OPEN;
    bas_init.bl_report_rd_sec = SEC_OPEN;

    err_code = ble_bas_init(&m_bas, &bas_init);
    shutdown_on_error(err_code);

    // DIS Service
    ble_dis_init_t dis_init;
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, MANUFACTURER_NAME);
    ble_srv_ascii_to_utf8(&dis_init.model_num_str, MODEL_NUMBER);

    dis_init.dis_char_rd_sec = SEC_OPEN;

    err_code = ble_dis_init(&dis_init);
    shutdown_on_error(err_code);
}

static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context) {
    ret_code_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id) {
        case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            stop_battery_timer();
            advertising_start();
            break;

        case BLE_GAP_EVT_CONNECTED:
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            shutdown_on_error(err_code);

            stop_adv_timer();
            start_battery_timer();
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
            ble_gap_phys_t const phys = {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            shutdown_on_error(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            shutdown_on_error(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            shutdown_on_error(err_code);
            break;

        default:
            break;
    }
}

static void init_ble_stack(void) {
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    shutdown_on_error(err_code);

    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_config_sett(APP_BLE_CONN_CFG_TAG, &ram_start);
    shutdown_on_error(err_code);

    err_code = nrf_sdh_ble_enable(&ram_start);
    shutdown_on_error(err_code);

    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

static void on_adv_evt(ble_adv_evt_t ble_adv_evt) {
    switch (ble_adv_evt) {
        case BLE_ADV_EVT_FAST:
            break;
        case BLE_ADV_EVT_IDLE:
            nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
            break;
        default:
            break;
    }
}

static void init_gap(void) {
    ret_code_t err_code;
    ble_gap_conn_params_t gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    shutdown_on_error(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    shutdown_on_error(err_code);
}

static void init_gatt(void) {
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    shutdown_on_error(err_code);
}

static void init_advertising(void) {
    ret_code_t err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = true;
    init.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids = m_adv_uuids;

    init.config.ble_adv_fast_enabled = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout = ADVERTISING_TIMEOUT_SEC * 100;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    shutdown_on_error(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

static void init_qwr(void) {
    ret_code_t err_code;
    nrf_ble_qwr_init_t qwr_init = {0};

    qwr_init.error_handler = shutdown_on_error;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    shutdown_on_error(err_code);
}

static void on_conn_params_evt(ble_conn_params_evt_t *p_evt) {
    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) {
        ret_code_t err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        shutdown_on_error(err_code);
    }
}

static void conn_params_error_handler(uint32_t nrf_error) {
    shutdown_on_error(nrf_error);
}

static void init_conn_params(void) {
    ret_code_t err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail = false;
    cp_init.evt_handler = on_conn_params_evt;
    cp_init.error_handler = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    shutdown_on_error(err_code);
}

static void advertising_start(void) {
    ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    shutdown_on_error(err_code);

    start_adv_timer();
}

static void init_gpio(void) {
    static app_button_cfg_t buttons[] = {
        {2,  APP_BUTTON_ACTIVE_LOW, BUTTON_PULL, button_handler_callback},
        {3,  APP_BUTTON_ACTIVE_LOW, BUTTON_PULL, button_handler_callback},
        {4,  APP_BUTTON_ACTIVE_LOW, BUTTON_PULL, button_handler_callback},
        {5,  APP_BUTTON_ACTIVE_LOW, BUTTON_PULL, button_handler_callback},
        {6,  APP_BUTTON_ACTIVE_LOW, BUTTON_PULL, button_handler_callback},
        {11, APP_BUTTON_ACTIVE_LOW, BUTTON_PULL, button_handler_callback},
        {12, APP_BUTTON_ACTIVE_LOW, BUTTON_PULL, button_handler_callback},
        {13, APP_BUTTON_ACTIVE_LOW, BUTTON_PULL, button_handler_callback},
        {14, APP_BUTTON_ACTIVE_LOW, BUTTON_PULL, button_handler_callback},
        {15, APP_BUTTON_ACTIVE_LOW, BUTTON_PULL, button_handler_callback},
        {16, APP_BUTTON_ACTIVE_LOW, BUTTON_PULL, button_handler_callback},
        {17, APP_BUTTON_ACTIVE_LOW, BUTTON_PULL, button_handler_callback},
        {18, APP_BUTTON_ACTIVE_LOW, BUTTON_PULL, button_handler_callback},
        {19, APP_BUTTON_ACTIVE_LOW, BUTTON_PULL, button_handler_callback},
        {20, APP_BUTTON_ACTIVE_LOW, BUTTON_PULL, button_handler_callback},
        {21, APP_BUTTON_ACTIVE_LOW, BUTTON_PULL, button_handler_callback},
        {22, APP_BUTTON_ACTIVE_LOW, BUTTON_PULL, button_handler_callback},
        {23, APP_BUTTON_ACTIVE_LOW, BUTTON_PULL, button_handler_callback},
        {24, APP_BUTTON_ACTIVE_LOW, BUTTON_PULL, button_handler_callback},
        {25, APP_BUTTON_ACTIVE_LOW, BUTTON_PULL, button_handler_callback}
    };

    ret_code_t err_code = app_button_init(buttons, ARRAY_SIZE(buttons), BUTTON_DETECTION_DELAY);
    shutdown_on_error(err_code);

    err_code = app_button_enable();
    shutdown_on_error(err_code);

    nrf_gpio_cfg_output(LED_PIN);
    nrf_gpio_pin_write(LED_PIN, LED_OFF);

    // Gestió del botón que despertó el chip
    bool button_pressed = false;
    uint8_t pins_to_check[] = {2, 3, 4, 5, 6, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25};
    for (size_t i = 0; i < sizeof(pins_to_check); i++) {
        if (nrf_gpio_pin_read(pins_to_check[i]) == APP_BUTTON_ACTIVE_LOW) {
            button_pressed = true;
            break;
        }
    }

    button_handler_callback(BUTTON_PIN, button_pressed ? APP_BUTTON_PUSH : APP_BUTTON_RELEASE);
}

static void init_power_management(void) {
    ret_code_t err_code = nrf_pwr_mgmt_init();
    shutdown_on_error(err_code);
}

static void init_logging(void) {
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    shutdown_on_error(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

static void idle_state_handle(void) {
    app_sched_execute();
    if (NRF_LOG_PROCESS() == false) {
        nrf_pwr_mgmt_run();
    }
}

int main(void) {
    init_logging();

    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);

    ret_code_t err_code = app_timer_init();
    shutdown_on_error(err_code);

    err_code = nrf_crypto_init();
    shutdown_on_error(err_code);

    init_power_management();

    init_ble_stack();
    init_gap();
    init_gatt();
    init_services();
    init_advertising();
    init_qwr();
    init_conn_params();

    create_timers();

    init_gpio();

    advertising_start();

    for (;;) {
        idle_state_handle();
    }
}

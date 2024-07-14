/*
 * SPDX-FileCopyrightText: 2016-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "string.h"
#include "esp_log.h"
#include "modbus_params.h" // for modbus parameters structures
#include "mbcontroller.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/dac_oneshot.h"
#include "esp_check.h"

// static void dac_output_task(void *args)
// {
//     dac_oneshot_handle_t handle = (dac_oneshot_handle_t)args;
// }

dac_oneshot_handle_t chan0_handle;
dac_oneshot_config_t chan0_cfg = {
    .chan_id = DAC_CHAN_0,
};
// Here are the user defined instances for device parameters packed by 1 byte
// These are keep the values that can be accessed from Modbus master
holding_reg_params_t holding_reg_params = {0};

#define MB_PORT_NUM (CONFIG_MB_UART_PORT_NUM)   // Number of UART port used for Modbus connection
#define MB_DEV_SPEED (CONFIG_MB_UART_BAUD_RATE) // The communication speed of the UART

// Note: Some pins on target chip cannot be assigned for UART communication.
// See UART documentation for selected board and target to configure pins using Kconfig.

// The number of parameters that intended to be used in the particular control process
#define MASTER_MAX_CIDS num_device_parameters

// Number of reading of parameters from slave
#define MASTER_MAX_RETRY 30

// Timeout to update cid over Modbus
#define UPDATE_CIDS_TIMEOUT_MS (0)
#define UPDATE_CIDS_TIMEOUT_TICS (UPDATE_CIDS_TIMEOUT_MS / portTICK_PERIOD_MS)

// Timeout between polls
#define POLL_TIMEOUT_MS (0)
#define POLL_TIMEOUT_TICS (POLL_TIMEOUT_MS / portTICK_PERIOD_MS)

// The macro to get offset for parameter in the appropriate structure
#define HOLD_OFFSET(field) ((uint16_t)(offsetof(holding_reg_params_t, field) + 1))

#define STR(fieldname) ((const char *)(fieldname))

static const char *TAG = "MASTER_TEST";

// Enumeration of modbus device addresses accessed by master device
enum
{
    MB_DEVICE_ADDR1 = 1 // Only one slave device used for the test (add other slave addresses here)
};

// Enumeration of all supported CIDs for device (used in parameter definition table)
enum
{
    CID_HOLDING_WEIGHT_0
    // ,
    // CID_HOLD_DATA_1
};

const mb_parameter_descriptor_t device_parameters[] = {
    {
        .cid = CID_HOLDING_WEIGHT_0,                /*!< Characteristic cid */
        .param_key = STR("Humidity_1"),             /*!< The key (name) of the parameter */
        .param_units = STR("%rH"),                  /*!< The physical units of the parameter */
        .mb_slave_addr = MB_DEVICE_ADDR1,           /*!< Slave address of device in the Modbus segment */
        .mb_param_type = MB_PARAM_HOLDING,          /*!< Type of modbus parameter */
        .mb_reg_start = 0,                          /*!< This is the Modbus register address. This is the 0 based value. */
        .mb_size = 2,                               /*!< Size of mb parameter in registers */
        .param_offset = HOLD_OFFSET(holding_data0), /*!< Parameter name (OFFSET in the parameter structure) */
        .param_type = PARAM_TYPE_U32,               /*!< Float, U8, U16, U32, ASCII, etc. */
        .param_size = 4,                            /*!< Number of bytes in the parameter. */
        .param_opts = {},                           /*!< Parameter options used to check limits and etc. */
        .access = PAR_PERMS_READ_WRITE_TRIGGER      /*!< Access permissions based on mode */
    }};

// Calculate number of parameters in the table
const uint16_t num_device_parameters = (sizeof(device_parameters) / sizeof(device_parameters[0]));

// The function to get pointer to parameter storage (instance) according to parameter description table
static void *master_get_param_data(const mb_parameter_descriptor_t *param_descriptor)
{
    assert(param_descriptor != NULL);
    void *instance_ptr = NULL;
    if (param_descriptor->param_offset != 0)
    {
        switch (param_descriptor->mb_param_type)
        {
        case MB_PARAM_HOLDING:
            instance_ptr = ((void *)&holding_reg_params + param_descriptor->param_offset - 1);
            break;
        default:
            instance_ptr = NULL;
            break;
        }
    }
    else
    {
        ESP_LOGE(TAG, "Wrong parameter offset for CID #%u", (unsigned)param_descriptor->cid);
        assert(instance_ptr != NULL);
    }
    return instance_ptr;
}

// User operation function to read slave values and check alarm
static void master_operation_func(void *arg)
{

    while (true)
    {

        // Буфер под данные
        int32_t value;
        // Настраиваем параметры запроса
        mb_param_request_t request_cfg = {
            .slave_addr = 0x01,  // Адрес устройства на шине
            .command = 0x03,     // Чтение holding регистра
            .reg_start = 0x0000, // Адрес первого регистра
            .reg_size = 0x0002   // Количество регистров подряд
        };
        // Отправляем запрос
        ESP_ERROR_CHECK(mbc_master_send_request(&request_cfg, &value));
        ESP_LOGI("RS485", "Temperature: %ld", value);
    }
}

// Modbus master initialization
static esp_err_t master_init(void)
{
    // Initialize and start Modbus controller
    mb_communication_info_t comm = {
        .port = MB_PORT_NUM,
#if CONFIG_MB_COMM_MODE_ASCII
        .mode = MB_MODE_ASCII,
#elif CONFIG_MB_COMM_MODE_RTU
        .mode = MB_MODE_RTU,
#endif
        .baudrate = MB_DEV_SPEED,
        .parity = MB_PARITY_NONE};
    void *master_handler = NULL;

    esp_err_t err = mbc_master_init(MB_PORT_SERIAL_MASTER, &master_handler);
    MB_RETURN_ON_FALSE((master_handler != NULL), ESP_ERR_INVALID_STATE, TAG,
                       "mb controller initialization fail.");
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                       "mb controller initialization fail, returns(0x%x).", (int)err);
    err = mbc_master_setup((void *)&comm);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                       "mb controller setup fail, returns(0x%x).", (int)err);

    // Set UART pin numbers
    err = uart_set_pin(MB_PORT_NUM, CONFIG_MB_UART_TXD, CONFIG_MB_UART_RXD,
                       CONFIG_MB_UART_RTS, UART_PIN_NO_CHANGE);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                       "mb serial set pin failure, uart_set_pin() returned (0x%x).", (int)err);

    err = mbc_master_start();
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                       "mb controller start fail, returned (0x%x).", (int)err);

    // Set driver mode to Half Duplex
    err = uart_set_mode(MB_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                       "mb serial set mode failure, uart_set_mode() returned (0x%x).", (int)err);

    vTaskDelay(5);
    err = mbc_master_set_descriptor(&device_parameters[0], num_device_parameters);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG,
                       "mb controller set descriptor fail, returns(0x%x).", (int)err);
    ESP_LOGI(TAG, "Modbus master stack initialized...");
    return err;
}

void app_main(void)
{
    dac_oneshot_new_channel(&chan0_cfg, &chan0_handle);
    uint32_t val = 127;
    ESP_ERROR_CHECK(dac_oneshot_output_voltage(chan0_handle, val));

    // Initialization of device peripheral and objects
    ESP_ERROR_CHECK(master_init());
    vTaskDelay(10);

    master_operation_func(NULL);
}
/*
 * ESP32 ULP SHT3x Application
  *
 * Copyright (C) 2017 KIMATA Tetsuya <kimata@green-rabbit.net>
 *
 * This program is free software ; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, version 2.
 */

#include "nvs_flash.h"
#include "soc/rtc.h"
#include "soc/sens_reg.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "lwip/sockets.h"

#include "esp32/ulp.h"

#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "esp_log.h"

#include "esp_adc_cal.h"
#include "esp_event_loop.h"
#include "esp_spi_flash.h"
#include "esp_task_wdt.h"
#include "esp_wifi.h"

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include "cJSON.h"

#include "ulp_main.h"

#include "wifi_config.h"
// wifi_config.h should define followings.
// #define WIFI_SSID "XXXXXXXX"            // WiFi SSID
// #define WIFI_PASS "XXXXXXXX"            // WiFi Password

////////////////////////////////////////////////////////////
// Configuration
#define FLUENTD_IP      "192.168.2.20"  // IP address of Fluentd
#define FLUENTD_PORT    8888            // Port of FLuentd
#define FLUENTD_TAG     "/sensor"       // Fluentd tag

#define WIFI_HOSTNAME   "ESP32-outdoor2"  // module's hostname
#define SENSE_INTERVAL  30              // sensing interval
#define SENSE_COUNT     20              // buffering count
#define SENSE_COUNT_MAX 30              // buffering count

#define ADC_VREF        1128            // ADC calibration data

////////////////////////////////////////////////////////////
const gpio_num_t gpio_scl    = GPIO_NUM_26;
const gpio_num_t gpio_sda    = GPIO_NUM_25;
const gpio_num_t gpio_bypass = GPIO_NUM_14;

SemaphoreHandle_t wifi_conn_done = NULL;

#define BATTERY_ADC_CH  ADC1_CHANNEL_4  // GPIO 32
#define BATTERY_ADC_SAMPLE  33
#define BATTERY_ADC_DIV  1
/* #define BATTERY_ADC_DIV  2.032 */

/* #define BATTERY_THRESHOLD 2400          // battery threshold (operating voltage of SHT3x) */
#define BATTERY_THRESHOLD 2800          // battery threshold (operating voltage of SHT3x)
////////////////////////////////////////////////////////////

#define WIFI_CONNECT_TIMEOUT 10
#define CLOCK_MEASURE   1024

typedef struct sense_data {
    uint32_t temp;
    uint32_t temp_crc;
    uint32_t humi;
    uint32_t humi_crc;
} sense_data_t;

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

#define TAG "ulp_sht3x"
#define EXPECTED_RESPONSE "HTTP/1.1 200 OK"
#define REQUEST "POST http://" FLUENTD_IP FLUENTD_TAG " HTTP/1.0\r\n" \
    "Content-Type: application/x-www-form-urlencoded\r\n" \
    "Content-Length: %d\r\n" \
    "\r\n" \
    "json=%s"


//////////////////////////////////////////////////////////////////////
// Error Handling
static void _error_check_failed(esp_err_t rc, const char *file, int line,
                                const char *function, const char *expression)
{
    ets_printf("ESP_ERROR_CHECK failed: esp_err_t 0x%x", rc);
#ifdef CONFIG_ESP_ERR_TO_NAME_LOOKUP
    ets_printf(" (%s)", esp_err_to_name(rc));
#endif //CONFIG_ESP_ERR_TO_NAME_LOOKUP
    ets_printf(" at 0x%08x\n", (intptr_t)__builtin_return_address(0) - 3);
    if (spi_flash_cache_enabled()) { // strings may be in flash cache
        ets_printf("file: \"%s\" line %d\nfunc: %s\nexpression: %s\n", file, line, function, expression);
    }
}

#define ERROR_RETURN(x, fail) do {                                      \
        esp_err_t __err_rc = (x);                                       \
        if (__err_rc != ESP_OK) {                                       \
            _error_check_failed(__err_rc, __FILE__, __LINE__,           \
                                __ASSERT_FUNC, #x);                     \
            return fail;                                                \
        }                                                               \
    } while(0);

//////////////////////////////////////////////////////////////////////
// Sensor Function
uint8_t crc8(const uint8_t *data, uint32_t len) {
    static const uint8_t POLY = 0x31;
    uint8_t crc = 0xFF;

    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint32_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ POLY;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static bool sense_value_check_crc(uint32_t data, uint8_t crc) {
    uint8_t buf[2];

    buf[0] = (uint8_t)((data >> 8) & 0xFF);
    buf[1] = (uint8_t)((data >> 0) & 0xFF);

    return crc8(buf, 2) == crc;
}

static bool sense_data_check_crc(sense_data_t *sense_data) {
    return sense_value_check_crc(sense_data->temp, (uint8_t)sense_data->temp_crc) &&
        sense_value_check_crc(sense_data->humi, (uint8_t)sense_data->humi_crc);
}

static float sense_calc_temp(sense_data_t *sense_data) {
    return -45 + (175 * (sense_data->temp & 0xFFFF)) / (float)((1 << 16) - 1);
}

static float sense_calc_humi(sense_data_t *sense_data) {
    return (100 * (sense_data->humi & 0xFFFF)) / (float)((1 << 16) - 1);
}

//////////////////////////////////////////////////////////////////////
// Fluentd Function
int cmp_volt(const uint32_t *a, const uint32_t *b)
{
    if (*a < *b) {
        return -1;
    } else if (*a == *b) {
        return 0;
    } else {
        return 1;
    }
}

uint32_t get_battery_voltage(void)
{
    uint32_t ad_volt_list[BATTERY_ADC_SAMPLE];
    esp_adc_cal_characteristics_t characteristics;

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(BATTERY_ADC_CH, ADC_ATTEN_11db);
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, ADC_VREF,
                             &characteristics);

    for (uint32_t i = 0; i < BATTERY_ADC_SAMPLE; i++) {
        ESP_ERROR_CHECK(esp_adc_cal_get_voltage(BATTERY_ADC_CH,
                                                &characteristics, ad_volt_list + i));
    }

    qsort(ad_volt_list, BATTERY_ADC_SAMPLE, sizeof(uint32_t),
          (int (*)(const void *, const void *))cmp_volt);

    // mean value
    return ad_volt_list[BATTERY_ADC_SAMPLE >> 1] * BATTERY_ADC_DIV;
}

static int connect_server()
{
    struct sockaddr_in server;
    int sock;

    sock = socket(AF_INET, SOCK_STREAM, 0);

    server.sin_family = AF_INET;
    server.sin_port = htons(FLUENTD_PORT);
    server.sin_addr.s_addr = inet_addr(FLUENTD_IP);

    if (connect(sock, (struct sockaddr *)&server, sizeof(server)) != 0) {
        ESP_LOGE(TAG, "FLUENTD CONNECT FAILED errno=%d", errno);
        return -1;
    }
    ESP_LOGI(TAG, "FLUENTD CONNECT SUCCESS");

    return sock;
}

static cJSON *sense_json(uint32_t battery_volt, wifi_ap_record_t *ap_record,
                         uint32_t wifi_con_msec)
{
    sense_data_t *sense_data = (sense_data_t *)&ulp_sense_data;
    cJSON *root = cJSON_CreateArray();

    for (uint32_t i = 0; i < ulp_sense_count; i++) {
        cJSON *item = cJSON_CreateObject();
        uint32_t index = ulp_sense_count - i - 1;

        if (!sense_data_check_crc(sense_data + i)) {
            ESP_LOGE(TAG, "CRC ERROR OCCURED (%d)", i);
            continue;
        }

        cJSON_AddNumberToObject(item, "temp", sense_calc_temp(sense_data + i));
        cJSON_AddNumberToObject(item, "humi", sense_calc_humi(sense_data + i));
        cJSON_AddStringToObject(item, "hostname", WIFI_HOSTNAME);
        cJSON_AddNumberToObject(item, "self_time", SENSE_INTERVAL * index); // negative offset

        if (index == 0) {
            cJSON_AddNumberToObject(item, "battery", battery_volt);
            cJSON_AddNumberToObject(item, "wifi_ch", ap_record->primary);
            cJSON_AddNumberToObject(item, "wifi_rssi", ap_record->rssi);
            cJSON_AddNumberToObject(item, "wifi_con_msec", wifi_con_msec);
            cJSON_AddNumberToObject(item, "retry", ulp_sense_count - SENSE_COUNT);
        }

        cJSON_AddItemToArray(root, item);
    }

    return root;
}

static bool process_sense_data(uint32_t connect_msec, uint32_t battery_volt)
{
    wifi_ap_record_t ap_record;
    char buffer[sizeof(EXPECTED_RESPONSE)];
    bool result = false;

    ERROR_RETURN(esp_wifi_sta_get_ap_info(&ap_record), false);

    int sock = connect_server();
    if (sock == -1) {
        return false;
    }

    cJSON *json = sense_json(battery_volt, &ap_record, connect_msec);
    char *json_str = cJSON_PrintUnformatted(json);

    do {
        if (dprintf(sock, REQUEST, strlen("json=") + strlen(json_str), json_str) < 0) {
            ESP_LOGE(TAG, "FLUENTD POST FAILED");
            break;
        }

        bzero(buffer, sizeof(buffer));
        read(sock, buffer, sizeof(buffer)-1);

        if (strcmp(buffer, EXPECTED_RESPONSE) != 0) {
            ESP_LOGE(TAG, "FLUENTD POST FAILED");
            break;
        }
        ESP_LOGI(TAG, "FLUENTD POST SUCCESSFUL");

        result = true;
    } while (0);

    close(sock);
    cJSON_Delete(json);

    return result;
}

//////////////////////////////////////////////////////////////////////
// Wifi Function
static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_START");
        ERROR_RETURN(tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, WIFI_HOSTNAME), ESP_FAIL);
        ERROR_RETURN(esp_wifi_connect(), ESP_FAIL);
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_GOT_IP");
        xSemaphoreGive(wifi_conn_done);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_DISCONNECTED");
        xSemaphoreGive(wifi_conn_done);
        break;
    default:
        break;
    }
    return ESP_OK;
}

static bool wifi_init()
{
    esp_err_t ret;

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ERROR_RETURN(nvs_flash_erase(), false);
        ret = nvs_flash_init();
    }
    ERROR_RETURN(ret, false);

    tcpip_adapter_init();

    ERROR_RETURN(esp_event_loop_init(event_handler, NULL), false);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ERROR_RETURN(esp_wifi_init(&cfg), false);
    ERROR_RETURN(esp_wifi_set_mode(WIFI_MODE_STA), false);

#ifdef WIFI_SSID
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    wifi_config_t wifi_config_cur;
    ERROR_RETURN(esp_wifi_get_config(WIFI_IF_STA, &wifi_config_cur), false);

    if (strcmp((const char *)wifi_config_cur.sta.ssid, (const char *)wifi_config.sta.ssid) ||
        strcmp((const char *)wifi_config_cur.sta.password, (const char *)wifi_config.sta.password)) {
        ESP_LOGI(TAG, "SAVE WIFI CONFIG");
        ERROR_RETURN(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config), false);
    }
#endif

    return true;
}

static bool wifi_connect()
{
    xSemaphoreTake(wifi_conn_done, portMAX_DELAY);
    ERROR_RETURN(esp_wifi_start(), false);
    if (xSemaphoreTake(wifi_conn_done, 10000 / portTICK_RATE_MS) == pdTRUE) {
        return true;
    } else {
        ESP_LOGE(TAG, "WIFI CONNECT TIMECOUT");
        return false;
    }
}

static bool wifi_stop()
{
    ERROR_RETURN(esp_wifi_disconnect(), false);
    ERROR_RETURN(esp_wifi_stop(), false);

    return true;
}

//////////////////////////////////////////////////////////////////////
// ULP Function
static void init_ulp_program()
{
    ESP_ERROR_CHECK(rtc_gpio_init(gpio_scl));
    ESP_ERROR_CHECK(rtc_gpio_set_direction(gpio_scl, RTC_GPIO_MODE_INPUT_ONLY));
    ESP_ERROR_CHECK(rtc_gpio_init(gpio_sda));
    ESP_ERROR_CHECK(rtc_gpio_set_direction(gpio_sda, RTC_GPIO_MODE_INPUT_ONLY));

    ESP_ERROR_CHECK(rtc_gpio_init(gpio_bypass));
    ESP_ERROR_CHECK(rtc_gpio_set_level(gpio_bypass, 1));
    ESP_ERROR_CHECK(rtc_gpio_pulldown_en(gpio_bypass));
    ESP_ERROR_CHECK(rtc_gpio_pullup_dis(gpio_bypass));
    ESP_ERROR_CHECK(rtc_gpio_set_direction(gpio_bypass, RTC_GPIO_MODE_OUTPUT_ONLY));
    ESP_ERROR_CHECK(rtc_gpio_hold_en(gpio_bypass));

    ESP_ERROR_CHECK(
        ulp_load_binary(
            0, ulp_main_bin_start,
            (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t)
        )
    );
}

void set_sleep_period()
{
    REG_SET_FIELD(SENS_ULP_CP_SLEEP_CYC0_REG, SENS_SLEEP_CYCLES_S0,
                  rtc_time_us_to_slowclk((uint64_t)(SENSE_INTERVAL) * 1e6,
                                         rtc_clk_cal(RTC_CAL_RTC_MUX , CLOCK_MEASURE)));
}

//////////////////////////////////////////////////////////////////////
void app_main()
{
    uint32_t time_start;
    uint32_t battery_volt;
    uint32_t connect_msec;

    vSemaphoreCreateBinary(wifi_conn_done);

    battery_volt = get_battery_voltage();

    esp_log_level_set("wifi", ESP_LOG_ERROR);

    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_ULP) {
        bool status = false;
        ulp_sense_count = ulp_sense_count & 0xFFFF; // mask

        ESP_LOGI(TAG, "Send to fluentd");
        time_start = xTaskGetTickCount();

        if (wifi_init() && wifi_connect()) {
            connect_msec = (xTaskGetTickCount() - time_start) * portTICK_PERIOD_MS;
            status = process_sense_data(connect_msec, battery_volt);
        }
        wifi_stop();

        if (status) {
            ulp_sense_count = 0;
            ulp_sense_full = SENSE_COUNT;
        } else if (ulp_sense_count >= SENSE_COUNT_MAX) {
            ESP_LOGI(TAG, "GIVE UP!");
            // count has reached max
            ulp_sense_count = 0;
            ulp_sense_full = SENSE_COUNT;
        } else {
            ESP_LOGI(TAG, "RETRY");
            ulp_sense_full++;
        }
    } else {
        init_ulp_program();
        ulp_sense_count = 0;
        ulp_sense_full = SENSE_COUNT;
    }

    vTaskDelay(100);

    set_sleep_period();

    // ULP program parameter
    if (battery_volt > BATTERY_THRESHOLD) {
        ESP_LOGI(TAG, "Enable TPS61291 bypass mode");
        ulp_bypass_mode_enable = 1;
    } else {
        ESP_LOGI(TAG, "Disable TPS61291 bypass mode");
        ulp_bypass_mode_enable = 0;
    }

    ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());
    ESP_ERROR_CHECK(ulp_run((&ulp_entry - RTC_SLOW_MEM) / sizeof(uint32_t)));

    ESP_LOGI(TAG, "Go to sleep");
    esp_deep_sleep_start();
}

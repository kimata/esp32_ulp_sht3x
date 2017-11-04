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

#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_task_wdt.h"
#include "esp32/ulp.h"

#include <stdbool.h>
#include <string.h>

#include "cJSON.h"

#include "ulp_main.h"

////////////////////////////////////////////////////////////
// Configuration
#define FLUENTD_IP      "192.168.2.20"  // IP address of Fluentd
#define FLUENTD_PORT    8888            // Port of FLuentd
#define FLUENTD_TAG     "/test"         // Fluentd tag

#define WIFI_HOSTNAME   "sht3x-sensor"  // module's hostname
#define SENSE_INTERVAL  30              // sensing interval
#define SENSE_COUNT     10              // buffering count

/* #define WIFI_SSID "XXXXXXXX" */
/* #define WIFI_PASS "XXXXXXXX" */

////////////////////////////////////////////////////////////

#define WIFI_CONNECT_TIMEOUT 10

typedef enum {
    wifi_connected,
    wifi_disconnected,
} wifi_status_t;

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

const gpio_num_t gpio_scl = GPIO_NUM_26;
const gpio_num_t gpio_sda = GPIO_NUM_25;

#define TAG "ulp_sht3x"
#define EXPECTED_RESPONSE "HTTP/1.1 200 OK"
#define REQUEST "POST http://" FLUENTD_IP FLUENTD_TAG " HTTP/1.0\r\n" \
    "Content-Type: application/x-www-form-urlencoded\r\n" \
    "Content-Length: %d\r\n" \
    "\r\n" \
    "json=%s"

static volatile wifi_status_t witi_status = wifi_disconnected;


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

typedef struct sense_data {
    uint32_t temp;
    uint32_t temp_crc;
    uint32_t humi;
    uint32_t humi_crc;
} sense_data_t;

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_START");
        ESP_ERROR_CHECK(tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, WIFI_HOSTNAME));
        ESP_ERROR_CHECK(esp_wifi_connect());

        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_GOT_IP");
        witi_status = wifi_connected;
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        ESP_ERROR_CHECK(esp_wifi_connect());
        witi_status = wifi_disconnected;
        break;
    default:
        break;
    }
    return ESP_OK;
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

static void init_ulp_program()
{
    rtc_gpio_init(gpio_scl);
    rtc_gpio_set_direction(gpio_scl, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_init(gpio_sda);
    rtc_gpio_set_direction(gpio_sda, RTC_GPIO_MODE_INPUT_ONLY);

    ESP_ERROR_CHECK(
        ulp_load_binary(
            0, ulp_main_bin_start,
            (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t)
        )
    );

    REG_SET_FIELD(SENS_ULP_CP_SLEEP_CYC0_REG, SENS_SLEEP_CYCLES_S0,
        rtc_clk_slow_freq_get_hz()*SENSE_INTERVAL);
}

static void connect_wifi()
{
    esp_err_t ret;

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    tcpip_adapter_init();

    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

#ifdef WIFI_SSID
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
#else
    wifi_config_t wifi_config;
    esp_wifi_get_config(WIFI_IF_STA, &wifi_config);
#endif

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
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

static cJSON *sense_json()
{
    sense_data_t *sense_data = (sense_data_t *)&ulp_sense_data;

    cJSON *root = cJSON_CreateArray();

    for (uint32_t i = 0; i < SENSE_COUNT; i++) {
        if (!sense_data_check_crc(sense_data + i)) {
            ESP_LOGE(TAG, "CRC ERROR OCCURED (%d)", i);
            continue;
        }
        cJSON *item = cJSON_CreateObject();
        cJSON_AddNumberToObject(item, "temp", sense_calc_temp(sense_data + i));
        cJSON_AddNumberToObject(item, "humi", sense_calc_humi(sense_data + i));
        cJSON_AddStringToObject(item, "hostname", WIFI_HOSTNAME);
        cJSON_AddNumberToObject(item, "self_time", SENSE_INTERVAL * i); // negative offset
        cJSON_AddItemToArray(root, item);
    }

    return root;
}

static void process_sense_data()
{
    char buffer[sizeof(EXPECTED_RESPONSE)];
    connect_wifi();

    uint32_t time_start = xTaskGetTickCount();
    while (witi_status != wifi_connected) {
        if ((xTaskGetTickCount() - time_start)> (WIFI_CONNECT_TIMEOUT * 1000 / portTICK_PERIOD_MS)) {
            ESP_LOGE(TAG, "WIFI CONNECT TIMECOUT");
            return;
        }
    }

    int sock = connect_server();
    if (sock == -1) {
        return;
    }

    cJSON *json = sense_json();
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
    } while (0);

    close(sock);
    cJSON_Delete(json);
}

void app_main()
{
    esp_log_level_set("wifi", ESP_LOG_ERROR);

    if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_ULP) {
        init_ulp_program();
    } else {
        process_sense_data();
    }

    ulp_sense_count = SENSE_COUNT;
    ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());
    ESP_ERROR_CHECK(ulp_run((&ulp_entry - RTC_SLOW_MEM) / sizeof(uint32_t)));

    esp_deep_sleep_start();
}

#include <stdint.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdarg.h>
#include <assert.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "board_pins.h"

static const char *TAG = "ear_board";
static const uart_port_t STM32_UART = UART_NUM_1;
static const uart_port_t GPS_UART = UART_NUM_2;
static const uart_port_t ASRPRO_UART = UART_NUM_0;
#define STM32_UART_BAUD_RATE 1000000
#define GPS_UART_BAUD_RATE 9600
#define ASRPRO_UART_BAUD_RATE 9600
#define WIFI_CONNECTED_BIT BIT0
#define LCD_SPI_HOST SPI3_HOST
#define UI_WAVE_X0 0
#define UI_WAVE_Y0 22
#define UI_WAVE_W 240
#define UI_WAVE_H 136
#define UI_VALUE_Y0 160
#define UI_VALUE_H 42
#define UI_MENU_Y0 204
#define UI_SAMPLE_DECIMATION 4
#define UI_FIXED_RANGE 5.0f
#define WEB_SAMPLE_RATE 500U
#define WEB_BATCH_SIZE 10U
#define WEB_PAYLOAD_SIZE 3072U
#define WEB_SAMPLE_FIELDS 12U
#define MAX_WEBSOCKET_CLIENTS 3U
#define HEART_RATE_MIN_INTERVAL_US 300000LL
#define HEART_RATE_MAX_INTERVAL_US 2000000LL
#define HEART_RATE_STALE_US 3000000LL

typedef struct {
    float ch1;
    float ch2;
    float ch3;
    float ch4;
} waveform_sample_t;

typedef struct {
    bool fix;
    double latitude;
    double longitude;
    unsigned long satellites;
    double hdop;
} gps_fix_t;

typedef struct {
    float envelope;
    bool above_threshold;
    int64_t last_peak_us;
    int64_t intervals_us[4];
    uint8_t interval_count;
    uint8_t next_interval;
} heart_rate_estimator_t;

#define WAVEFORM_FIFO_LEN 512U

static waveform_sample_t waveform_fifo[WAVEFORM_FIFO_LEN];
static volatile uint16_t waveform_fifo_head;
static volatile uint16_t waveform_fifo_tail;
static volatile uint32_t waveform_rx_count;
static volatile uint32_t waveform_drop_count;
static volatile float waveform_latest_ch1;
static volatile float waveform_latest_ch2;
static volatile float latest_gyro_x_dps;
static volatile float latest_gyro_y_dps;
static volatile float latest_gyro_z_dps;
static volatile float latest_roll_deg;
static volatile float latest_pitch_deg;
static volatile float latest_yaw_deg;
static volatile float latest_temperature_c;
static volatile uint8_t latest_fall;
static volatile uint16_t latest_heart_rate_bpm;
static volatile int64_t latest_heart_peak_us;
static volatile bool latest_vitals_valid;
static volatile uint8_t ui_menu_index;
static volatile bool ui_pause;
static volatile bool ui_show_ch1 = true;
static volatile bool ui_show_ch2 = true;
static volatile bool ui_clear_request;
static httpd_handle_t web_server;
static int websocket_client_fds[MAX_WEBSOCKET_CLIENTS] = {-1, -1, -1};
static portMUX_TYPE websocket_clients_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile int64_t epoch_offset_us;
static portMUX_TYPE time_sync_mux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE gps_mux = portMUX_INITIALIZER_UNLOCKED;
static EventGroupHandle_t wifi_event_group;
static gps_fix_t latest_gps;
static volatile uint32_t gps_rx_bytes;
static volatile uint32_t gps_sentence_count;
static volatile uint32_t gps_valid_sentence_count;
static float web_batch[WEB_BATCH_SIZE][WEB_SAMPLE_FIELDS];
static uint8_t web_batch_count;
static uint64_t web_sequence;
static uint64_t web_batch_seq_start;
static int64_t web_batch_timestamp_us;
static char web_payload[WEB_PAYLOAD_SIZE];
static heart_rate_estimator_t heart_rate_estimator;

static uint16_t heart_rate_estimator_update(heart_rate_estimator_t *estimator,
                                            float ecg_mv,
                                            int64_t now_us)
{
    const float magnitude = fabsf(ecg_mv);
    const float threshold_floor = 0.03f;
    float threshold;

    if (estimator == NULL) {
        return 0U;
    }

    estimator->envelope = (estimator->envelope * 0.995f) + (magnitude * 0.005f);
    threshold = fmaxf(threshold_floor, estimator->envelope * 3.0f);
    if (!estimator->above_threshold && magnitude >= threshold) {
        const int64_t interval_us = now_us - estimator->last_peak_us;

        estimator->above_threshold = true;
        if (estimator->last_peak_us != 0 &&
            interval_us >= HEART_RATE_MIN_INTERVAL_US &&
            interval_us <= HEART_RATE_MAX_INTERVAL_US) {
            int64_t total_us = 0;

            estimator->intervals_us[estimator->next_interval] = interval_us;
            estimator->next_interval = (uint8_t)((estimator->next_interval + 1U) % 4U);
            if (estimator->interval_count < 4U) {
                estimator->interval_count++;
            }
            for (uint8_t index = 0U; index < estimator->interval_count; ++index) {
                total_us += estimator->intervals_us[index];
            }
            estimator->last_peak_us = now_us;
            return (uint16_t)((60000000LL * estimator->interval_count + (total_us / 2LL)) /
                              total_us);
        }
        if (estimator->last_peak_us == 0 || interval_us > HEART_RATE_MAX_INTERVAL_US) {
            estimator->last_peak_us = now_us;
        }
    } else if (estimator->above_threshold && magnitude < threshold) {
        estimator->above_threshold = false;
    }
    return 0U;
}

static void heart_rate_self_check(void)
{
    heart_rate_estimator_t estimator = {0};
    uint16_t bpm = 0U;

    for (int64_t time_us = 0; time_us <= 6000000LL; time_us += 2000LL) {
        const bool peak = (time_us % 1000000LL) == 0 && time_us != 0;
        bpm = heart_rate_estimator_update(&estimator, peak ? 1.0f : 0.0f, time_us);
    }
    assert(bpm >= 59U && bpm <= 61U);
}

static void waveform_fifo_push(float ch1, float ch2, float ch3, float ch4)
{
    const uint16_t next_head = (uint16_t)((waveform_fifo_head + 1U) % WAVEFORM_FIFO_LEN);

    waveform_latest_ch1 = ch1;
    waveform_latest_ch2 = ch2;
    waveform_rx_count++;

    if (next_head == waveform_fifo_tail) {
        waveform_drop_count++;
        return;
    }

    waveform_fifo[waveform_fifo_head].ch1 = ch1;
    waveform_fifo[waveform_fifo_head].ch2 = ch2;
    waveform_fifo[waveform_fifo_head].ch3 = ch3;
    waveform_fifo[waveform_fifo_head].ch4 = ch4;
    waveform_fifo_head = next_head;
}

static bool waveform_fifo_pop(waveform_sample_t *sample)
{
    if (sample == NULL || waveform_fifo_tail == waveform_fifo_head) {
        return false;
    }

    *sample = waveform_fifo[waveform_fifo_tail];
    waveform_fifo_tail = (uint16_t)((waveform_fifo_tail + 1U) % WAVEFORM_FIFO_LEN);
    return true;
}

static bool parse_waveform_line(const char *line,
                                float *ch1,
                                float *ch2,
                                float *ch3,
                                float *ch4,
                                float *gyro_x,
                                float *gyro_y,
                                float *gyro_z,
                                uint8_t *fall,
                                float *roll,
                                float *pitch,
                                float *yaw,
                                float *temperature)
{
    char *end = NULL;
    const char *cursor = line;
    float values[11];
    unsigned long fall_value = 0;

    if (line == NULL || ch1 == NULL || ch2 == NULL || ch3 == NULL || ch4 == NULL ||
        gyro_x == NULL || gyro_y == NULL || gyro_z == NULL || fall == NULL ||
        roll == NULL || pitch == NULL || yaw == NULL || temperature == NULL) {
        return false;
    }

    for (size_t index = 0; index < 4; ++index) {
        values[index] = strtof(cursor, &end);
        if (end == cursor) {
            return false;
        }
        if (index < 3) {
            if (*end != ',') {
                return false;
            }
            cursor = end + 1;
        }
    }

    *gyro_x = 0.0f;
    *gyro_y = 0.0f;
    *gyro_z = 0.0f;
    *fall = 0U;
    *roll = 0.0f;
    *pitch = 0.0f;
    *yaw = 0.0f;
    *temperature = 0.0f;
    if (*end == ',') {
        cursor = end + 1;
        for (size_t index = 4; index < 7; ++index) {
            values[index] = strtof(cursor, &end);
            if (end == cursor || *end != ',') {
                return false;
            }
            cursor = end + 1;
        }
        fall_value = strtoul(cursor, &end, 10);
        *gyro_x = values[4];
        *gyro_y = values[5];
        *gyro_z = values[6];
        *fall = (fall_value != 0U) ? 1U : 0U;
        if (*end == ',') {
            cursor = end + 1;
            for (size_t index = 7; index < 11; ++index) {
                values[index] = strtof(cursor, &end);
                if (end == cursor) {
                    return false;
                }
                if (index < 10) {
                    if (*end != ',') {
                        return false;
                    }
                    cursor = end + 1;
                }
            }
            *roll = values[7];
            *pitch = values[8];
            *yaw = values[9];
            *temperature = values[10];
        }
    }

    while (*end == ' ') {
        end++;
    }
    if (*end != '\0') {
        return false;
    }

    *ch1 = values[0];
    *ch2 = values[1];
    *ch3 = values[2];
    *ch4 = values[3];
    return true;
}

static bool is_vofa_sample_line(const char *line)
{
    bool has_digit = false;

    if (line == NULL || line[0] == '\0') {
        return false;
    }

    for (const char *p = line; *p != '\0'; ++p) {
        const char c = *p;

        if (c >= '0' && c <= '9') {
            has_digit = true;
            continue;
        }
        if (c == '-' || c == '+' || c == '.' || c == ',' || c == ' ') {
            continue;
        }
        return false;
    }

    return has_digit;
}

static int hex_nibble(char value)
{
    if (value >= '0' && value <= '9') {
        return value - '0';
    }
    if (value >= 'A' && value <= 'F') {
        return value - 'A' + 10;
    }
    if (value >= 'a' && value <= 'f') {
        return value - 'a' + 10;
    }
    return -1;
}

static bool nmea_checksum_valid(const char *line)
{
    const char *checksum_text;
    uint8_t checksum = 0U;
    int high;
    int low;

    if (line == NULL || line[0] != '$') {
        return false;
    }

    checksum_text = strchr(line, '*');
    if (checksum_text == NULL || checksum_text[1] == '\0' ||
        checksum_text[2] == '\0' || checksum_text[3] != '\0') {
        return false;
    }

    high = hex_nibble(checksum_text[1]);
    low = hex_nibble(checksum_text[2]);
    if (high < 0 || low < 0) {
        return false;
    }

    for (const char *cursor = line + 1; cursor < checksum_text; ++cursor) {
        checksum ^= (uint8_t)*cursor;
    }

    return checksum == (uint8_t)((high << 4) | low);
}

static bool nmea_coordinate(double raw,
                            char hemisphere,
                            bool latitude,
                            double *coordinate)
{
    const int degrees = (int)(raw / 100.0);
    const double minutes = raw - (double)degrees * 100.0;
    const int maximum_degrees = latitude ? 90 : 180;

    if (coordinate == NULL || raw < 0.0 || minutes < 0.0 || minutes >= 60.0 ||
        degrees > maximum_degrees ||
        (degrees == maximum_degrees && minutes > 0.0)) {
        return false;
    }
    if ((latitude && hemisphere != 'N' && hemisphere != 'S') ||
        (!latitude && hemisphere != 'E' && hemisphere != 'W')) {
        return false;
    }

    *coordinate = (double)degrees + minutes / 60.0;
    if (hemisphere == 'S' || hemisphere == 'W') {
        *coordinate = -*coordinate;
    }
    return true;
}

static bool parse_gga_line(const char *line, gps_fix_t *fix)
{
    char copy[128];
    char *fields[9];
    char *checksum_text;
    char *end;
    unsigned long quality;
    double raw_latitude;
    double raw_longitude;

    if (fix == NULL || !nmea_checksum_valid(line) || strlen(line) >= sizeof(copy)) {
        return false;
    }

    strcpy(copy, line);
    checksum_text = strchr(copy, '*');
    *checksum_text = '\0';
    fields[0] = copy;
    for (size_t index = 1; index < 9; ++index) {
        fields[index] = strchr(fields[index - 1], ',');
        if (fields[index] == NULL) {
            return false;
        }
        *fields[index] = '\0';
        fields[index]++;
    }
    if (strlen(fields[0]) != 6U || strcmp(fields[0] + 3, "GGA") != 0) {
        return false;
    }

    quality = strtoul(fields[6], &end, 10);
    if (end == fields[6] || *end != '\0') {
        return false;
    }

    fix->fix = quality != 0U;
    fix->latitude = 0.0;
    fix->longitude = 0.0;
    fix->satellites = strtoul(fields[7], &end, 10);
    if (*end != '\0') {
        return false;
    }
    end = strchr(fields[8], ',');
    if (end == NULL) {
        return false;
    }
    *end = '\0';
    fix->hdop = strtod(fields[8], &end);
    if (end == fields[8] || *end != '\0') {
        return false;
    }
    if (!fix->fix) {
        return true;
    }

    raw_latitude = strtod(fields[2], &end);
    if (end == fields[2] || *end != '\0' || strlen(fields[3]) != 1U ||
        !nmea_coordinate(raw_latitude, fields[3][0], true, &fix->latitude)) {
        return false;
    }

    raw_longitude = strtod(fields[4], &end);
    if (end == fields[4] || *end != '\0' || strlen(fields[5]) != 1U ||
        !nmea_coordinate(raw_longitude, fields[5][0], false, &fix->longitude)) {
        return false;
    }

    return true;
}

static void gps_parser_self_check(void)
{
    static const char sample[] =
        "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
    gps_fix_t fix;

    ESP_ERROR_CHECK(parse_gga_line(sample, &fix) &&
                    fix.fix &&
                    fabs(fix.latitude - 48.1173) < 0.000001 &&
                    fabs(fix.longitude - 11.5166666667) < 0.000001
                        ? ESP_OK
                        : ESP_FAIL);
}

static void process_gps_line(const char *line)
{
    gps_fix_t fix;

    gps_sentence_count++;
    if (!parse_gga_line(line, &fix)) {
        return;
    }

    gps_valid_sentence_count++;
    portENTER_CRITICAL(&gps_mux);
    latest_gps = fix;
    portEXIT_CRITICAL(&gps_mux);
}

static void gps_uart_task(void *arg)
{
    uint8_t data[64];
    char line[128];
    size_t line_length = 0U;

    (void)arg;

    while (true) {
        const int length = uart_read_bytes(GPS_UART, data, sizeof(data),
                                           pdMS_TO_TICKS(200));
        if (length > 0) {
            gps_rx_bytes += (uint32_t)length;
        }

        for (int index = 0; index < length; ++index) {
            if (data[index] == '\n') {
                if (line_length > 0U && line[line_length - 1U] == '\r') {
                    line_length--;
                }
                line[line_length] = '\0';
                process_gps_line(line);
                line_length = 0U;
            } else if (line_length < sizeof(line) - 1U) {
                line[line_length++] = (char)data[index];
            } else {
                line_length = 0U;
            }
        }
    }
}

static void init_gps_uart(void)
{
    const uart_config_t config = {
        .baud_rate = GPS_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    gps_parser_self_check();
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART, 1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(GPS_UART, &config));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART,
                                 BOARD_GPS_UART_TX_GPIO,
                                 BOARD_GPS_UART_RX_GPIO,
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));
}

static int64_t web_timestamp_us(void)
{
    int64_t offset;

    portENTER_CRITICAL(&time_sync_mux);
    offset = epoch_offset_us;
    portEXIT_CRITICAL(&time_sync_mux);
    return esp_timer_get_time() + offset;
}

static void apply_time_sync(const char *payload)
{
    const char *field;
    const char *colon;
    uint64_t timestamp_us;

    if (payload == NULL || strstr(payload, "\"type\":\"timeSync\"") == NULL) {
        return;
    }

    field = strstr(payload, "\"timestampUs\"");
    colon = field != NULL ? strchr(field, ':') : NULL;
    if (colon == NULL) {
        return;
    }

    timestamp_us = strtoull(colon + 1, NULL, 10);
    if (timestamp_us > 1000000000000000ULL) {
        portENTER_CRITICAL(&time_sync_mux);
        epoch_offset_us = (int64_t)timestamp_us - esp_timer_get_time();
        portEXIT_CRITICAL(&time_sync_mux);
    }
}

static void apply_phone_location(const char *payload)
{
    const char *latitude_field;
    const char *longitude_field;
    char *end;
    double latitude;
    double longitude;

    if (payload == NULL || strstr(payload, "\"type\":\"phoneLocation\"") == NULL) {
        return;
    }
    latitude_field = strstr(payload, "\"latitude\"");
    longitude_field = strstr(payload, "\"longitude\"");
    latitude_field = latitude_field != NULL ? strchr(latitude_field, ':') : NULL;
    longitude_field = longitude_field != NULL ? strchr(longitude_field, ':') : NULL;
    if (latitude_field == NULL || longitude_field == NULL) {
        return;
    }
    latitude = strtod(latitude_field + 1, &end);
    if (end == latitude_field + 1) {
        return;
    }
    longitude = strtod(longitude_field + 1, &end);
    if (end == longitude_field + 1 || !isfinite(latitude) || !isfinite(longitude) ||
        latitude < -90.0 || latitude > 90.0 || longitude < -180.0 || longitude > 180.0) {
        return;
    }
    portENTER_CRITICAL(&gps_mux);
    latest_gps.fix = true;
    latest_gps.latitude = latitude;
    latest_gps.longitude = longitude;
    latest_gps.satellites = 0U;
    latest_gps.hdop = 0.0;
    portEXIT_CRITICAL(&gps_mux);
}

static void websocket_client_add(int client_fd)
{
    portENTER_CRITICAL(&websocket_clients_mux);
    for (size_t index = 0U; index < MAX_WEBSOCKET_CLIENTS; ++index) {
        if (websocket_client_fds[index] == client_fd) {
            portEXIT_CRITICAL(&websocket_clients_mux);
            return;
        }
    }
    for (size_t index = 0U; index < MAX_WEBSOCKET_CLIENTS; ++index) {
        if (websocket_client_fds[index] < 0) {
            websocket_client_fds[index] = client_fd;
            break;
        }
    }
    portEXIT_CRITICAL(&websocket_clients_mux);
}

static void websocket_client_remove(int client_fd)
{
    portENTER_CRITICAL(&websocket_clients_mux);
    for (size_t index = 0U; index < MAX_WEBSOCKET_CLIENTS; ++index) {
        if (websocket_client_fds[index] == client_fd) {
            websocket_client_fds[index] = -1;
        }
    }
    portEXIT_CRITICAL(&websocket_clients_mux);
}

static size_t websocket_client_snapshot(int *client_fds)
{
    size_t count = 0U;

    portENTER_CRITICAL(&websocket_clients_mux);
    for (size_t index = 0U; index < MAX_WEBSOCKET_CLIENTS; ++index) {
        if (websocket_client_fds[index] >= 0) {
            client_fds[count++] = websocket_client_fds[index];
        }
    }
    portEXIT_CRITICAL(&websocket_clients_mux);
    return count;
}

static esp_err_t websocket_handler(httpd_req_t *req)
{
    httpd_ws_frame_t frame = {0};
    char payload[160] = {0};
    esp_err_t status;
    const int client_fd = httpd_req_to_sockfd(req);

    if (req->method == HTTP_GET) {
        websocket_client_add(client_fd);
        return ESP_OK;
    }

    frame.payload = (uint8_t *)payload;
    status = httpd_ws_recv_frame(req, &frame, sizeof(payload) - 1U);
    if (status != ESP_OK) {
        websocket_client_remove(client_fd);
        return status;
    }

    if (frame.type == HTTPD_WS_TYPE_CLOSE) {
        websocket_client_remove(client_fd);
        return ESP_OK;
    }

    if (frame.type == HTTPD_WS_TYPE_TEXT) {
        payload[frame.len] = '\0';
        apply_time_sync(payload);
        apply_phone_location(payload);
    }

    return ESP_OK;
}

static void start_websocket_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    const httpd_uri_t websocket_uri = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = websocket_handler,
        .user_ctx = NULL,
        .is_websocket = true,
        .handle_ws_control_frames = true,
    };

    config.max_open_sockets = 4;
    config.lru_purge_enable = true;
    config.stack_size = 6144;

    ESP_ERROR_CHECK(httpd_start(&web_server, &config));
    ESP_ERROR_CHECK(httpd_register_uri_handler(web_server, &websocket_uri));
}

static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    (void)arg;
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        (void)esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        (void)esp_wifi_connect();
        printf("WIFI,DISCONNECTED\r\n");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *event = (const ip_event_got_ip_t *)event_data;
        printf("ESP32_IP," IPSTR "\r\n", IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void init_wifi_sta(void)
{
    wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = BOARD_WIFI_STA_SSID,
            .password = BOARD_WIFI_STA_PASSWORD,
            .threshold = { .authmode = WIFI_AUTH_WPA2_PSK },
        },
    };
    esp_err_t status = nvs_flash_init();

    if (status == ESP_ERR_NVS_NO_FREE_PAGES ||
        status == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        status = nvs_flash_init();
    }
    ESP_ERROR_CHECK(status);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(wifi_event_group == NULL ? ESP_ERR_NO_MEM : ESP_OK);
    esp_netif_create_default_wifi_sta();
    ESP_ERROR_CHECK(esp_wifi_init(&init_config));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
                                               ESP_EVENT_ANY_ID,
                                               wifi_event_handler,
                                               NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT,
                                               IP_EVENT_STA_GOT_IP,
                                               wifi_event_handler,
                                               NULL));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    printf("WIFI,STA_CONNECTING,%s\r\n", BOARD_WIFI_STA_SSID);
}

static bool append_web_payload(size_t *used, const char *format, ...)
{
    va_list args;
    int written;

    if (used == NULL || *used >= sizeof(web_payload)) {
        return false;
    }

    va_start(args, format);
    written = vsnprintf(web_payload + *used,
                        sizeof(web_payload) - *used,
                        format,
                        args);
    va_end(args);

    if (written < 0 || (size_t)written >= (sizeof(web_payload) - *used)) {
        return false;
    }

    *used += (size_t)written;
    return true;
}

static void websocket_send_batch(void)
{
    httpd_ws_frame_t frame = {0};
    int client_fds[MAX_WEBSOCKET_CLIENTS];
    size_t client_count;
    gps_fix_t gps;
    size_t used = 0;

    if (web_server == NULL) {
        return;
    }
    client_count = websocket_client_snapshot(client_fds);
    if (client_count == 0U) {
        return;
    }

    portENTER_CRITICAL(&gps_mux);
    gps = latest_gps;
    portEXIT_CRITICAL(&gps_mux);

    if (!append_web_payload(&used,
                            "{\"type\":\"samples\",\"version\":1,"
                            "\"deviceId\":\"bioscope-esp32-01\","
                            "\"sampleRate\":%u,\"seqStart\":%" PRIu64 ","
                            "\"timestampUs\":%" PRId64 ",\"samples\":[",
                            WEB_SAMPLE_RATE,
                            web_batch_seq_start,
                            web_batch_timestamp_us)) {
        return;
    }

    for (uint8_t index = 0; index < web_batch_count; ++index) {
        if (!append_web_payload(&used,
                                "%s[%.5f,%.5f,%.5f,%.5f,%.2f,%.2f,%.2f,%.0f,%.2f,%.2f,%.2f,%.2f]",
                                index == 0U ? "" : ",",
                                (double)web_batch[index][0],
                                (double)web_batch[index][1],
                                (double)web_batch[index][2],
                                (double)web_batch[index][3],
                                (double)web_batch[index][4],
                                (double)web_batch[index][5],
                                (double)web_batch[index][6],
                                (double)web_batch[index][7],
                                (double)web_batch[index][8],
                                (double)web_batch[index][9],
                                (double)web_batch[index][10],
                                (double)web_batch[index][11])) {
            return;
        }
    }

    if (gps.fix) {
        if (!append_web_payload(&used,
                                "],\"gps\":{\"fix\":true,"
                                "\"latitude\":%.6f,\"longitude\":%.6f,"
                                "\"satellites\":%lu,\"hdop\":%.2f,"
                                "\"rxBytes\":%lu,\"sentences\":%lu,"
                                "\"validSentences\":%lu}}",
                                gps.latitude,
                                gps.longitude,
                                gps.satellites,
                                gps.hdop,
                                (unsigned long)gps_rx_bytes,
                                (unsigned long)gps_sentence_count,
                                (unsigned long)gps_valid_sentence_count)) {
            return;
        }
    } else if (!append_web_payload(&used,
                                   "],\"gps\":{\"fix\":false,"
                                   "\"satellites\":%lu,\"hdop\":%.2f,"
                                   "\"rxBytes\":%lu,\"sentences\":%lu,"
                                   "\"validSentences\":%lu}}",
                                   gps.satellites,
                                   gps.hdop,
                                   (unsigned long)gps_rx_bytes,
                                   (unsigned long)gps_sentence_count,
                                   (unsigned long)gps_valid_sentence_count)) {
        return;
    }

    frame.type = HTTPD_WS_TYPE_TEXT;
    frame.payload = (uint8_t *)web_payload;
    frame.len = used;
    for (size_t index = 0U; index < client_count; ++index) {
        if (httpd_ws_send_frame_async(web_server, client_fds[index], &frame) != ESP_OK) {
            websocket_client_remove(client_fds[index]);
        }
    }
}

static void websocket_batch_push(float ch1,
                                 float ch2,
                                 float ch3,
                                 float ch4,
                                 float gyro_x,
                                 float gyro_y,
                                 float gyro_z,
                                 uint8_t fall,
                                 float roll,
                                 float pitch,
                                 float yaw,
                                 float temperature)
{
    if (web_batch_count == 0U) {
        web_batch_timestamp_us = web_timestamp_us();
        web_batch_seq_start = web_sequence;
    }

    web_batch[web_batch_count][0] = ch1;
    web_batch[web_batch_count][1] = ch2;
    web_batch[web_batch_count][2] = ch3;
    web_batch[web_batch_count][3] = ch4;
    web_batch[web_batch_count][4] = gyro_x;
    web_batch[web_batch_count][5] = gyro_y;
    web_batch[web_batch_count][6] = gyro_z;
    web_batch[web_batch_count][7] = fall ? 1.0f : 0.0f;
    web_batch[web_batch_count][8] = roll;
    web_batch[web_batch_count][9] = pitch;
    web_batch[web_batch_count][10] = yaw;
    web_batch[web_batch_count][11] = temperature;
    web_batch_count++;
    web_sequence++;

    if (web_batch_count == WEB_BATCH_SIZE) {
        websocket_send_batch();
        web_batch_count = 0U;
    }
}

static void websocket_flush_batch(void)
{
    if (web_batch_count > 0U) {
        websocket_send_batch();
        web_batch_count = 0U;
    }
}

#if BOARD_ENABLE_WEB_TEST_SIGNAL
static void web_test_signal_task(void *arg)
{
    const float two_pi = 6.28318530718f;
    TickType_t wake_time = xTaskGetTickCount();
    uint32_t sample_index = 0U;

    (void)arg;

    while (true) {
        for (uint8_t index = 0U; index < WEB_BATCH_SIZE; ++index) {
            const float time_s = (float)sample_index / (float)WEB_SAMPLE_RATE;
            const uint32_t ecg_phase = sample_index % 417U;
            float ecg = 0.12f * sinf(two_pi * 1.2f * time_s);
            const float emg1 = 0.22f * sinf(two_pi * 18.0f * time_s) +
                               0.05f * sinf(two_pi * 53.0f * time_s);
            const float emg2 = 0.18f * sinf(two_pi * 27.0f * time_s);
            const float emg3 = 0.14f * sinf(two_pi * 36.0f * time_s) +
                               0.03f * sinf(two_pi * 71.0f * time_s);

            if (ecg_phase >= 4U && ecg_phase < 7U) {
                ecg -= 0.45f;
            } else if (ecg_phase >= 7U && ecg_phase < 12U) {
                ecg += 1.45f;
            } else if (ecg_phase >= 12U && ecg_phase < 16U) {
                ecg -= 0.60f;
            }

            waveform_fifo_push(ecg, emg1, emg2, emg3);
            websocket_batch_push(ecg, emg1, emg2, emg3,
                                 0.0f, 0.0f, 0.0f, 0U,
                                 0.0f, 0.0f, 0.0f, 0.0f);
            sample_index++;
        }

        vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(20));
    }
}
#endif

static void stm32_uart_send(const char *text)
{
    uart_write_bytes(STM32_UART, text, strlen(text));
}

static void init_stm32_uart(void)
{
    const uart_config_t config = {
        .baud_rate = STM32_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(STM32_UART, 4096, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(STM32_UART, &config));
    ESP_ERROR_CHECK(uart_set_pin(STM32_UART,
                                 BOARD_STM32_UART_TX_GPIO,
                                 BOARD_STM32_UART_RX_GPIO,
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));

    stm32_uart_send("ESP32,HELLO\r\n");
}

static void asrpro_uart_send(const char *text)
{
    if (text != NULL) {
        (void)uart_write_bytes(ASRPRO_UART, text, strlen(text));
    }
}

static void asrpro_send_vitals(const char *request)
{
    char response[48];
    const int64_t now_us = esp_timer_get_time();
    const bool heart_valid = latest_heart_rate_bpm != 0U &&
                             (now_us - latest_heart_peak_us) <= HEART_RATE_STALE_US;
    const long temperature_hundredths = lroundf(latest_temperature_c * 100.0f);
    gps_fix_t gps;

    if (strcmp(request, "ASR,GET,COORD") == 0) {
        portENTER_CRITICAL(&gps_mux);
        gps = latest_gps;
        portEXIT_CRITICAL(&gps_mux);

        if (!gps.fix) {
            asrpro_uart_send("ASR,DATA,COORD,NONE\n");
        } else {
            (void)snprintf(response, sizeof(response), "ASR,DATA,COORD,%ld,%ld\n",
                           lround(gps.latitude * 1000000.0),
                           lround(gps.longitude * 1000000.0));
            asrpro_uart_send(response);
        }
        return;
    }

    if (!latest_vitals_valid) {
        asrpro_uart_send("ASR,DATA,NONE\n");
    } else if (strcmp(request, "ASR,GET,TEMP") == 0) {
        (void)snprintf(response, sizeof(response), "ASR,DATA,TEMP,%ld\n", temperature_hundredths);
        asrpro_uart_send(response);
    } else if (strcmp(request, "ASR,GET,HEART") == 0) {
        (void)snprintf(response, sizeof(response), "ASR,DATA,HEART,%lu\n",
                       heart_valid ? (unsigned long)latest_heart_rate_bpm * 100UL : 0UL);
        asrpro_uart_send(response);
    } else if (strcmp(request, "ASR,GET,ALL") == 0) {
        (void)snprintf(response, sizeof(response), "ASR,DATA,ALL,%ld,%lu\n",
                       temperature_hundredths,
                       heart_valid ? (unsigned long)latest_heart_rate_bpm * 100UL : 0UL);
        asrpro_uart_send(response);
    }
}

static void asrpro_uart_task(void *arg)
{
    uint8_t data[32];
    char line[48];
    size_t line_length = 0U;

    (void)arg;
    while (true) {
        const int length = uart_read_bytes(ASRPRO_UART, data, sizeof(data),
                                           pdMS_TO_TICKS(100));

        for (int index = 0; index < length; ++index) {
            if (data[index] == '\n') {
                if (line_length > 0U && line[line_length - 1U] == '\r') {
                    line_length--;
                }
                line[line_length] = '\0';
                asrpro_send_vitals(line);
                line_length = 0U;
            } else if (line_length < sizeof(line) - 1U) {
                line[line_length++] = (char)data[index];
            } else {
                line_length = 0U;
            }
        }
    }
}

static void init_asrpro_uart(void)
{
    const uart_config_t config = {
        .baud_rate = ASRPRO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(ASRPRO_UART, 256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(ASRPRO_UART, &config));
    ESP_ERROR_CHECK(uart_set_pin(ASRPRO_UART,
                                 BOARD_ASRPRO_UART_TX_GPIO,
                                 BOARD_ASRPRO_UART_RX_GPIO,
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));
}

static void process_stm32_line(const char *line)
{
    static const char ping_prefix[] = "STM32,PING,";
    char response[64];
    unsigned long sequence;
    float ch1;
    float ch2;
    float ch3;
    float ch4;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float roll;
    float pitch;
    float yaw;
    float temperature;
    uint8_t fall;

    if (is_vofa_sample_line(line)) {
        if (parse_waveform_line(line,
                                &ch1,
                                &ch2,
                                &ch3,
                                &ch4,
                                &gyro_x,
                                &gyro_y,
                                &gyro_z,
                                &fall,
                                &roll,
                                &pitch,
                                &yaw,
                                &temperature)) {
            latest_gyro_x_dps = gyro_x;
            latest_gyro_y_dps = gyro_y;
            latest_gyro_z_dps = gyro_z;
            latest_roll_deg = roll;
            latest_pitch_deg = pitch;
            latest_yaw_deg = yaw;
            latest_temperature_c = temperature;
            latest_fall = fall;
            latest_vitals_valid = true;
            const uint16_t bpm = heart_rate_estimator_update(&heart_rate_estimator,
                                                              ch1,
                                                              esp_timer_get_time());
            latest_heart_peak_us = heart_rate_estimator.last_peak_us;
            if (bpm != 0U) {
                latest_heart_rate_bpm = bpm;
            }
            waveform_fifo_push(ch1, ch2, ch3, ch4);
            websocket_batch_push(ch1, ch2, ch3, ch4,
                                 gyro_x, gyro_y, gyro_z, fall,
                                 roll, pitch, yaw, temperature);
            if (ch1 == 0.0f && ch2 == 0.0f && ch3 == 0.0f && ch4 == 0.0f) {
                websocket_flush_batch();
            }
        }
        return;
    }

    if (strcmp(line, "STM32,HELLO") == 0) {
        stm32_uart_send("ESP32,READY\r\n");
        return;
    }

    if (strncmp(line, ping_prefix, sizeof(ping_prefix) - 1) == 0) {
        sequence = strtoul(line + sizeof(ping_prefix) - 1, NULL, 10);
        snprintf(response, sizeof(response), "ESP32,PONG,%lu\r\n", sequence);
        stm32_uart_send(response);
        return;
    }

    if (strncmp(line, "STM32,ACK,", 10) == 0) {
        sequence = strtoul(line + 10, NULL, 10);
        ESP_LOGI(TAG, "UART round trip confirmed, sequence=%lu", sequence);
    }
}

static void __attribute__((unused)) stm32_uart_task(void *arg)
{
    uint8_t data[64];
    char line[128];
    size_t line_length = 0;

    (void)arg;

    while (true) {
        const int length = uart_read_bytes(STM32_UART, data, sizeof(data),
                                           pdMS_TO_TICKS(10));

        for (int i = 0; i < length; ++i) {
            if (data[i] == '\n') {
                if (line_length > 0 && line[line_length - 1] == '\r') {
                    line_length--;
                }
                line[line_length] = '\0';
                process_stm32_line(line);
                line_length = 0;
            } else if (line_length < sizeof(line) - 1) {
                line[line_length++] = (char)data[i];
            } else {
                line_length = 0;
            }
        }
    }
}

#if BOARD_ENABLE_LCD
static void init_lcd_backlight(void)
{
    const gpio_config_t config = {
        .pin_bit_mask = (1ULL << BOARD_LCD_BL_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&config));
    gpio_set_level(BOARD_LCD_BL_GPIO, 1);
}

static esp_lcd_panel_handle_t init_lcd(void)
{
    init_lcd_backlight();

    const spi_bus_config_t bus_config = {
        .sclk_io_num = BOARD_LCD_SCLK_GPIO,
        .mosi_io_num = BOARD_LCD_MOSI_GPIO,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = BOARD_LCD_WIDTH * 40 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_SPI_HOST, &bus_config, SPI_DMA_CH_AUTO));

    const esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = BOARD_LCD_DC_GPIO,
        .cs_gpio_num = BOARD_LCD_CS_GPIO,
        .pclk_hz = 80 * 1000 * 1000,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 2,
        .trans_queue_depth = 10,
        .flags = {
            .sio_mode = true,
        },
    };
    esp_lcd_panel_io_handle_t io = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_SPI_HOST,
                                              &io_config, &io));

    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = BOARD_LCD_RST_GPIO,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    esp_lcd_panel_handle_t panel = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io, &panel_config, &panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel, BOARD_LCD_X_GAP, BOARD_LCD_Y_GAP));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));
    return panel;
}

static void lcd_fill_rect(esp_lcd_panel_handle_t panel,
                          int x_start,
                          int y_start,
                          int x_end,
                          int y_end,
                          uint16_t color)
{
    static uint16_t line[BOARD_LCD_WIDTH * 20];
    const int width = x_end - x_start;
    int y = y_start;

    if (width <= 0 || x_start < 0 || x_end > BOARD_LCD_WIDTH) {
        return;
    }

    for (int i = 0; i < (width * 20); ++i) {
        line[i] = color;
    }

    while (y < y_end) {
        int next_y = y + 20;
        if (next_y > y_end) {
            next_y = y_end;
        }
        ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel, x_start, y, x_end, next_y,
                                                   line));
        y = next_y;
    }
}

static void lcd_fill_screen(esp_lcd_panel_handle_t panel, uint16_t color)
{
    lcd_fill_rect(panel, 0, 0, BOARD_LCD_WIDTH, BOARD_LCD_HEIGHT, color);
}

static void lcd_draw_pixel(esp_lcd_panel_handle_t panel, int x, int y, uint16_t color)
{
    if (x < 0 || x >= BOARD_LCD_WIDTH || y < 0 || y >= BOARD_LCD_HEIGHT) {
        return;
    }

    lcd_fill_rect(panel, x, y, x + 1, y + 1, color);
}

static void lcd_draw_line(esp_lcd_panel_handle_t panel,
                          int x0,
                          int y0,
                          int x1,
                          int y1,
                          uint16_t color)
{
    int dx = abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    while (true) {
        lcd_draw_pixel(panel, x0, y0, color);
        if (x0 == x1 && y0 == y1) {
            break;
        }
        const int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

static void font5x7_get(char c, uint8_t out[5])
{
    static const uint8_t blank[5] = {0, 0, 0, 0, 0};
    const uint8_t *g = blank;

    static const uint8_t n0[5] = {0x3E, 0x51, 0x49, 0x45, 0x3E};
    static const uint8_t n1[5] = {0x00, 0x42, 0x7F, 0x40, 0x00};
    static const uint8_t n2[5] = {0x42, 0x61, 0x51, 0x49, 0x46};
    static const uint8_t n3[5] = {0x21, 0x41, 0x45, 0x4B, 0x31};
    static const uint8_t n4[5] = {0x18, 0x14, 0x12, 0x7F, 0x10};
    static const uint8_t n5[5] = {0x27, 0x45, 0x45, 0x45, 0x39};
    static const uint8_t n6[5] = {0x3C, 0x4A, 0x49, 0x49, 0x30};
    static const uint8_t n7[5] = {0x01, 0x71, 0x09, 0x05, 0x03};
    static const uint8_t n8[5] = {0x36, 0x49, 0x49, 0x49, 0x36};
    static const uint8_t n9[5] = {0x06, 0x49, 0x49, 0x29, 0x1E};
    static const uint8_t A[5] = {0x7E, 0x11, 0x11, 0x11, 0x7E};
    static const uint8_t C[5] = {0x3E, 0x41, 0x41, 0x41, 0x22};
    static const uint8_t D[5] = {0x7F, 0x41, 0x41, 0x22, 0x1C};
    static const uint8_t E[5] = {0x7F, 0x49, 0x49, 0x49, 0x41};
    static const uint8_t F[5] = {0x7F, 0x09, 0x09, 0x09, 0x01};
    static const uint8_t G[5] = {0x3E, 0x41, 0x49, 0x49, 0x7A};
    static const uint8_t H[5] = {0x7F, 0x08, 0x08, 0x08, 0x7F};
    static const uint8_t I[5] = {0x00, 0x41, 0x7F, 0x41, 0x00};
    static const uint8_t L[5] = {0x7F, 0x40, 0x40, 0x40, 0x40};
    static const uint8_t M[5] = {0x7F, 0x02, 0x0C, 0x02, 0x7F};
    static const uint8_t N[5] = {0x7F, 0x04, 0x08, 0x10, 0x7F};
    static const uint8_t O[5] = {0x3E, 0x41, 0x41, 0x41, 0x3E};
    static const uint8_t P[5] = {0x7F, 0x09, 0x09, 0x09, 0x06};
    static const uint8_t R[5] = {0x7F, 0x09, 0x19, 0x29, 0x46};
    static const uint8_t S[5] = {0x46, 0x49, 0x49, 0x49, 0x31};
    static const uint8_t T[5] = {0x01, 0x01, 0x7F, 0x01, 0x01};
    static const uint8_t U[5] = {0x3F, 0x40, 0x40, 0x40, 0x3F};
    static const uint8_t V[5] = {0x1F, 0x20, 0x40, 0x20, 0x1F};
    static const uint8_t X[5] = {0x63, 0x14, 0x08, 0x14, 0x63};
    static const uint8_t colon[5] = {0x00, 0x36, 0x36, 0x00, 0x00};
    static const uint8_t dot[5] = {0x00, 0x60, 0x60, 0x00, 0x00};
    static const uint8_t minus[5] = {0x08, 0x08, 0x08, 0x08, 0x08};
    static const uint8_t slash[5] = {0x20, 0x10, 0x08, 0x04, 0x02};
    static const uint8_t gt[5] = {0x41, 0x22, 0x14, 0x08, 0x00};

    switch (c) {
    case '0': g = n0; break;
    case '1': g = n1; break;
    case '2': g = n2; break;
    case '3': g = n3; break;
    case '4': g = n4; break;
    case '5': g = n5; break;
    case '6': g = n6; break;
    case '7': g = n7; break;
    case '8': g = n8; break;
    case '9': g = n9; break;
    case 'A': g = A; break;
    case 'C': g = C; break;
    case 'D': g = D; break;
    case 'E': g = E; break;
    case 'F': g = F; break;
    case 'G': g = G; break;
    case 'H': g = H; break;
    case 'I': g = I; break;
    case 'L': g = L; break;
    case 'M': g = M; break;
    case 'N': g = N; break;
    case 'O': g = O; break;
    case 'P': g = P; break;
    case 'R': g = R; break;
    case 'S': g = S; break;
    case 'T': g = T; break;
    case 'U': g = U; break;
    case 'V': g = V; break;
    case 'X': g = X; break;
    case ':': g = colon; break;
    case '.': g = dot; break;
    case '-': g = minus; break;
    case '/': g = slash; break;
    case '>': g = gt; break;
    default: break;
    }

    memcpy(out, g, 5);
}

static void lcd_draw_char(esp_lcd_panel_handle_t panel,
                          int x,
                          int y,
                          char c,
                          uint16_t color,
                          uint8_t scale)
{
    uint8_t glyph[5];

    font5x7_get(c, glyph);
    for (int col = 0; col < 5; ++col) {
        for (int row = 0; row < 7; ++row) {
            if ((glyph[col] & (1U << row)) != 0U) {
                lcd_fill_rect(panel,
                              x + col * scale,
                              y + row * scale,
                              x + (col + 1) * scale,
                              y + (row + 1) * scale,
                              color);
            }
        }
    }
}

static void lcd_draw_text(esp_lcd_panel_handle_t panel,
                          int x,
                          int y,
                          const char *text,
                          uint16_t color,
                          uint8_t scale)
{
    while (text != NULL && *text != '\0') {
        lcd_draw_char(panel, x, y, *text, color, scale);
        x += 6 * scale;
        text++;
    }
}

static int waveform_y(float value, int center_y)
{
    const float half_scale = 30.0f;
    int y;

    if (value > UI_FIXED_RANGE) {
        value = UI_FIXED_RANGE;
    } else if (value < -UI_FIXED_RANGE) {
        value = -UI_FIXED_RANGE;
    }

    y = center_y - (int)((value / UI_FIXED_RANGE) * half_scale);
    if (y < UI_WAVE_Y0) {
        y = UI_WAVE_Y0;
    } else if (y >= (UI_WAVE_Y0 + UI_WAVE_H)) {
        y = UI_WAVE_Y0 + UI_WAVE_H - 1;
    }
    return y;
}

static void ui_draw_grid(esp_lcd_panel_handle_t panel)
{
    const uint16_t grid = 0x0320;
    const uint16_t axis = 0x07E0;
    const int ch1_center = UI_WAVE_Y0 + UI_WAVE_H / 4;
    const int ch2_center = UI_WAVE_Y0 + (UI_WAVE_H * 3) / 4;

    lcd_fill_screen(panel, 0x0000);
    lcd_fill_rect(panel, 0, 20, BOARD_LCD_WIDTH, 21, axis);
    lcd_fill_rect(panel, 0, UI_VALUE_Y0 - 2, BOARD_LCD_WIDTH, UI_VALUE_Y0 - 1, axis);
    lcd_fill_rect(panel, 0, UI_MENU_Y0 - 2, BOARD_LCD_WIDTH, UI_MENU_Y0 - 1, axis);

    for (int x = 0; x < UI_WAVE_W; x += 24) {
        lcd_fill_rect(panel, x, UI_WAVE_Y0, x + 1, UI_WAVE_Y0 + UI_WAVE_H, grid);
    }
    for (int y = UI_WAVE_Y0; y < UI_WAVE_Y0 + UI_WAVE_H; y += 17) {
        lcd_fill_rect(panel, 0, y, UI_WAVE_W, y + 1, grid);
    }
    lcd_fill_rect(panel, 0, ch1_center, UI_WAVE_W, ch1_center + 1, 0x05A0);
    lcd_fill_rect(panel, 0, ch2_center, UI_WAVE_W, ch2_center + 1, 0x05A0);
    lcd_draw_text(panel, 4, 6, "EEG SCOPE", axis, 1);
    lcd_draw_text(panel, 4, ch1_center - 12, "CH1", axis, 1);
    lcd_draw_text(panel, 4, ch2_center - 12, "CH2", axis, 1);
}

static void ui_draw_values(esp_lcd_panel_handle_t panel)
{
    char line[32];
    const uint16_t fg = 0x07E0;

    lcd_fill_rect(panel, 0, UI_VALUE_Y0, BOARD_LCD_WIDTH, UI_MENU_Y0 - 2, 0x0000);
    snprintf(line, sizeof(line), "CH1:%7.3f", (double)waveform_latest_ch1);
    lcd_draw_text(panel, 4, UI_VALUE_Y0 + 4, line, fg, 1);
    snprintf(line, sizeof(line), "CH2:%7.3f", (double)waveform_latest_ch2);
    lcd_draw_text(panel, 4, UI_VALUE_Y0 + 20, line, fg, 1);
    snprintf(line, sizeof(line), "DROP:%lu", (unsigned long)waveform_drop_count);
    lcd_draw_text(panel, 128, UI_VALUE_Y0 + 20, line, fg, 1);
}

static void ui_draw_menu(esp_lcd_panel_handle_t panel)
{
    static const char *items[] = {"RUN", "CH1", "CH2", "CLR"};
    char label[8];
    const uint16_t fg = 0x07E0;

    lcd_fill_rect(panel, 0, UI_MENU_Y0, BOARD_LCD_WIDTH, BOARD_LCD_HEIGHT, 0x0000);
    for (int i = 0; i < 4; ++i) {
        snprintf(label, sizeof(label), "%s%s", (ui_menu_index == i) ? ">" : " ", items[i]);
        lcd_draw_text(panel, 4 + i * 58, UI_MENU_Y0 + 6, label, fg, 1);
    }
    lcd_draw_text(panel, 4, UI_MENU_Y0 + 20, ui_pause ? "PAUSE" : "LIVE", fg, 1);
    lcd_draw_text(panel, 70, UI_MENU_Y0 + 20, ui_show_ch1 ? "C1ON" : "C1OFF", fg, 1);
    lcd_draw_text(panel, 140, UI_MENU_Y0 + 20, ui_show_ch2 ? "C2ON" : "C2OFF", fg, 1);
}

static void lcd_scope_task(void *arg)
{
    esp_lcd_panel_handle_t panel = (esp_lcd_panel_handle_t)arg;
    waveform_sample_t sample;
    uint32_t decimate = 0;
    uint32_t last_ui_ms = 0;
    int x = 0;
    int prev_y1 = waveform_y(0.0f, UI_WAVE_Y0 + UI_WAVE_H / 4);
    int prev_y2 = waveform_y(0.0f, UI_WAVE_Y0 + (UI_WAVE_H * 3) / 4);

    ui_draw_grid(panel);
    ui_draw_values(panel);
    ui_draw_menu(panel);

    while (true) {
        if (ui_clear_request) {
            ui_clear_request = false;
            x = 0;
            prev_y1 = waveform_y(0.0f, UI_WAVE_Y0 + UI_WAVE_H / 4);
            prev_y2 = waveform_y(0.0f, UI_WAVE_Y0 + (UI_WAVE_H * 3) / 4);
            ui_draw_grid(panel);
            ui_draw_values(panel);
            ui_draw_menu(panel);
        }

        while (!ui_pause && waveform_fifo_pop(&sample)) {
            decimate++;
            if ((decimate % UI_SAMPLE_DECIMATION) != 0U) {
                continue;
            }

            const int ch1_center = UI_WAVE_Y0 + UI_WAVE_H / 4;
            const int ch2_center = UI_WAVE_Y0 + (UI_WAVE_H * 3) / 4;
            const int y1 = waveform_y(sample.ch1, ch1_center);
            const int y2 = waveform_y(sample.ch2, ch2_center);
            const int next_x = (x + 1) % UI_WAVE_W;

            lcd_fill_rect(panel, x, UI_WAVE_Y0, x + 2, UI_WAVE_Y0 + UI_WAVE_H, 0x0000);
            if (ui_show_ch1) {
                lcd_draw_line(panel, x > 0 ? x - 1 : x, prev_y1, x, y1, 0x07E0);
            }
            if (ui_show_ch2) {
                lcd_draw_line(panel, x > 0 ? x - 1 : x, prev_y2, x, y2, 0x04FF);
            }

            prev_y1 = y1;
            prev_y2 = y2;
            x = next_x;
            if (x == 0) {
                ui_draw_grid(panel);
            }
        }

        const uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        if ((now_ms - last_ui_ms) >= 250U) {
            char status[32];
            lcd_fill_rect(panel, 128, 4, BOARD_LCD_WIDTH, 18, 0x0000);
            snprintf(status, sizeof(status), "RX:%lu", (unsigned long)waveform_rx_count);
            lcd_draw_text(panel, 158, 6, status, 0x07E0, 1);
            ui_draw_values(panel);
            ui_draw_menu(panel);
            last_ui_ms = now_ms;
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

#endif

#if BOARD_ENABLE_ENCODER
static void encoder_task(void *arg)
{
    const gpio_config_t config = {
        .pin_bit_mask = (1ULL << BOARD_ENCODER_A_GPIO) |
                        (1ULL << BOARD_ENCODER_B_GPIO) |
                        (1ULL << BOARD_ENCODER_KEY_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&config));

    int last_a = gpio_get_level(BOARD_ENCODER_A_GPIO);
    int last_key = gpio_get_level(BOARD_ENCODER_KEY_GPIO);
    uint32_t key_down_ms = 0;

    while (true) {
        const uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        const int a = gpio_get_level(BOARD_ENCODER_A_GPIO);
        const int b = gpio_get_level(BOARD_ENCODER_B_GPIO);
        const int key = gpio_get_level(BOARD_ENCODER_KEY_GPIO);

        if (a != last_a && a == 1) {
            if (b != a) {
                ui_menu_index = (uint8_t)((ui_menu_index + 1U) % 4U);
            } else {
                ui_menu_index = (uint8_t)((ui_menu_index + 3U) % 4U);
            }
        }
        if (key != last_key) {
            if (key == 0) {
                key_down_ms = now_ms;
            } else {
                const uint32_t press_ms = now_ms - key_down_ms;
                if (press_ms >= 800U) {
                    ui_menu_index = 0U;
                } else {
                    switch (ui_menu_index) {
                    case 0:
                        ui_pause = !ui_pause;
                        break;
                    case 1:
                        ui_show_ch1 = !ui_show_ch1;
                        ui_clear_request = true;
                        break;
                    case 2:
                        ui_show_ch2 = !ui_show_ch2;
                        ui_clear_request = true;
                        break;
                    default:
                        ui_clear_request = true;
                        break;
                    }
                }
            }
        }
        last_a = a;
        last_key = key;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}
#endif

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_NONE);
    heart_rate_self_check();
    init_stm32_uart();
    init_asrpro_uart();
    init_gps_uart();
    xTaskCreate(gps_uart_task, "gps_uart", 3072, NULL, 5, NULL);
    xTaskCreate(asrpro_uart_task, "asrpro_uart", 2048, NULL, 5, NULL);
    init_wifi_sta();
    start_websocket_server();
#if BOARD_ENABLE_WEB_TEST_SIGNAL
    xTaskCreate(web_test_signal_task, "web_test", 3072, NULL, 6, NULL);
#else
    xTaskCreate(stm32_uart_task, "stm32_uart", 4096, NULL, 6, NULL);
#endif
#if BOARD_ENABLE_ENCODER
    xTaskCreate(encoder_task, "encoder", 3072, NULL, 4, NULL);
#endif

#if BOARD_ENABLE_LCD
    esp_lcd_panel_handle_t panel = init_lcd();
    xTaskCreate(lcd_scope_task, "lcd_scope", 4096, panel, 3, NULL);
#else
    ESP_LOGI(TAG, "LCD init disabled for UART/monitor stability test");
#endif
}

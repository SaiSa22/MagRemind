/* Wi-Fi Provisioning + Dual Button + Single Master Task Architecture */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include "esp_task_wdt.h"
#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include "esp_sleep.h"
#include <wifi_provisioning/manager.h>
#include "esp_sntp.h"
#include "time.h"

#ifdef CONFIG_EXAMPLE_PROV_TRANSPORT_BLE
#include <wifi_provisioning/scheme_ble.h>
#endif
#ifdef CONFIG_EXAMPLE_PROV_TRANSPORT_SOFTAP
#include <wifi_provisioning/scheme_softap.h>
#endif
#include "qrcode.h"
#include "cJSON.h"
#include "esp_http_client.h"
#include "esp_spiffs.h"
#include "esp_crt_bundle.h"

/* --- PIN CONFIGURATION --- */
#define I2S_BCLK_IO1       GPIO_NUM_10  // Pin D10
#define I2S_WS_IO1         GPIO_NUM_9   // Pin D9
#define I2S_DO_IO1         GPIO_NUM_8   // Pin D8

#define BUTTON_PLAY_PIN    GPIO_NUM_3   // Pin D1
#define BUTTON_UPDATE_PIN  GPIO_NUM_4   // Pin D2

/* --- MP3 DECODER --- */
#define MINIMP3_IMPLEMENTATION
#define MINIMP3_NO_STDIO
#include "minimp3.h"

#define MP3_READ_BUFFER_SIZE 8192   
#define PCM_OUTPUT_BUFFER_SIZE 4608 

static const char *TAG = "app";

/* [PROVISIONING BOILERPLATE START] */
#if CONFIG_EXAMPLE_PROV_SECURITY_VERSION_2
    #if CONFIG_EXAMPLE_PROV_SEC2_DEV_MODE
    #define EXAMPLE_PROV_SEC2_USERNAME "wifiprov"
    #define EXAMPLE_PROV_SEC2_PWD "abcd1234"
    static const char sec2_salt[] = { 0x03, 0x6e, 0xe0, 0xc7, 0xbc, 0xb9, 0xed, 0xa8, 0x4c, 0x9e, 0xac, 0x97, 0xd9, 0x3d, 0xec, 0xf4 };
    static const char sec2_verifier[] = { 0x7c, 0x7c, 0x85, 0x47, 0x65, 0x08, 0x94, 0x6d, 0xd6, 0x36, 0xaf, 0x37, 0xd7, 0xe8, 0x91, 0x43, 0x78, 0xcf, 0xfd, 0x61, 0x6c, 0x59, 0xd2, 0xf8, 0x39, 0x08, 0x12, 0x72, 0x38, 0xde, 0x9e, 0x24, 0xa4, 0x70, 0x26, 0x1c, 0xdf, 0xa9, 0x03, 0xc2, 0xb2, 0x70, 0xe7, 0xb1, 0x32, 0x24, 0xda, 0x11, 0x1d, 0x97, 0x18, 0xdc, 0x60, 0x72, 0x08, 0xcc, 0x9a, 0xc9, 0x0c, 0x48, 0x27, 0xe2, 0xae, 0x89, 0xaa, 0x16, 0x25, 0xb8, 0x04, 0xd2, 0x1a, 0x9b, 0x3a, 0x8f, 0x37, 0xf6, 0xe4, 0x3a, 0x71, 0x2e, 0xe1, 0x27, 0x86, 0x6e, 0xad, 0xce, 0x28, 0xff, 0x54, 0x46, 0x60, 0x1f, 0xb9, 0x96, 0x87, 0xdc, 0x57, 0x40, 0xa7, 0xd4, 0x6c, 0xc9, 0x77, 0x54, 0xdc, 0x16, 0x82, 0xf0, 0xed, 0x35, 0x6a, 0xc4, 0x70, 0xad, 0x3d, 0x90, 0xb5, 0x81, 0x94, 0x70, 0xd7, 0xbc, 0x65, 0xb2, 0xd5, 0x18, 0xe0, 0x2e, 0xc3, 0xa5, 0xf9, 0x68, 0xdd, 0x64, 0x7b, 0xb8, 0xb7, 0x3c, 0x9c, 0xfc, 0x00, 0xd8, 0x71, 0x7e, 0xb7, 0x9a, 0x7c, 0xb1, 0xb7, 0xc2, 0xc3, 0x18, 0x34, 0x29, 0x32, 0x43, 0x3e, 0x00, 0x99, 0xe9, 0x82, 0x94, 0xe3, 0xd8, 0x2a, 0xb0, 0x96, 0x29, 0xb7, 0xdf, 0x0e, 0x5f, 0x08, 0x33, 0x40, 0x76, 0x52, 0x91, 0x32, 0x00, 0x9f, 0x97, 0x2c, 0x89, 0x6c, 0x39, 0x1e, 0xc8, 0x28, 0x05, 0x44, 0x17, 0x3f, 0x68, 0x02, 0x8a, 0x9f, 0x44, 0x61, 0xd1, 0xf5, 0xa1, 0x7e, 0x5a, 0x70, 0xd2, 0xc7, 0x23, 0x81, 0xcb, 0x38, 0x68, 0xe4, 0x2c, 0x20, 0xbc, 0x40, 0x57, 0x76, 0x17, 0xbd, 0x08, 0xb8, 0x96, 0xbc, 0x26, 0xeb, 0x32, 0x46, 0x69, 0x35, 0x05, 0x8c, 0x15, 0x70, 0xd9, 0x1b, 0xe9, 0xbe, 0xcc, 0xa9, 0x38, 0xa6, 0x67, 0xf0, 0xad, 0x50, 0x13, 0x19, 0x72, 0x64, 0xbf, 0x52, 0xc2, 0x34, 0xe2, 0x1b, 0x11, 0x79, 0x74, 0x72, 0xbd, 0x34, 0x5b, 0xb1, 0xe2, 0xfd, 0x66, 0x73, 0xfe, 0x71, 0x64, 0x74, 0xd0, 0x4e, 0xbc, 0x51, 0x24, 0x19, 0x40, 0x87, 0x0e, 0x92, 0x40, 0xe6, 0x21, 0xe7, 0x2d, 0x4e, 0x37, 0x76, 0x2f, 0x2e, 0xe2, 0x68, 0xc7, 0x89, 0xe8, 0x32, 0x13, 0x42, 0x06, 0x84, 0x84, 0x53, 0x4a, 0xb3, 0x0c, 0x1b, 0x4c, 0x8d, 0x1c, 0x51, 0x97, 0x19, 0xab, 0xae, 0x77, 0xff, 0xdb, 0xec, 0xf0, 0x10, 0x95, 0x34, 0x33, 0x6b, 0xcb, 0x3e, 0x84, 0x0f, 0xb9, 0xd8, 0x5f, 0xb8, 0xa0, 0xb8, 0x55, 0x53, 0x3e, 0x70, 0xf7, 0x18, 0xf5, 0xce, 0x7b, 0x4e, 0xbf, 0x27, 0xce, 0xce, 0xa8, 0xb3, 0xbe, 0x40, 0xc5, 0xc5, 0x32, 0x29, 0x3e, 0x71, 0x64, 0x9e, 0xde, 0x8c, 0xf6, 0x75, 0xa1, 0xe6, 0xf6, 0x53, 0xc8, 0x31, 0xa8, 0x78, 0xde, 0x50, 0x40, 0xf7, 0x62, 0xde, 0x36, 0xb2, 0xba };
    static __attribute__((unused)) esp_err_t example_get_sec2_salt(const char **salt, uint16_t *salt_len) { *salt = sec2_salt; *salt_len = sizeof(sec2_salt); return ESP_OK; }
    static __attribute__((unused)) esp_err_t example_get_sec2_verifier(const char **verifier, uint16_t *verifier_len) { *verifier = sec2_verifier; *verifier_len = sizeof(sec2_verifier); return ESP_OK; }
    #endif
#endif

const int WIFI_CONNECTED_EVENT = BIT0;
static EventGroupHandle_t wifi_event_group;
#define PROV_QR_VERSION "v1"
#define PROV_TRANSPORT_SOFTAP "softap"
#define PROV_TRANSPORT_BLE "ble"
#define QRCODE_BASE_URL "https://espressif.github.io/esp-jumpstart/qrcode.html"

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_PROV_EVENT) {
        switch (event_id) {
            case WIFI_PROV_START: ESP_LOGI(TAG, "Provisioning started"); break;
            case WIFI_PROV_CRED_RECV: ESP_LOGI(TAG, "Received Wi-Fi credentials"); break;
            case WIFI_PROV_CRED_FAIL: ESP_LOGE(TAG, "Provisioning failed!"); break;
            case WIFI_PROV_CRED_SUCCESS: ESP_LOGI(TAG, "Provisioning successful"); break;
            case WIFI_PROV_END: wifi_prov_mgr_deinit(); break;
        }
    } else if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START: esp_wifi_connect(); break;
            case WIFI_EVENT_STA_DISCONNECTED: ESP_LOGI(TAG, "Disconnected. Reconnecting..."); esp_wifi_connect(); break;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Connected with IP Address:" IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_EVENT);
    }
}

static void wifi_init_sta(void) {
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void get_device_service_name(char *service_name, size_t max) {
    uint8_t eth_mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, eth_mac);
    snprintf(service_name, max, "PROV_%02X%02X%02X", eth_mac[3], eth_mac[4], eth_mac[5]);
}

esp_err_t custom_prov_data_handler(uint32_t session_id, const uint8_t *inbuf, ssize_t inlen, uint8_t **outbuf, ssize_t *outlen, void *priv_data) {
    char response[] = "SUCCESS";
    *outbuf = (uint8_t *)strdup(response);
    if (*outbuf == NULL) return ESP_ERR_NO_MEM;
    *outlen = strlen(response) + 1;
    return ESP_OK;
}

static void wifi_prov_print_qr(const char *name, const char *username, const char *pop, const char *transport) {
    ESP_LOGI(TAG, "If QR code is not visible, copy paste the below URL in a browser.\n%s?data={\"ver\":\"%s\",\"name\":\"%s\",\"pop\":\"%s\",\"transport\":\"%s\"}", QRCODE_BASE_URL, PROV_QR_VERSION, name, pop ? pop : "", transport);
}

#ifdef CONFIG_EXAMPLE_PROV_ENABLE_APP_CALLBACK
void wifi_prov_app_callback(void *user_data, wifi_prov_cb_event_t event, void *event_data) { }
const wifi_prov_event_handler_t wifi_prov_event_handler = { .event_cb = wifi_prov_app_callback, .user_data = NULL };
#endif
/* [PROVISIONING BOILERPLATE END] */


#define MANIFEST_URL "https://remindaudio.sfo3.digitaloceanspaces.com/status.json"
#define STORAGE_PATH "/spiffs/audio.mp3"
#define NVS_VERSION_KEY "audio_ver"
#define MAX_HTTP_BUFFER 1024


void obtain_time(void) {
    ESP_LOGI(TAG, "Initializing SNTP...");
    
    /* 1. Config & Start SNTP */
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "time.google.com");
    esp_sntp_init();

    /* 2. Wait for time to be set (with 10s timeout) */
    int retry = 0;
    const int retry_count = 10;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }

    /* 3. Verify and Log */
    if (sntp_get_sync_status() == SNTP_SYNC_STATUS_COMPLETED) {
        ESP_LOGI(TAG, "Time updated via NTP!");
        
        // Get the raw Unix timestamp (which is now stored in the RTC)
        time_t now;
        time(&now);
        ESP_LOGI(TAG, "RTC Initialized with Unix Timestamp: %ld", (long)now);
    } else {
        ESP_LOGE(TAG, "Could not get time from NTP!");
    }
}


void print_current_date() {
    time_t now;
    struct tm timeinfo;
    
    /* 1. Get raw Unix timestamp from the RTC */
    time(&now);
    
    /* 2. Convert to Readable Structure */
    localtime_r(&now, &timeinfo);

    /* 3. Create formatted string (YYYY-MM-DD HH:MM:SS) */
    char strftime_buf[64];
    strftime(strftime_buf, sizeof(strftime_buf), "%Y-%m-%d %H:%M:%S", &timeinfo);

    /* 4. Print both in one line */
    ESP_LOGI(TAG, "Unix Timestamp: %ld | Date: %s", (long)now, strftime_buf);
}

/* ---------------------------------------------------------
   STORAGE & DOWNLOAD LOGIC
   --------------------------------------------------------- */
static void mount_storage(void) {
    esp_vfs_spiffs_conf_t conf = { .base_path = "/spiffs", .partition_label = "storage", .max_files = 5, .format_if_mount_failed = true };
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) ESP_LOGE(TAG, "Failed to mount SPIFFS");
    else ESP_LOGI(TAG, "Storage mounted successfully");
}

void save_version(int new_version) {
    nvs_handle_t my_handle;
    if (nvs_open("storage", NVS_READWRITE, &my_handle) == ESP_OK) {
        nvs_set_i32(my_handle, NVS_VERSION_KEY, new_version);
        nvs_commit(my_handle);
        nvs_close(my_handle);
    }
}

int get_current_version() {
    nvs_handle_t my_handle;
    int32_t version = 0;
    if (nvs_open("storage", NVS_READWRITE, &my_handle) == ESP_OK) {
        nvs_get_i32(my_handle, NVS_VERSION_KEY, &version);
        nvs_close(my_handle);
    }
    return version;
}

esp_err_t download_url_to_file(const char *url, const char *filepath) {
    ESP_LOGI(TAG, "Downloading: %s", url);
    esp_http_client_config_t config = { .url = url, .crt_bundle_attach = esp_crt_bundle_attach, .timeout_ms = 20000 };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) { esp_http_client_cleanup(client); return ESP_FAIL; }
    
    esp_http_client_fetch_headers(client);
    FILE *f = fopen(filepath, "wb");
    if (f == NULL) { esp_http_client_cleanup(client); return ESP_FAIL; }
    
    char *buffer = malloc(MAX_HTTP_BUFFER);
    if (buffer == NULL) { fclose(f); esp_http_client_cleanup(client); return ESP_FAIL; }
    
    while (1) {
        int read_len = esp_http_client_read(client, buffer, MAX_HTTP_BUFFER);
        if (read_len > 0) fwrite(buffer, 1, read_len, f);
        else break;
    }
    free(buffer); fclose(f); esp_http_client_cleanup(client);
    return ESP_OK;
}

void check_and_update_content() {
    ESP_LOGI(TAG, "Checking for updates...");
    char *json_buffer = malloc(1024);
    if (json_buffer == NULL) return;

    esp_http_client_config_t config = { .url = MANIFEST_URL, .crt_bundle_attach = esp_crt_bundle_attach, .timeout_ms = 10000 };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (esp_http_client_open(client, 0) == ESP_OK) {
        esp_http_client_fetch_headers(client);
        int read_len = esp_http_client_read(client, json_buffer, 1024);
        if (read_len > 0) {
            json_buffer[read_len] = 0;
            ESP_LOGW(TAG, "RAW CONTENT: \n%s\n", json_buffer); 
            cJSON *json = cJSON_Parse(json_buffer);
            if (json) {
                cJSON *ver_item = cJSON_GetObjectItem(json, "version");
                cJSON *url_item = cJSON_GetObjectItem(json, "audio_url");
                if (ver_item && url_item) {
                    int cloud_ver = ver_item->valueint;
                    int local_ver = get_current_version();
                    ESP_LOGI(TAG, "Cloud: %d | Local: %d", cloud_ver, local_ver);
                    if (cloud_ver != local_ver) {
                        if (download_url_to_file(url_item->valuestring, STORAGE_PATH) == ESP_OK) {
                            save_version(cloud_ver);
                            ESP_LOGI(TAG, "Update Saved.");
                        }
                    } else ESP_LOGI(TAG, "Up to date.");
                }
                cJSON_Delete(json);
            }
        }
    }
    esp_http_client_cleanup(client); free(json_buffer);
}

/* ---------------------------------------------------------
   HELPER: Centralized Deep Sleep Logic
   --------------------------------------------------------- */
void enter_deep_sleep(void) {
    ESP_LOGI(TAG, "Job Done. Configuring Deep Sleep...");

    /* 1. Define Bitmask for BOTH Update (D2/GPIO4) and Play (D1/GPIO3) */
    uint64_t pin_mask = (1ULL << BUTTON_UPDATE_PIN) | (1ULL << BUTTON_PLAY_PIN);

    /* 2. Configure GPIOs as Input with Pull-Up */
    const gpio_config_t wakeup_conf = {
        .pin_bit_mask = pin_mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,   
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&wakeup_conf);

    /* 3. Enable Wake-up on LOW Logic Level */
    ESP_ERROR_CHECK(esp_deep_sleep_enable_gpio_wakeup(pin_mask, ESP_GPIO_WAKEUP_GPIO_LOW));

    ESP_LOGI(TAG, "Entering Deep Sleep. Press D1 (Play) or D2 (Update) to wake.");
    fflush(stdout); 
    esp_deep_sleep_start();
}


/* I2S & Audio Logic */
i2s_chan_handle_t tx_handle = NULL;

void init_i2s(void) {
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = 32; chan_cfg.dma_frame_num = 512;   
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle, NULL));

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(16000), 
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCLK_IO1, .ws = I2S_WS_IO1, .dout = I2S_DO_IO1,
            .din = I2S_GPIO_UNUSED, .invert_flags = {0},
        },
    };
    std_cfg.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT;
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));
    ESP_LOGI(TAG, "I2S Initialized: 16kHz / Stereo Output / 32-bit Slots");
}

/* --- THE WORKING PLAY FUNCTION (DIRECT CALL) --- */
void play_mp3(const char *filepath) {
    ESP_LOGI(TAG, "Opening file: %s", filepath);
    FILE *f = fopen(filepath, "rb");
    if (!f) { ESP_LOGE(TAG, "Failed to open file"); return; }

    mp3dec_t *mp3d = malloc(sizeof(mp3dec_t)); 
    mp3dec_frame_info_t info;
    mp3dec_init(mp3d);

    uint8_t *read_buf = malloc(MP3_READ_BUFFER_SIZE);
    int16_t *pcm_buf = malloc(PCM_OUTPUT_BUFFER_SIZE); 
    
    if (!read_buf || !pcm_buf) {
        if(read_buf) free(read_buf); 
        if(pcm_buf) free(pcm_buf); 
        free(mp3d); 
        fclose(f); 
        return;
    }

    size_t bytes_in_buf = fread(read_buf, 1, MP3_READ_BUFFER_SIZE, f);
    uint8_t *buf_ptr = read_buf;
    int samples;
    
    int frame_count = 0; 

    /* Add this task to the Watchdog so we can feed it */
    esp_task_wdt_add(NULL);
    ESP_LOGI(TAG, "Starting Playback (Original Logic)...");

    while (bytes_in_buf > 0) {
        esp_task_wdt_reset();

        samples = mp3dec_decode_frame(mp3d, buf_ptr, bytes_in_buf, pcm_buf, &info);
        buf_ptr += info.frame_bytes;
        bytes_in_buf -= info.frame_bytes;

        if (samples > 0) {
            if (info.channels == 1) {
                for (int i = samples - 1; i >= 0; i--) {
                    pcm_buf[2 * i] = pcm_buf[i];     
                    pcm_buf[2 * i + 1] = pcm_buf[i]; 
                }
            }
            size_t bytes_written = 0;
            i2s_channel_write(tx_handle, pcm_buf, samples * 2 * 2, &bytes_written, 1000);
        }

        if (bytes_in_buf < 4096 && !feof(f)) {
            memmove(read_buf, buf_ptr, bytes_in_buf);
            size_t bytes_read = fread(read_buf + bytes_in_buf, 1, MP3_READ_BUFFER_SIZE - bytes_in_buf, f);
            bytes_in_buf += bytes_read;
            buf_ptr = read_buf;
            frame_count = 0;
        }

        frame_count++;
        if (frame_count >= 10) {
            vTaskDelay(1); 
            frame_count = 0;
        }
    }

    /* Flush */
    memset(pcm_buf, 0, PCM_OUTPUT_BUFFER_SIZE); 
    size_t bytes_written = 0;
    for (int i = 0; i < 15; i++) {
        esp_task_wdt_reset();
        i2s_channel_write(tx_handle, pcm_buf, PCM_OUTPUT_BUFFER_SIZE, &bytes_written, 1000);
    }
    
    /* Wait for silence to play (Fixes chirp) */
    vTaskDelay(pdMS_TO_TICKS(1000));

    i2s_channel_disable(tx_handle);
    esp_task_wdt_delete(NULL);
    ESP_LOGI(TAG, "Done.");
    
    free(read_buf); free(pcm_buf); free(mp3d); fclose(f);
}


/* ---------------------------------------------------------
   MASTER TASK (REPLACES APP_MAIN LOGIC)
   --------------------------------------------------------- */
/* This task will handle EVERYTHING in order: Buttons -> Wi-Fi -> Audio -> Sleep */
/* We run it here to guarantee 32KB stack without messing with app_main size */
void master_task(void *pvParameters)
{    
    /* 1. Determine Wake-Up Source */
    uint64_t wakeup_pin_mask = esp_sleep_get_gpio_wakeup_status();
    
    bool is_play_button = (wakeup_pin_mask & (1ULL << BUTTON_PLAY_PIN)); 
    bool is_update_button = (wakeup_pin_mask & (1ULL << BUTTON_UPDATE_PIN));

    /* --- LOGIC: If UPDATE Button (D2) OR Power On -> Run Wi-Fi Update --- */
    /* If PLAY Button (D1) -> Skip Wi-Fi */
    bool run_update = is_update_button || (!is_play_button && !is_update_button);

    if (run_update) {
        if (is_update_button) ESP_LOGI(TAG, "Wake Source: UPDATE BUTTON (D2).");
        else ESP_LOGI(TAG, "Wake Source: POWER ON/RESET. Defaulting to Update Check.");

        /* --- Wi-Fi & Provisioning Block --- */
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_ERROR_CHECK(nvs_flash_erase()); ESP_ERROR_CHECK(nvs_flash_init());
        }

        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        wifi_event_group = xEventGroupCreate();

        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
        #ifdef CONFIG_EXAMPLE_PROV_TRANSPORT_BLE
        ESP_ERROR_CHECK(esp_event_handler_register(PROTOCOMM_TRANSPORT_BLE_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
        #endif
        ESP_ERROR_CHECK(esp_event_handler_register(PROTOCOMM_SECURITY_SESSION_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

        esp_netif_create_default_wifi_sta();
        #ifdef CONFIG_EXAMPLE_PROV_TRANSPORT_SOFTAP
        esp_netif_create_default_wifi_ap();
        #endif
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));

        /* Provisioning Config */
        wifi_prov_mgr_config_t config = {
            #ifdef CONFIG_EXAMPLE_PROV_TRANSPORT_BLE
            .scheme = wifi_prov_scheme_ble,
            .scheme_event_handler = WIFI_PROV_SCHEME_BLE_EVENT_HANDLER_FREE_BTDM
            #endif
            #ifdef CONFIG_EXAMPLE_PROV_TRANSPORT_SOFTAP
            .scheme = wifi_prov_scheme_softap,
            .scheme_event_handler = WIFI_PROV_EVENT_HANDLER_NONE
            #endif
        };
        ESP_ERROR_CHECK(wifi_prov_mgr_init(config));

        bool provisioned = false;
        ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&provisioned));

        if (!provisioned) {
            ESP_LOGI(TAG, "Starting provisioning");
            char service_name[12];
            get_device_service_name(service_name, sizeof(service_name));
            wifi_prov_security_t security = WIFI_PROV_SECURITY_1;
            const char *pop = "abcd1234";
            wifi_prov_security1_params_t *sec_params = (wifi_prov_security1_params_t *)pop;
            const char *service_key = NULL;

            wifi_prov_mgr_endpoint_create("custom-data");
            ESP_ERROR_CHECK(wifi_prov_mgr_start_provisioning(security, (const void *) sec_params, service_name, service_key));
            wifi_prov_mgr_endpoint_register("custom-data", custom_prov_data_handler, NULL);
            #ifdef CONFIG_EXAMPLE_PROV_TRANSPORT_BLE
            wifi_prov_print_qr(service_name, NULL, pop, PROV_TRANSPORT_BLE);
            #else
            wifi_prov_print_qr(service_name, NULL, pop, PROV_TRANSPORT_SOFTAP);
            #endif
        } else {
            ESP_LOGI(TAG, "Already provisioned, starting Wi-Fi STA");
            wifi_prov_mgr_deinit();
            ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
            wifi_init_sta();
        }

        /* Wait for Connection */
        xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_EVENT, true, true, portMAX_DELAY);

        /* The RTC is now set! It will keep counting even in Deep Sleep. */
        obtain_time();

        /* Mount & Update */
        mount_storage();
        check_and_update_content();

        /* SHUT DOWN WI-FI BEFORE PLAYING */
        ESP_LOGI(TAG, "Update Check Complete. Shutting down Wi-Fi...");
        esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler);
        esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler);
        esp_wifi_disconnect();
        esp_wifi_stop();
        esp_wifi_deinit();
        
    } else {
        ESP_LOGI(TAG, "Wake Source: PLAY BUTTON (D1). Skipping Wi-Fi.");
        mount_storage();
    }

    /* ---------------------------------------------------------
       COMMON PLAYBACK LOGIC (Runs for BOTH Play Mode and Update Mode)
       --------------------------------------------------------- */
    ESP_LOGI(TAG, "Starting Audio Playback...");

    //Print Current Time
    print_current_date();

    struct stat st;
    if (stat(STORAGE_PATH, &st) == 0) {
        ESP_LOGI(TAG, "File found. Playing...");
        init_i2s();
        
        /* Direct call here works because this task has 32KB stack */
        play_mp3(STORAGE_PATH);
        
    } else {
        ESP_LOGE(TAG, "No file found!");
    }
    
    /* SLEEP WHEN DONE */
    enter_deep_sleep();
    //vTaskDelete(NULL); // Should not reach here
}

/* APP MAIN just launches the master task to ensure stack size is correct */
void app_main(void) {
    xTaskCreate(master_task, "master_task", 32768, NULL, 5, NULL);
}
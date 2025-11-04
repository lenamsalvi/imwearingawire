#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <dirent.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "ssd1306.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "nvs_flash.h"

#define TAG "NAV_TEST"

// I2C Configuration
#define I2C_MASTER_SCL_IO           22      // GPIO for I2C clock
#define I2C_MASTER_SDA_IO           21      // GPIO for I2C data
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          400000  // 400kHz
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_TIMEOUT_MS       1000

// OLED Configuration
#define OLED_I2C_ADDRESS            0x3D
#define OLED_WIDTH                  128
#define OLED_HEIGHT                 64

// Button Configuration
#define BTN_LEFT_GPIO               32      // Previous screen
#define BTN_MIDDLE_GPIO             33      // Toggle invert
#define BTN_RIGHT_GPIO              34      // Next screen
#define BTN_DEBOUNCE_MS             200     // Debounce time in milliseconds

// SD Card Configuration (SPI)
#define SD_MISO_GPIO                19
#define SD_MOSI_GPIO                23
#define SD_CLK_GPIO                 18
#define SD_CS_GPIO                  5
#define MOUNT_POINT                 "/sdcard"

// I2S Microphone Configuration
#define I2S_WS_GPIO                 25
#define I2S_SCK_GPIO                26
#define I2S_SD_GPIO                 27
#define I2S_SAMPLE_RATE             16000
#define I2S_BITS_PER_SAMPLE         16
#define I2S_CHANNELS                1
#define I2S_BUFFER_SIZE             1024

// WAV file configuration
#define WAV_HEADER_SIZE             44
#define MAX_FILENAME_LEN            64
#define MAX_FILES_IN_LIST           20

// Bluetooth configuration
#define MAX_BT_DEVICES              10
#define BT_DEVICE_NAME_LEN          64

// Screen states
#define NUM_SCREENS                 4
typedef enum {
    SCREEN_RECORD = 0,
    SCREEN_PLAYBACK,
    SCREEN_BLUETOOTH,
    SCREEN_SYSTEM
} screen_state_t;

// Bluetooth states
typedef enum {
    BT_STATE_IDLE = 0,      // Not paired, not scanning
    BT_STATE_SCANNING,      // Discovering devices
    BT_STATE_PAIRING,       // Attempting to pair/connect
    BT_STATE_CONNECTED,     // Successfully connected
    BT_STATE_DISCONNECTING  // Disconnecting from device
} bt_state_t;

// Bluetooth device info
typedef struct {
    esp_bd_addr_t bda;                          // Bluetooth device address
    char name[BT_DEVICE_NAME_LEN];              // Device name
} bt_device_t;

// WAV header structure
typedef struct {
    char riff[4];
    uint32_t file_size;
    char wave[4];
    char fmt[4];
    uint32_t fmt_size;
    uint16_t audio_format;
    uint16_t num_channels;
    uint32_t sample_rate;
    uint32_t byte_rate;
    uint16_t block_align;
    uint16_t bits_per_sample;
    char data[4];
    uint32_t data_size;
} __attribute__((packed)) wav_header_t;

// Global state
static screen_state_t current_screen = SCREEN_RECORD;
static bool sd_initialized = false;
static uint32_t last_button_time = 0;

// Recording state
static i2s_chan_handle_t rx_handle = NULL;
static bool is_recording = false;
static FILE *recording_file = NULL;
static uint32_t recorded_samples = 0;
static uint32_t recording_counter = 1;
static char current_filename[MAX_FILENAME_LEN];
static uint32_t recording_start_time = 0;  // For elapsed time

// Playback state
static bool playback_browse_mode = false;
static char file_list[MAX_FILES_IN_LIST][MAX_FILENAME_LEN];
static int file_count = 0;
static int selected_file_index = 0;  // 0 = "Back", 1+ = actual files
static int scroll_offset = 0;  // Which item is at top of visible window

// System info
static uint64_t sd_total_bytes = 0;
static uint64_t sd_free_bytes = 0;

// Bluetooth state
static bt_state_t bt_state = BT_STATE_IDLE;
static bt_device_t bt_devices[MAX_BT_DEVICES];
static int bt_device_count = 0;
static int bt_selected_device = 0;
static int bt_scroll_offset = 0;
static bool bt_devices_updated = false;  // Flag to trigger screen refresh
static char bt_connected_device_name[BT_DEVICE_NAME_LEN] = {0};
static esp_bd_addr_t bt_connected_bda = {0};  // Address of connected device

// Forward declarations
static void scan_sd_files(void);
static void get_sd_card_info(void);
static void draw_button_labels(ssd1306_handle_t dev, const char *left, const char *middle, const char *right);
static void bt_gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
static void bt_a2dp_callback(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);
static int32_t bt_a2dp_data_callback(uint8_t *data, int32_t len);
static void bt_avrc_ct_callback(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param);
static void bt_avrc_tg_callback(esp_avrc_tg_cb_event_t event, esp_avrc_tg_cb_param_t *param);
static void bt_start_discovery(void);
static void bt_connect_device(int device_idx);
static void bt_disconnect(void);

// Scan SD card for WAV files and update file list
static void scan_sd_files(void)
{
    file_count = 0;
    int max_memo_num = 0;

    if (!sd_initialized) {
        return;
    }

    DIR *dir = opendir(MOUNT_POINT);
    if (dir == NULL) {
        ESP_LOGE(TAG, "Failed to open directory");
        return;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL && file_count < MAX_FILES_IN_LIST) {
        // Only include .wav files
        if (strstr(entry->d_name, ".wav") != NULL || strstr(entry->d_name, ".WAV") != NULL) {
            strncpy(file_list[file_count], entry->d_name, MAX_FILENAME_LEN - 1);
            file_list[file_count][MAX_FILENAME_LEN - 1] = '\0';
            file_count++;

            // Check if it's a memo_XXX.wav file to find highest number
            int num;
            if (sscanf(entry->d_name, "MEMO_%d.WAV", &num) == 1) {
                if (num > max_memo_num) {
                    max_memo_num = num;
                }
            }
        }
    }
    closedir(dir);

    // Set recording counter to next available number
    recording_counter = max_memo_num + 1;

    ESP_LOGI(TAG, "Found %d WAV files, next recording: memo_%03lu.wav", file_count, recording_counter);
}

// Get SD card size and free space
static void get_sd_card_info(void)
{
    if (!sd_initialized) {
        return;
    }

    FATFS *fs;
    DWORD fre_clust;

    if (f_getfree("0:", &fre_clust, &fs) == FR_OK) {
        uint64_t total_sectors = (fs->n_fatent - 2) * fs->csize;
        uint64_t free_sectors = fre_clust * fs->csize;

        sd_total_bytes = total_sectors * CONFIG_WL_SECTOR_SIZE;
        sd_free_bytes = free_sectors * CONFIG_WL_SECTOR_SIZE;
    }
}

// Draw button labels at bottom of screen
static void draw_button_labels(ssd1306_handle_t dev, const char *left, const char *middle, const char *right)
{
    const int y_pos = 52;  // Bottom of screen (64 - 12)

    // Left button
    if (left && strlen(left) > 0) {
        char label[16];
        snprintf(label, sizeof(label), "[%s]", left);
        ssd1306_draw_string(dev, 0, y_pos, (const uint8_t *)label, 12, 1);
    } else {
        ssd1306_draw_string(dev, 0, y_pos, (const uint8_t *)"[]", 12, 1);
    }

    // Middle button
    int middle_x = 40;  // Approximate center
    if (middle && strlen(middle) > 0) {
        char label[16];
        snprintf(label, sizeof(label), "[%s]", middle);
        ssd1306_draw_string(dev, middle_x, y_pos, (const uint8_t *)label, 12, 1);
    } else {
        ssd1306_draw_string(dev, middle_x, y_pos, (const uint8_t *)"[]", 12, 1);
    }

    // Right button
    if (right && strlen(right) > 0) {
        char label[16];
        snprintf(label, sizeof(label), "[%s]", right);
        ssd1306_draw_string(dev, 104, y_pos, (const uint8_t *)label, 12, 1);
    } else {
        ssd1306_draw_string(dev, 104, y_pos, (const uint8_t *)"[]", 12, 1);
    }
}

// Bluetooth GAP callback - handles device discovery events
static void bt_gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_DISC_RES_EVT:
        // Device discovered during scan
        if (bt_device_count >= MAX_BT_DEVICES) {
            break;  // List full
        }

        // Check for duplicates
        bool is_duplicate = false;
        for (int i = 0; i < bt_device_count; i++) {
            if (memcmp(bt_devices[i].bda, param->disc_res.bda, ESP_BD_ADDR_LEN) == 0) {
                is_duplicate = true;
                break;
            }
        }

        if (is_duplicate) {
            break;  // Already in list
        }

        ESP_LOGI(TAG, "New device discovered");

        // Store device address
        memcpy(bt_devices[bt_device_count].bda, param->disc_res.bda, ESP_BD_ADDR_LEN);

        // Use placeholder until name arrives
        snprintf(bt_devices[bt_device_count].name, BT_DEVICE_NAME_LEN, "Device %d", bt_device_count + 1);

        // Request device name - will update via ESP_BT_GAP_READ_REMOTE_NAME_EVT
        esp_bt_gap_read_remote_name(param->disc_res.bda);

        ESP_LOGI(TAG, "Found device: %s (requesting name...)", bt_devices[bt_device_count].name);
        bt_device_count++;
        bt_devices_updated = true;  // Signal main loop to refresh screen
        break;

    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
        // Discovery state changed
        if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
            ESP_LOGI(TAG, "Discovery stopped, found %d devices", bt_device_count);
            if (bt_state == BT_STATE_SCANNING) {
                // Keep in scanning state to show results
            }
        } else if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED) {
            ESP_LOGI(TAG, "Discovery started");
        }
        break;

    case ESP_BT_GAP_READ_REMOTE_NAME_EVT:
        // Remote device name received
        if (param->read_rmt_name.stat == ESP_BT_STATUS_SUCCESS) {
            // Find device in our list by address
            for (int i = 0; i < bt_device_count; i++) {
                if (memcmp(bt_devices[i].bda, param->read_rmt_name.bda, ESP_BD_ADDR_LEN) == 0) {
                    // Update device name
                    strncpy(bt_devices[i].name, (char *)param->read_rmt_name.rmt_name, BT_DEVICE_NAME_LEN - 1);
                    bt_devices[i].name[BT_DEVICE_NAME_LEN - 1] = '\0';
                    ESP_LOGI(TAG, "Updated device name: %s", bt_devices[i].name);
                    bt_devices_updated = true;  // Trigger screen refresh
                    break;
                }
            }
        }
        break;

    case ESP_BT_GAP_AUTH_CMPL_EVT:
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Authentication success with %s", param->auth_cmpl.device_name);
        } else {
            ESP_LOGE(TAG, "Authentication failed, status: %d", param->auth_cmpl.stat);
        }
        break;

    case ESP_BT_GAP_PIN_REQ_EVT:
        ESP_LOGI(TAG, "PIN request - auto accepting");
        esp_bt_gap_pin_reply(param->pin_req.bda, true, 0, NULL);
        break;

    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(TAG, "SSP confirm request - auto accepting (passkey: %lu)", param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;

    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(TAG, "SSP passkey notification: %lu", param->key_notif.passkey);
        break;

    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(TAG, "SSP passkey request");
        break;

    default:
        ESP_LOGI(TAG, "GAP event: %d", event);
        break;
    }
}

// Start Bluetooth device discovery
static void bt_start_discovery(void)
{
    ESP_LOGI(TAG, "Starting Bluetooth discovery");

    // Clear previous results
    bt_device_count = 0;
    bt_selected_device = 0;
    bt_scroll_offset = 0;
    memset(bt_devices, 0, sizeof(bt_devices));

    // Start discovery (scan for 10 seconds)
    esp_err_t ret = esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start discovery: %s", esp_err_to_name(ret));
        bt_state = BT_STATE_IDLE;
    } else {
        bt_state = BT_STATE_SCANNING;
    }
}

// A2DP callback - handles connection state changes (SOURCE mode)
static void bt_a2dp_callback(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT:
        if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
            ESP_LOGI(TAG, "A2DP connected successfully!");
            bt_state = BT_STATE_CONNECTED;
            memcpy(bt_connected_bda, param->conn_stat.remote_bda, ESP_BD_ADDR_LEN);
            bt_devices_updated = true;
        } else if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
            ESP_LOGI(TAG, "A2DP disconnected");
            bt_state = BT_STATE_IDLE;
            memset(bt_connected_device_name, 0, sizeof(bt_connected_device_name));
            memset(bt_connected_bda, 0, sizeof(bt_connected_bda));
            bt_devices_updated = true;
        } else if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTING) {
            ESP_LOGI(TAG, "A2DP connecting...");
        } else if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTING) {
            ESP_LOGI(TAG, "A2DP disconnecting...");
        }
        break;

    case ESP_A2D_AUDIO_STATE_EVT:
        ESP_LOGI(TAG, "A2DP audio state: %d", param->audio_stat.state);
        break;

    case ESP_A2D_AUDIO_CFG_EVT:
        ESP_LOGI(TAG, "A2DP audio config: sample_rate=%d",
                 param->audio_cfg.mcc.type == ESP_A2D_MCT_SBC ?
                 param->audio_cfg.mcc.cie.sbc[0] : 0);
        break;

    case ESP_A2D_MEDIA_CTRL_ACK_EVT:
        ESP_LOGI(TAG, "A2DP media control ACK: cmd=%d, status=%d",
                 param->media_ctrl_stat.cmd, param->media_ctrl_stat.status);
        break;

    default:
        ESP_LOGI(TAG, "A2DP event: %d", event);
        break;
    }
}

// A2DP data callback - called when headphones request audio data
static int32_t bt_a2dp_data_callback(uint8_t *data, int32_t len)
{
    // For now, just send silence (zeros) to establish connection
    // Later in Phase 4, we'll read from WAV file and send actual audio
    if (data == NULL || len < 0) {
        return 0;
    }

    memset(data, 0, len);
    return len;
}

// Connect to a discovered Bluetooth device
static void bt_connect_device(int device_idx)
{
    if (device_idx < 0 || device_idx >= bt_device_count) {
        ESP_LOGE(TAG, "Invalid device index: %d", device_idx);
        return;
    }

    ESP_LOGI(TAG, "Attempting to connect to: %s", bt_devices[device_idx].name);

    // Save device name for UI display
    strncpy(bt_connected_device_name, bt_devices[device_idx].name, BT_DEVICE_NAME_LEN - 1);
    bt_connected_device_name[BT_DEVICE_NAME_LEN - 1] = '\0';

    // Update state to show we're pairing
    bt_state = BT_STATE_PAIRING;
    bt_devices_updated = true;

    // Connect A2DP to the device (SOURCE mode - we send audio to headphones)
    esp_err_t ret = esp_a2d_source_connect(bt_devices[device_idx].bda);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initiate A2DP connection: %s", esp_err_to_name(ret));
        bt_state = BT_STATE_IDLE;
        bt_devices_updated = true;
    }
}

// Disconnect from current Bluetooth device
static void bt_disconnect(void)
{
    if (bt_state != BT_STATE_CONNECTED) {
        ESP_LOGW(TAG, "Not connected to any device");
        return;
    }

    ESP_LOGI(TAG, "Disconnecting from: %s", bt_connected_device_name);
    bt_state = BT_STATE_DISCONNECTING;
    bt_devices_updated = true;

    esp_err_t ret = esp_a2d_source_disconnect(bt_connected_bda);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disconnect: %s", esp_err_to_name(ret));
        // Force state back to idle on error
        bt_state = BT_STATE_IDLE;
        memset(bt_connected_device_name, 0, sizeof(bt_connected_device_name));
        bt_devices_updated = true;
    }
}

// AVRCP controller callback (we are the controller, headphones are target)
static void bt_avrc_ct_callback(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param)
{
    switch (event) {
    case ESP_AVRC_CT_CONNECTION_STATE_EVT:
        ESP_LOGI(TAG, "AVRCP CT connection state: %d", param->conn_stat.connected);
        break;
    case ESP_AVRC_CT_PASSTHROUGH_RSP_EVT:
        ESP_LOGI(TAG, "AVRCP passthrough response");
        break;
    case ESP_AVRC_CT_METADATA_RSP_EVT:
        ESP_LOGI(TAG, "AVRCP metadata response");
        break;
    default:
        ESP_LOGI(TAG, "AVRCP CT event: %d", event);
        break;
    }
}

// AVRCP target callback (for remote control from headphones)
static void bt_avrc_tg_callback(esp_avrc_tg_cb_event_t event, esp_avrc_tg_cb_param_t *param)
{
    switch (event) {
    case ESP_AVRC_TG_CONNECTION_STATE_EVT:
        ESP_LOGI(TAG, "AVRCP TG connection state: %d", param->conn_stat.connected);
        break;
    case ESP_AVRC_TG_PASSTHROUGH_CMD_EVT:
        ESP_LOGI(TAG, "AVRCP passthrough command: key %d, state %d",
                 param->psth_cmd.key_code, param->psth_cmd.key_state);
        break;
    default:
        ESP_LOGI(TAG, "AVRCP TG event: %d", event);
        break;
    }
}

// Initialize Bluetooth Classic
static esp_err_t bt_init(void)
{
    ESP_LOGI(TAG, "Initializing Bluetooth");

    // Release BLE memory (we only need Classic)
    esp_err_t ret = esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "BLE memory release failed: %s", esp_err_to_name(ret));
    }

    // Initialize BT controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BT controller init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Enable BT controller in Classic mode
    ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BT controller enable failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize Bluedroid stack
    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set device name
    esp_bt_gap_set_device_name("ImWearingAWire");

    // Register GAP callback
    ret = esp_bt_gap_register_callback(bt_gap_callback);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GAP callback register failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set discoverable and connectable mode
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

    // Configure authentication - set to no bonding, no MITM (simpler pairing)
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
    esp_bt_gap_set_security_param(ESP_BT_SP_IOCAP_MODE, &iocap, sizeof(uint8_t));

    // Enable SSP (Secure Simple Pairing)
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);

    // Initialize AVRCP FIRST (required before A2DP)
    // AVRCP controller (we control the headphones)
    ret = esp_avrc_ct_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "AVRCP CT init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_avrc_ct_register_callback(bt_avrc_ct_callback);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "AVRCP CT callback register failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // AVRCP target (headphones can send commands to us)
    ret = esp_avrc_tg_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "AVRCP TG init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_avrc_tg_register_callback(bt_avrc_tg_callback);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "AVRCP TG callback register failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize A2DP SOURCE AFTER AVRCP (we send audio to headphones)
    ret = esp_a2d_register_callback(bt_a2dp_callback);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "A2DP callback register failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_a2d_source_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "A2DP source init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Register A2DP data callback for sending audio
    esp_a2d_source_register_data_callback(bt_a2dp_data_callback);

    ESP_LOGI(TAG, "Bluetooth initialized successfully (A2DP SOURCE mode)");
    return ESP_OK;
}

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        return err;
    }

    return i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}

static void button_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BTN_LEFT_GPIO) | (1ULL << BTN_MIDDLE_GPIO) | (1ULL << BTN_RIGHT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    ESP_LOGI(TAG, "Buttons initialized on GPIO %d, %d, %d", BTN_LEFT_GPIO, BTN_MIDDLE_GPIO, BTN_RIGHT_GPIO);
}

static bool is_debounced(void)
{
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (now - last_button_time > BTN_DEBOUNCE_MS) {
        last_button_time = now;
        return true;
    }
    return false;
}

static void sd_card_mount(void)
{
    ESP_LOGI(TAG, "Mounting SD card");

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 0  // Use card's native cluster size
    };

    sdmmc_card_t *card;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.max_freq_khz = 4000;  // 4MHz - reduce if you get errors on breadboard

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SD_MOSI_GPIO,
        .miso_io_num = SD_MISO_GPIO,
        .sclk_io_num = SD_CLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    esp_err_t ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_CS_GPIO;
    slot_config.host_id = host.slot;

    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SD card: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "SD card mounted successfully");
    sd_initialized = true;
}

static bool write_wav_header(FILE *f, uint32_t data_size)
{
    wav_header_t header = {
        .riff = {'R', 'I', 'F', 'F'},
        .file_size = data_size + WAV_HEADER_SIZE - 8,
        .wave = {'W', 'A', 'V', 'E'},
        .fmt = {'f', 'm', 't', ' '},
        .fmt_size = 16,
        .audio_format = 1,  // PCM
        .num_channels = I2S_CHANNELS,
        .sample_rate = I2S_SAMPLE_RATE,
        .byte_rate = I2S_SAMPLE_RATE * I2S_CHANNELS * (I2S_BITS_PER_SAMPLE / 8),
        .block_align = I2S_CHANNELS * (I2S_BITS_PER_SAMPLE / 8),
        .bits_per_sample = I2S_BITS_PER_SAMPLE,
        .data = {'d', 'a', 't', 'a'},
        .data_size = data_size
    };
    size_t written = fwrite(&header, sizeof(header), 1, f);
    return (written == 1);
}

static esp_err_t i2s_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    esp_err_t ret = i2s_new_channel(&chan_cfg, NULL, &rx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2S channel");
        return ret;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(I2S_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_SCK_GPIO,
            .ws = I2S_WS_GPIO,
            .dout = I2S_GPIO_UNUSED,
            .din = I2S_SD_GPIO,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    ret = i2s_channel_init_std_mode(rx_handle, &std_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init I2S standard mode");
        return ret;
    }

    // Don't enable yet - will enable when recording starts
    ESP_LOGI(TAG, "I2S microphone initialized (disabled for power saving)");
    return ESP_OK;
}

static void start_recording(void)
{
    if (is_recording) {
        ESP_LOGW(TAG, "Already recording");
        return;
    }

    if (!sd_initialized) {
        ESP_LOGW(TAG, "Cannot record - SD card not mounted");
        return;
    }

    // Initialize I2S on first use
    if (rx_handle == NULL) {
        ESP_LOGI(TAG, "Initializing I2S microphone...");
        esp_err_t ret = i2s_init();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "I2S init failed: %s", esp_err_to_name(ret));
            return;
        }
    }

    // Enable I2S channel for recording
    esp_err_t ret = i2s_channel_enable(rx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable I2S: %s", esp_err_to_name(ret));
        return;
    }

    // Generate unique filename
    snprintf(current_filename, sizeof(current_filename),
             MOUNT_POINT "/MEMO_%03lu.WAV", recording_counter);

    recording_file = fopen(current_filename, "wb");
    if (recording_file == NULL) {
        ESP_LOGE(TAG, "Failed to open recording file");
        return;
    }

    // Write placeholder header (will update on stop)
    if (!write_wav_header(recording_file, 0)) {
        ESP_LOGE(TAG, "Failed to write WAV header");
        fclose(recording_file);
        recording_file = NULL;
        return;
    }

    is_recording = true;
    recorded_samples = 0;
    recording_start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    ESP_LOGI(TAG, "Recording started");
}

static void stop_recording(void)
{
    if (!is_recording) {
        return;
    }

    is_recording = false;

    // Update WAV header with actual data size
    uint32_t data_size = recorded_samples * (I2S_BITS_PER_SAMPLE / 8);
    fseek(recording_file, 0, SEEK_SET);
    write_wav_header(recording_file, data_size);

    fclose(recording_file);
    recording_file = NULL;

    // Disable I2S to save power
    if (rx_handle != NULL) {
        i2s_channel_disable(rx_handle);
        ESP_LOGI(TAG, "I2S disabled for power saving");
    }

    ESP_LOGI(TAG, "Recording stopped. %lu samples recorded to %s", recorded_samples, current_filename);
}

// Screen 1: RECORD
static void draw_screen_record(ssd1306_handle_t dev)
{
    ssd1306_clear_screen(dev, 0x00);

    if (is_recording) {
        // Title
        ssd1306_draw_string(dev, 5, 0, (const uint8_t *)"RECORDING...", 12, 1);

        // Elapsed time
        uint32_t elapsed_ms = (xTaskGetTickCount() * portTICK_PERIOD_MS) - recording_start_time;
        uint32_t elapsed_sec = elapsed_ms / 1000;
        char time_str[16];
        snprintf(time_str, sizeof(time_str), "  %02lu:%02lu", elapsed_sec / 60, elapsed_sec % 60);
        ssd1306_draw_string(dev, 25, 16, (const uint8_t *)time_str, 16, 1);

        // Current filename
        ssd1306_draw_string(dev, 5, 36, (const uint8_t *)current_filename + strlen(MOUNT_POINT) + 1, 12, 1);

        // Button labels
        draw_button_labels(dev, "<", "STOP", ">");
    } else {
        // Title
        ssd1306_draw_string(dev, 5, 0, (const uint8_t *)"RECORD MEMO", 12, 1);

        // Instructions
        ssd1306_draw_string(dev, 5, 16, (const uint8_t *)"Ready", 12, 1);

        // Next filename
        char next_file[32];
        snprintf(next_file, sizeof(next_file), "Next: MEMO_%03lu.WAV", recording_counter);
        ssd1306_draw_string(dev, 5, 28, (const uint8_t *)next_file, 12, 1);

        // Button labels
        draw_button_labels(dev, "<", "REC", ">");
    }
}

// Screen 2: PLAYBACK (with two modes)
static void draw_screen_playback(ssd1306_handle_t dev)
{
    ssd1306_clear_screen(dev, 0x00);

    if (!playback_browse_mode) {
        // Mode 1: Screen navigation
        char title[24];
        snprintf(title, sizeof(title), ">FILES (%d)", file_count);
        ssd1306_draw_string(dev, 5, 0, (const uint8_t *)title, 12, 1);

        // Show up to 3 files (no cursor) - tight spacing
        for (int i = 0; i < 3 && i < file_count; i++) {
            ssd1306_draw_string(dev, 5, 14 + (i * 12), (const uint8_t *)file_list[i], 12, 1);
        }

        // Button labels
        if (file_count > 0) {
            draw_button_labels(dev, "<", "Select", ">");
        } else {
            draw_button_labels(dev, "<", "", ">");
        }
    } else {
        // Mode 2: List browsing with scrolling
        char title[24];
        snprintf(title, sizeof(title), "FILES (%d)", file_count);
        ssd1306_draw_string(dev, 5, 0, (const uint8_t *)title, 12, 1);

        // Show 3 items starting from scroll_offset
        // Items: 0="Back", 1=file_list[0], 2=file_list[1], etc.
        int total_items = file_count + 1;  // +1 for "Back"

        for (int i = 0; i < 3; i++) {
            int item_index = scroll_offset + i;
            if (item_index >= total_items) break;  // No more items

            int y_pos = 14 + (i * 12);
            bool is_selected = (item_index == selected_file_index);

            if (item_index == 0) {
                // "Back" item
                if (is_selected) {
                    ssd1306_draw_string(dev, 5, y_pos, (const uint8_t *)">Back", 12, 1);
                } else {
                    ssd1306_draw_string(dev, 8, y_pos, (const uint8_t *)"Back", 12, 1);
                }
            } else {
                // File item
                int file_idx = item_index - 1;  // Convert to file_list index
                if (is_selected) {
                    char cursor_line[MAX_FILENAME_LEN + 2];
                    snprintf(cursor_line, sizeof(cursor_line), ">%s", file_list[file_idx]);
                    ssd1306_draw_string(dev, 5, y_pos, (const uint8_t *)cursor_line, 12, 1);
                } else {
                    ssd1306_draw_string(dev, 8, y_pos, (const uint8_t *)file_list[file_idx], 12, 1);
                }
            }
        }

        // Button labels
        draw_button_labels(dev, "v", "Play", "^");
    }
}

// Screen 3: BLUETOOTH (Phase 4 - Dynamic UI)
static void draw_screen_bluetooth(ssd1306_handle_t dev)
{
    ssd1306_clear_screen(dev, 0x00);

    switch (bt_state) {
        case BT_STATE_IDLE:
            // Not paired, not scanning
            ssd1306_draw_string(dev, 10, 0, (const uint8_t *)"BLUETOOTH", 12, 1);
            ssd1306_draw_string(dev, 5, 16, (const uint8_t *)"Not paired", 12, 1);
            draw_button_labels(dev, "<", "PAIR", ">");
            break;

        case BT_STATE_SCANNING:
            // Discovering devices - show scrolling list
            if (bt_device_count == 0) {
                // Still scanning, no devices yet
                ssd1306_draw_string(dev, 5, 0, (const uint8_t *)"BLUETOOTH", 12, 1);
                ssd1306_draw_string(dev, 5, 16, (const uint8_t *)"Scanning...", 12, 1);
                draw_button_labels(dev, "<", "", ">");
            } else {
                // Show device list with scrolling (including "Cancel" option)
                char title[24];
                snprintf(title, sizeof(title), "BT (%d found)", bt_device_count);
                ssd1306_draw_string(dev, 5, 0, (const uint8_t *)title, 12, 1);

                // Show 3 items at a time
                // Item 0 = "Cancel", Items 1+ = devices
                int total_items = bt_device_count + 1;  // +1 for "Cancel"

                for (int i = 0; i < 3; i++) {
                    int item_index = bt_scroll_offset + i;
                    if (item_index >= total_items) break;  // No more items

                    int y_pos = 14 + (i * 12);
                    bool is_selected = (item_index == bt_selected_device);

                    if (item_index == 0) {
                        // "Cancel" option
                        if (is_selected) {
                            ssd1306_draw_string(dev, 5, y_pos, (const uint8_t *)">Cancel", 12, 1);
                        } else {
                            ssd1306_draw_string(dev, 8, y_pos, (const uint8_t *)"Cancel", 12, 1);
                        }
                    } else {
                        // Device item
                        int device_idx = item_index - 1;  // Convert to bt_devices index
                        if (is_selected) {
                            char cursor_line[BT_DEVICE_NAME_LEN + 2];
                            snprintf(cursor_line, sizeof(cursor_line), ">%.13s", bt_devices[device_idx].name);
                            ssd1306_draw_string(dev, 5, y_pos, (const uint8_t *)cursor_line, 12, 1);
                        } else {
                            char device_line[BT_DEVICE_NAME_LEN];
                            snprintf(device_line, sizeof(device_line), " %.13s", bt_devices[device_idx].name);
                            ssd1306_draw_string(dev, 5, y_pos, (const uint8_t *)device_line, 12, 1);
                        }
                    }
                }

                // Button labels
                draw_button_labels(dev, "v", "Select", "^");
            }
            break;

        case BT_STATE_PAIRING:
            // Attempting to pair with device
            ssd1306_draw_string(dev, 10, 0, (const uint8_t *)"BLUETOOTH", 12, 1);
            ssd1306_draw_string(dev, 5, 16, (const uint8_t *)"Connecting...", 12, 1);

            // Show device name being paired
            if (strlen(bt_connected_device_name) > 0) {
                char device_name[BT_DEVICE_NAME_LEN];
                snprintf(device_name, sizeof(device_name), "%.16s", bt_connected_device_name);
                ssd1306_draw_string(dev, 5, 28, (const uint8_t *)device_name, 12, 1);
            }

            draw_button_labels(dev, "", "", "");
            break;

        case BT_STATE_CONNECTED:
            // Successfully connected to device
            ssd1306_draw_string(dev, 10, 0, (const uint8_t *)"BLUETOOTH", 12, 1);
            ssd1306_draw_string(dev, 5, 16, (const uint8_t *)"CONNECTED", 12, 1);

            // Show connected device name
            if (strlen(bt_connected_device_name) > 0) {
                char device_name[BT_DEVICE_NAME_LEN];
                snprintf(device_name, sizeof(device_name), "%.16s", bt_connected_device_name);
                ssd1306_draw_string(dev, 5, 28, (const uint8_t *)device_name, 12, 1);
            }

            draw_button_labels(dev, "<", "DISC", ">");
            break;

        case BT_STATE_DISCONNECTING:
            // Disconnecting from device
            ssd1306_draw_string(dev, 10, 0, (const uint8_t *)"BLUETOOTH", 12, 1);
            ssd1306_draw_string(dev, 5, 16, (const uint8_t *)"Disconnecting...", 12, 1);

            // Show device name being disconnected
            if (strlen(bt_connected_device_name) > 0) {
                char device_name[BT_DEVICE_NAME_LEN];
                snprintf(device_name, sizeof(device_name), "%.16s", bt_connected_device_name);
                ssd1306_draw_string(dev, 5, 28, (const uint8_t *)device_name, 12, 1);
            }

            draw_button_labels(dev, "", "", "");
            break;
    }
}

// Screen 4: SYSTEM
static void draw_screen_system(ssd1306_handle_t dev)
{
    ssd1306_clear_screen(dev, 0x00);

    // Title
    ssd1306_draw_string(dev, 5, 0, (const uint8_t *)"SYSTEM INFO", 12, 1);

    // SD card status
    if (sd_initialized) {
        char sd_status[32];
        snprintf(sd_status, sizeof(sd_status), "SD: OK");
        ssd1306_draw_string(dev, 5, 14, (const uint8_t *)sd_status, 12, 1);

        char files_str[32];
        snprintf(files_str, sizeof(files_str), "Files: %d", file_count);
        ssd1306_draw_string(dev, 5, 26, (const uint8_t *)files_str, 12, 1);

        // Show free space in MB or GB
        if (sd_free_bytes > 0) {
            char free_str[32];
            if (sd_free_bytes > 1024*1024*1024) {
                snprintf(free_str, sizeof(free_str), "Free: %.1f GB", sd_free_bytes / (1024.0*1024.0*1024.0));
            } else {
                snprintf(free_str, sizeof(free_str), "Free: %.0f MB", sd_free_bytes / (1024.0*1024.0));
            }
            ssd1306_draw_string(dev, 5, 38, (const uint8_t *)free_str, 12, 1);
        }
    } else {
        ssd1306_draw_string(dev, 5, 14, (const uint8_t *)"SD: Not mounted", 12, 1);
    }

    // Button labels
    draw_button_labels(dev, "<", "", ">");
}

static void draw_current_screen(ssd1306_handle_t dev)
{
    switch (current_screen) {
        case SCREEN_RECORD:
            draw_screen_record(dev);
            break;
        case SCREEN_PLAYBACK:
            draw_screen_playback(dev);
            break;
        case SCREEN_BLUETOOTH:
            draw_screen_bluetooth(dev);
            break;
        case SCREEN_SYSTEM:
            draw_screen_system(dev);
            break;
    }

    ssd1306_refresh_gram(dev);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting button + display navigation test");

    // Initialize NVS (required for Bluetooth)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized");

    // Initialize I2C
    ESP_LOGI(TAG, "Initializing I2C on SDA=%d, SCL=%d", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "I2C initialized successfully");

    // Initialize buttons
    button_init();

    // Initialize SSD1306 display
    ESP_LOGI(TAG, "Initializing SSD1306 OLED at address 0x%02X", OLED_I2C_ADDRESS);
    ssd1306_handle_t ssd1306_dev = ssd1306_create(I2C_MASTER_NUM, OLED_I2C_ADDRESS);
    if (ssd1306_dev == NULL) {
        ESP_LOGE(TAG, "SSD1306 creation failed");
        return;
    }

    ret = ssd1306_refresh_gram(ssd1306_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SSD1306 initialization failed: %s", esp_err_to_name(ret));
        ssd1306_delete(ssd1306_dev);
        return;
    }
    ESP_LOGI(TAG, "SSD1306 initialized successfully");

    // Mount SD card
    sd_card_mount();

    // Scan for existing files and get SD card info
    if (sd_initialized) {
        scan_sd_files();
        get_sd_card_info();
    }

    // Initialize Bluetooth
    ret = bt_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluetooth initialization failed");
    }

    // Draw initial screen
    draw_current_screen(ssd1306_dev);
    ESP_LOGI(TAG, "Ready. Use buttons to navigate.");

    // Main loop - poll buttons and update display
    while (1) {
        bool screen_changed = false;

        // Button handling - context dependent
        if (current_screen == SCREEN_PLAYBACK && playback_browse_mode) {
            // PLAYBACK BROWSE MODE - Different button behavior

            // LEFT button (v) - Move cursor down
            if (gpio_get_level(BTN_LEFT_GPIO) == 1 && is_debounced()) {
                int max_index = file_count;  // 0=Back, 1..file_count=files
                if (selected_file_index < max_index) {
                    selected_file_index++;
                    // Scroll down if cursor moves past bottom of visible window
                    if (selected_file_index >= scroll_offset + 3) {
                        scroll_offset++;
                    }
                } else {
                    selected_file_index = 0;  // Wrap to Back
                    scroll_offset = 0;  // Reset scroll to top
                }
                screen_changed = true;
                ESP_LOGI(TAG, "Playback cursor moved down to index %d", selected_file_index);
            }

            // RIGHT button (^) - Move cursor up
            if (gpio_get_level(BTN_RIGHT_GPIO) == 1 && is_debounced()) {
                if (selected_file_index > 0) {
                    selected_file_index--;
                    // Scroll up if cursor moves above top of visible window
                    if (selected_file_index < scroll_offset) {
                        scroll_offset--;
                    }
                } else {
                    selected_file_index = file_count;  // Wrap to end
                    // Scroll to show last item
                    int total_items = file_count + 1;
                    scroll_offset = (total_items > 3) ? (total_items - 3) : 0;
                }
                screen_changed = true;
                ESP_LOGI(TAG, "Playback cursor moved up to index %d", selected_file_index);
            }

            // MIDDLE button (Play) - Play file or exit browse mode
            if (gpio_get_level(BTN_MIDDLE_GPIO) == 1 && is_debounced()) {
                if (selected_file_index == 0) {
                    // "Back" selected - exit browse mode
                    playback_browse_mode = false;
                    selected_file_index = 0;
                    scroll_offset = 0;
                    screen_changed = true;
                    ESP_LOGI(TAG, "Exited playback browse mode");
                } else {
                    // File selected - play it (Phase 4)
                    ESP_LOGI(TAG, "Play file: %s (not yet implemented)", file_list[selected_file_index - 1]);
                }
            }
        } else if (current_screen == SCREEN_BLUETOOTH && bt_state == BT_STATE_SCANNING && bt_device_count > 0) {
            // BLUETOOTH SCANNING MODE - Device selection
            // Items: 0="Cancel", 1..N=devices

            // LEFT button (v) - Move cursor down
            if (gpio_get_level(BTN_LEFT_GPIO) == 1 && is_debounced()) {
                int max_index = bt_device_count;  // 0=Cancel, 1..device_count=devices
                if (bt_selected_device < max_index) {
                    bt_selected_device++;
                    // Scroll down if cursor moves past bottom of visible window
                    if (bt_selected_device >= bt_scroll_offset + 3) {
                        bt_scroll_offset++;
                    }
                } else {
                    bt_selected_device = 0;  // Wrap to Cancel
                    bt_scroll_offset = 0;  // Reset scroll to top
                }
                screen_changed = true;
                ESP_LOGI(TAG, "BT cursor moved down to index %d", bt_selected_device);
            }

            // RIGHT button (^) - Move cursor up
            if (gpio_get_level(BTN_RIGHT_GPIO) == 1 && is_debounced()) {
                if (bt_selected_device > 0) {
                    bt_selected_device--;
                    // Scroll up if cursor moves above top of visible window
                    if (bt_selected_device < bt_scroll_offset) {
                        bt_scroll_offset--;
                    }
                } else {
                    bt_selected_device = bt_device_count;  // Wrap to last device
                    // Scroll to show last item
                    int total_items = bt_device_count + 1;
                    bt_scroll_offset = (total_items > 3) ? (total_items - 3) : 0;
                }
                screen_changed = true;
                ESP_LOGI(TAG, "BT cursor moved up to index %d", bt_selected_device);
            }

            // MIDDLE button (Select) - Select device or cancel
            if (gpio_get_level(BTN_MIDDLE_GPIO) == 1 && is_debounced()) {
                if (bt_selected_device == 0) {
                    // "Cancel" selected - return to IDLE
                    bt_state = BT_STATE_IDLE;
                    bt_device_count = 0;
                    bt_selected_device = 0;
                    bt_scroll_offset = 0;
                    screen_changed = true;
                    ESP_LOGI(TAG, "Bluetooth scan cancelled, returning to IDLE");
                } else {
                    // Device selected - connect to it
                    int device_idx = bt_selected_device - 1;  // Convert to device array index
                    bt_connect_device(device_idx);
                    screen_changed = true;
                }
            }
        } else {
            // NORMAL NAVIGATION MODE - Standard button behavior

            // LEFT button (<) - Previous screen
            if (gpio_get_level(BTN_LEFT_GPIO) == 1 && is_debounced()) {
                current_screen = (current_screen == 0) ? (NUM_SCREENS - 1) : (current_screen - 1);
                screen_changed = true;
                ESP_LOGI(TAG, "Previous screen: %d", current_screen);
            }

            // RIGHT button (>) - Next screen
            if (gpio_get_level(BTN_RIGHT_GPIO) == 1 && is_debounced()) {
                current_screen = (current_screen + 1) % NUM_SCREENS;
                screen_changed = true;
                ESP_LOGI(TAG, "Next screen: %d", current_screen);
            }

            // MIDDLE button - Context-dependent action
            if (gpio_get_level(BTN_MIDDLE_GPIO) == 1 && is_debounced()) {
                if (current_screen == SCREEN_RECORD) {
                    // Record screen: Toggle recording
                    if (is_recording) {
                        stop_recording();
                        scan_sd_files();  // Refresh file list
                        get_sd_card_info();  // Update free space
                    } else {
                        start_recording();
                    }
                    screen_changed = true;
                    ESP_LOGI(TAG, "Recording: %s", is_recording ? "ON" : "OFF");
                } else if (current_screen == SCREEN_PLAYBACK && file_count > 0) {
                    // Playback screen: Enter browse mode
                    playback_browse_mode = true;
                    selected_file_index = 0;  // Start on "Back"
                    scroll_offset = 0;  // Start at top
                    screen_changed = true;
                    ESP_LOGI(TAG, "Entered playback browse mode");
                } else if (current_screen == SCREEN_BLUETOOTH) {
                    // Bluetooth screen: State-dependent actions
                    if (bt_state == BT_STATE_IDLE) {
                        // Start discovery
                        bt_start_discovery();
                        screen_changed = true;
                        ESP_LOGI(TAG, "Starting Bluetooth scan");
                    } else if (bt_state == BT_STATE_CONNECTED) {
                        // Disconnect from device
                        bt_disconnect();
                        screen_changed = true;
                    }
                }
                // SYSTEM screen: MIDDLE does nothing
            }
        }

        // Handle I2S reading while recording
        if (is_recording && recording_file != NULL) {
            int16_t buffer[I2S_BUFFER_SIZE];
            size_t bytes_read = 0;

            esp_err_t ret = i2s_channel_read(rx_handle, buffer, sizeof(buffer), &bytes_read, 10);
            if (ret == ESP_OK && bytes_read > 0) {
                // Write to file
                size_t written = fwrite(buffer, 1, bytes_read, recording_file);
                if (written != bytes_read) {
                    ESP_LOGE(TAG, "Write failed! Stopping recording");
                    stop_recording();
                    scan_sd_files();
                    continue;
                }

                // Update sample count
                recorded_samples += bytes_read / (I2S_BITS_PER_SAMPLE / 8);

                screen_changed = true;  // Update display with elapsed time
            }
        }

        // Check if Bluetooth devices were updated
        if (bt_devices_updated) {
            bt_devices_updated = false;
            screen_changed = true;
        }

        // Redraw screen if changed
        if (screen_changed) {
            draw_current_screen(ssd1306_dev);
        }

        vTaskDelay(pdMS_TO_TICKS(50));  // Poll every 50ms
    }
}

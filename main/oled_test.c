#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <sys/unistd.h>
#include <sys/stat.h>
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
#define RECORD_FILENAME             MOUNT_POINT "/test_rec.wav"

// Screen states
#define NUM_SCREENS                 6
typedef enum {
    SCREEN_TITLE = 0,
    SCREEN_SD_TEST,
    SCREEN_RECORD,
    SCREEN_PLAYBACK,
    SCREEN_SETTINGS,
    SCREEN_GRAPHICS
} screen_state_t;

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
static screen_state_t current_screen = SCREEN_TITLE;
static bool invert_mode = false;
static char sd_test_result[64] = "Not tested";
static bool sd_initialized = false;
static uint32_t last_button_time = 0;
static i2s_chan_handle_t rx_handle = NULL;
static bool is_recording = false;
static FILE *recording_file = NULL;
static uint32_t recorded_samples = 0;
static int16_t audio_level = 0;

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
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_card_t *card;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.max_freq_khz = 400;  // Slow down to 400kHz for breadboard wiring

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

static void write_wav_header(FILE *f, uint32_t data_size)
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
    fwrite(&header, sizeof(header), 1, f);
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

    ret = i2s_channel_enable(rx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable I2S channel");
        return ret;
    }

    ESP_LOGI(TAG, "I2S microphone initialized");
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
        snprintf(sd_test_result, sizeof(sd_test_result), "SD card error");
        return;
    }

    // Initialize I2S on first use
    if (rx_handle == NULL) {
        ESP_LOGI(TAG, "Initializing I2S microphone...");
        esp_err_t ret = i2s_init();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "I2S init failed: %s", esp_err_to_name(ret));
            snprintf(sd_test_result, sizeof(sd_test_result), "Mic init failed");
            return;
        }
    }

    recording_file = fopen(RECORD_FILENAME, "wb");
    if (recording_file == NULL) {
        ESP_LOGE(TAG, "Failed to open recording file");
        snprintf(sd_test_result, sizeof(sd_test_result), "File open failed");
        return;
    }

    // Write placeholder header (will update on stop)
    write_wav_header(recording_file, 0);

    is_recording = true;
    recorded_samples = 0;
    audio_level = 0;
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

    ESP_LOGI(TAG, "Recording stopped. %lu samples recorded", recorded_samples);
}

static void draw_screen_title(ssd1306_handle_t dev)
{
    ssd1306_clear_screen(dev, 0x00);
    ssd1306_draw_string(dev, 10, 16, (const uint8_t *)"I'm Wearing", 16, 1);
    ssd1306_draw_string(dev, 20, 40, (const uint8_t *)"a Wire", 16, 1);
}

static void draw_screen_sd_test(ssd1306_handle_t dev)
{
    ssd1306_clear_screen(dev, 0x00);
    ssd1306_draw_string(dev, 10, 8, (const uint8_t *)"SD CARD STATUS", 12, 1);

    if (sd_initialized) {
        ssd1306_draw_string(dev, 25, 32, (const uint8_t *)"MOUNTED", 16, 1);
        ssd1306_draw_string(dev, 35, 50, (const uint8_t *)"Ready", 8, 1);
    } else {
        ssd1306_draw_string(dev, 10, 32, (const uint8_t *)"NOT MOUNTED", 12, 1);
        if (strlen(sd_test_result) > 0) {
            ssd1306_draw_string(dev, 5, 50, (const uint8_t *)sd_test_result, 8, 1);
        }
    }
}

static void draw_screen_record(ssd1306_handle_t dev)
{
    ssd1306_clear_screen(dev, 0x00);

    if (is_recording) {
        ssd1306_draw_string(dev, 15, 8, (const uint8_t *)"RECORDING", 16, 1);

        // Show sample count
        char samples_str[32];
        snprintf(samples_str, sizeof(samples_str), "Samples: %lu", recorded_samples);
        ssd1306_draw_string(dev, 5, 28, (const uint8_t *)samples_str, 8, 1);

        // Draw audio level meter (bar)
        int bar_width = (audio_level * 100) / 32768;  // Scale to 0-100
        if (bar_width > 100) bar_width = 100;
        ssd1306_draw_string(dev, 5, 40, (const uint8_t *)"Level:", 8, 1);
        for (int x = 0; x < bar_width; x++) {
            ssd1306_draw_line(dev, 5 + x, 52, 5 + x, 60);
        }

        ssd1306_draw_string(dev, 10, 52, (const uint8_t *)"Press to stop", 8, 1);
    } else {
        ssd1306_draw_string(dev, 5, 8, (const uint8_t *)"RECORD READY", 16, 1);
        ssd1306_draw_string(dev, 10, 35, (const uint8_t *)"Press middle", 12, 1);
        ssd1306_draw_string(dev, 10, 50, (const uint8_t *)"to start", 12, 1);
    }
}

static void draw_screen_playback(ssd1306_handle_t dev)
{
    ssd1306_clear_screen(dev, 0x00);
    ssd1306_draw_string(dev, 20, 8, (const uint8_t *)"PLAYBACK", 16, 1);
    ssd1306_draw_string(dev, 5, 32, (const uint8_t *)"memo_001.wav", 12, 1);
    ssd1306_draw_string(dev, 5, 48, (const uint8_t *)"memo_002.wav", 12, 1);
}

static void draw_screen_settings(ssd1306_handle_t dev)
{
    ssd1306_clear_screen(dev, 0x00);
    ssd1306_draw_string(dev, 25, 8, (const uint8_t *)"SETTINGS", 16, 1);
    ssd1306_draw_string(dev, 5, 30, (const uint8_t *)"> Sample Rate", 12, 1);
    ssd1306_draw_string(dev, 5, 44, (const uint8_t *)"  Bluetooth", 12, 1);
}

static void draw_screen_graphics(ssd1306_handle_t dev)
{
    ssd1306_clear_screen(dev, 0x00);
    ssd1306_draw_string(dev, 20, 2, (const uint8_t *)"GRAPHICS", 12, 1);

    // Draw test rectangles
    for (int i = 0; i < 3; i++) {
        int y = 18 + (i * 14);
        ssd1306_draw_line(dev, 10, y, 50, y);
        ssd1306_draw_line(dev, 10, y + 10, 50, y + 10);
        ssd1306_draw_line(dev, 10, y, 10, y + 10);
        ssd1306_draw_line(dev, 50, y, 50, y + 10);
    }

    // Draw audio level bars
    for (int i = 0; i < 5; i++) {
        int height = 10 + (i * 8);
        int x = 60 + (i * 12);
        for (int dx = 0; dx < 8; dx++) {
            ssd1306_draw_line(dev, x + dx, 60 - height, x + dx, 60);
        }
    }
}

static void draw_current_screen(ssd1306_handle_t dev)
{
    switch (current_screen) {
        case SCREEN_TITLE:
            draw_screen_title(dev);
            break;
        case SCREEN_SD_TEST:
            draw_screen_sd_test(dev);
            break;
        case SCREEN_RECORD:
            draw_screen_record(dev);
            break;
        case SCREEN_PLAYBACK:
            draw_screen_playback(dev);
            break;
        case SCREEN_SETTINGS:
            draw_screen_settings(dev);
            break;
        case SCREEN_GRAPHICS:
            draw_screen_graphics(dev);
            break;
    }

    // Show asterisk indicator when button 3 is toggled
    if (invert_mode) {
        ssd1306_draw_string(dev, 116, 0, (const uint8_t *)"*", 8, 1);
    }

    ssd1306_refresh_gram(dev);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting button + display navigation test");

    // Initialize I2C
    ESP_LOGI(TAG, "Initializing I2C on SDA=%d, SCL=%d", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    esp_err_t ret = i2c_master_init();
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

    // Draw initial screen
    draw_current_screen(ssd1306_dev);
    ESP_LOGI(TAG, "Ready. Use buttons to navigate.");

    // Main loop - poll buttons and update display
    while (1) {
        bool screen_changed = false;

        // Check left button (previous screen)
        if (gpio_get_level(BTN_LEFT_GPIO) == 1 && is_debounced()) {
            current_screen = (current_screen == 0) ? (NUM_SCREENS - 1) : (current_screen - 1);
            screen_changed = true;
            ESP_LOGI(TAG, "Left button pressed - Screen: %d", current_screen);
        }

        // Check right button (next screen)
        if (gpio_get_level(BTN_RIGHT_GPIO) == 1 && is_debounced()) {
            current_screen = (current_screen + 1) % NUM_SCREENS;
            screen_changed = true;
            ESP_LOGI(TAG, "Right button pressed - Screen: %d", current_screen);
        }

        // Check middle button (toggle recording or invert)
        if (gpio_get_level(BTN_MIDDLE_GPIO) == 1 && is_debounced()) {
            if (current_screen == SCREEN_RECORD) {
                // Toggle recording on record screen
                if (is_recording) {
                    stop_recording();
                } else {
                    start_recording();
                }
                screen_changed = true;
                ESP_LOGI(TAG, "Middle button - Recording: %s", is_recording ? "ON" : "OFF");
            } else {
                // Toggle invert on other screens
                invert_mode = !invert_mode;
                screen_changed = true;
                ESP_LOGI(TAG, "Middle button - Invert: %s", invert_mode ? "ON" : "OFF");
            }
        }

        // Handle I2S reading while recording
        if (is_recording && recording_file != NULL) {
            int16_t buffer[I2S_BUFFER_SIZE];
            size_t bytes_read = 0;

            esp_err_t ret = i2s_channel_read(rx_handle, buffer, sizeof(buffer), &bytes_read, 10);
            if (ret == ESP_OK && bytes_read > 0) {
                // Write to file
                fwrite(buffer, 1, bytes_read, recording_file);

                // Update sample count
                recorded_samples += bytes_read / (I2S_BITS_PER_SAMPLE / 8);

                // Calculate audio level (abs average)
                int32_t sum = 0;
                int sample_count = bytes_read / sizeof(int16_t);
                for (int i = 0; i < sample_count; i++) {
                    sum += abs(buffer[i]);
                }
                audio_level = sum / sample_count;

                screen_changed = true;  // Update display with new level
            }
        }

        // Redraw screen if changed
        if (screen_changed) {
            draw_current_screen(ssd1306_dev);
        }

        vTaskDelay(pdMS_TO_TICKS(50));  // Poll every 50ms
    }
}

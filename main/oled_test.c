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

// Global state
static screen_state_t current_screen = SCREEN_TITLE;
static bool invert_mode = false;
static char sd_test_result[64] = "Not tested";
static bool sd_initialized = false;
static uint32_t last_button_time = 0;

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

static void sd_card_init_and_test(void)
{
    ESP_LOGI(TAG, "Initializing SD card");

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
        ESP_LOGE(TAG, "Failed to initialize SPI bus");
        snprintf(sd_test_result, sizeof(sd_test_result), "SPI init failed");
        return;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_CS_GPIO;
    slot_config.host_id = host.slot;

    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SD card: %s", esp_err_to_name(ret));
        snprintf(sd_test_result, sizeof(sd_test_result), "Mount failed");
        return;
    }

    ESP_LOGI(TAG, "SD card mounted successfully");
    sd_initialized = true;

    // Write test file
    const char *test_data = "Hello from ESP32!";
    const char *filepath = MOUNT_POINT "/test.txt";

    FILE *f = fopen(filepath, "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        snprintf(sd_test_result, sizeof(sd_test_result), "Write failed");
        return;
    }

    fprintf(f, "%s", test_data);
    fclose(f);
    ESP_LOGI(TAG, "File written successfully");

    // Read test file
    f = fopen(filepath, "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        snprintf(sd_test_result, sizeof(sd_test_result), "Read failed");
        return;
    }

    char read_buffer[64];
    fgets(read_buffer, sizeof(read_buffer), f);
    fclose(f);

    // Verify contents
    if (strcmp(read_buffer, test_data) == 0) {
        ESP_LOGI(TAG, "SD card test PASSED");
        snprintf(sd_test_result, sizeof(sd_test_result), "PASS: Read/Write OK");
    } else {
        ESP_LOGE(TAG, "SD card test FAILED - data mismatch");
        snprintf(sd_test_result, sizeof(sd_test_result), "FAIL: Data mismatch");
    }
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
    ssd1306_draw_string(dev, 20, 8, (const uint8_t *)"SD CARD TEST", 16, 1);
    ssd1306_draw_string(dev, 5, 32, (const uint8_t *)sd_test_result, 12, 1);
    if (sd_initialized) {
        ssd1306_draw_string(dev, 5, 50, (const uint8_t *)"test.txt written", 8, 1);
    }
}

static void draw_screen_record(ssd1306_handle_t dev)
{
    ssd1306_clear_screen(dev, 0x00);
    ssd1306_draw_string(dev, 5, 8, (const uint8_t *)"RECORD READY", 16, 1);
    ssd1306_draw_string(dev, 30, 32, (const uint8_t *)"00:00", 16, 1);
    ssd1306_draw_string(dev, 20, 52, (const uint8_t *)"Press to rec", 8, 1);
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

    // Initialize and test SD card
    sd_card_init_and_test();

    // Draw initial screen
    draw_current_screen(ssd1306_dev);
    ESP_LOGI(TAG, "Navigation test ready. Use buttons to navigate.");

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

        // Check middle button (toggle invert)
        if (gpio_get_level(BTN_MIDDLE_GPIO) == 1 && is_debounced()) {
            invert_mode = !invert_mode;
            screen_changed = true;
            ESP_LOGI(TAG, "Middle button pressed - Invert: %s", invert_mode ? "ON" : "OFF");
        }

        // Redraw screen if changed
        if (screen_changed) {
            draw_current_screen(ssd1306_dev);
        }

        vTaskDelay(pdMS_TO_TICKS(50));  // Poll every 50ms
    }
}

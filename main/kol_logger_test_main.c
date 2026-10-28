#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include <string.h>

#include "esp_random.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include <stdbool.h>

#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include "driver/gpio.h"

#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "esp_http_server.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_ili9341.h"

#include "driver/spi_master.h"
#include "esp_netif.h"

#include "esp_timer.h"

// using ESP32-S3 SPI2 hardware peripheral FOR lCD and SD card
#define SPI_HOST SPI2_HOST
#define LCD_PIXEL_CLOCK_HZ (10 * 1000 * 1000)

// GPIO
#define PIN_NUM_MOSI GPIO_NUM_11
#define PIN_NUM_MISO GPIO_NUM_13
#define PIN_NUM_SCLK GPIO_NUM_12
#define PIN_NUM_LCD_CS GPIO_NUM_10
#define PIN_NUM_SD_CS GPIO_NUM_15
#define PIN_NUM_DC GPIO_NUM_9
#define PIN_NUM_RST GPIO_NUM_14

// native resolution of the ILI9341 LCD
#define LCD_H_RES 320
#define LCD_V_RES 240

// graph dimensions and location on the LCD screen
#define GRAPH_X 48
#define GRAPH_Y 83
#define GRAPH_W 263
#define GRAPH_H 125

#define GRAPH_HISTORY (5LL * 60 * 1000) // want to display the latest 5 minutes of measurements, so GRAPH_HISTORY is 5 minutes in milliseconds
#define GRAPH_BUFFER_SIZE 330           // how many measurements to keep in RAM, slightly more than 5 minutes

// current threshhold
typedef struct
{
    uint32_t after_seconds;
    float limit_ma;
} limit_stage_t;

static const limit_stage_t limit_stages[] =
    {
        {0, 2500.0f}, // 2.5 A
        {60, 200.0f}, // 200 mA
        {100, 20.0f}  // 20 mA
};

// glyphs for display
typedef struct
{
    char character;
    uint8_t columns[5];
} glyph_t;

static const glyph_t font5x7[] = {
    {' ', {0x00, 0x00, 0x00, 0x00, 0x00}},
    {'.', {0x00, 0x60, 0x60, 0x00, 0x00}},
    {':', {0x00, 0x36, 0x36, 0x00, 0x00}},
    {'-', {0x08, 0x08, 0x08, 0x08, 0x08}},

    {'0', {0x3E, 0x51, 0x49, 0x45, 0x3E}},
    {'1', {0x00, 0x42, 0x7F, 0x40, 0x00}},
    {'2', {0x42, 0x61, 0x51, 0x49, 0x46}},
    {'3', {0x21, 0x41, 0x45, 0x4B, 0x31}},
    {'4', {0x18, 0x14, 0x12, 0x7F, 0x10}},
    {'5', {0x27, 0x45, 0x45, 0x45, 0x39}},
    {'6', {0x3C, 0x4A, 0x49, 0x49, 0x30}},
    {'7', {0x01, 0x71, 0x09, 0x05, 0x03}},
    {'8', {0x36, 0x49, 0x49, 0x49, 0x36}},
    {'9', {0x06, 0x49, 0x49, 0x29, 0x1E}},

    {'A', {0x7E, 0x11, 0x11, 0x11, 0x7E}},
    {'B', {0x7F, 0x49, 0x49, 0x49, 0x36}},
    {'C', {0x3E, 0x41, 0x41, 0x41, 0x22}},
    {'E', {0x7F, 0x49, 0x49, 0x49, 0x41}},
    {'I', {0x00, 0x41, 0x7F, 0x41, 0x00}},
    {'L', {0x7F, 0x40, 0x40, 0x40, 0x40}},
    {'M', {0x7F, 0x02, 0x0C, 0x02, 0x7F}},
    {'N', {0x7F, 0x04, 0x08, 0x10, 0x7F}},
    {'O', {0x3E, 0x41, 0x41, 0x41, 0x3E}},
    {'R', {0x7F, 0x09, 0x19, 0x29, 0x46}},
    {'T', {0x01, 0x01, 0x7F, 0x01, 0x01}},
    {'U', {0x3F, 0x40, 0x40, 0x40, 0x3F}},
    {'V', {0x1F, 0x20, 0x40, 0x20, 0x1F}},
    {'W', {0x3F, 0x40, 0x38, 0x40, 0x3F}},
    {'m', {0x7C, 0x04, 0x18, 0x04, 0x78}}
};

// configuration macros for WiFi settings, defined in project configuration
#define ESP_WIFI_SSID CONFIG_ESP_WIFI_SSID // preprocessor macro for WiFi SSID from project configuration, associate ESP_WIFI_SSID with CONFIG_ESP_WIFI_SSID
#define ESP_WIFI_PASS CONFIG_ESP_WIFI_PASSWORD
#define ESP_WIFI_CHANNEL CONFIG_ESP_WIFI_CHANNEL
#define MAX_STA_CONN CONFIG_ESP_MAX_STA_CONN

#if CONFIG_ESP_GTK_REKEYING_ENABLE
#define GTK_REKEY_INTERVAL CONFIG_ESP_GTK_REKEY_INTERVAL
#else
#define GTK_REKEY_INTERVAL 0
#endif

#define SD_MOUNT_POINT "/sdcard"
#define SD_MAX_LOG_LINES 200

// button
#define PIN_NUM_RESET_BUTTON GPIO_NUM_4
static TaskHandle_t reset_button_task_handle = NULL;

static sdmmc_card_t *sd_card = NULL;
static FILE *sd_log_file = NULL;

static uint32_t sd_line_number = 1;

static bool sd_logging_active = false;

// initialise fake measurements for testing
typedef struct
{
    float current;        // current draw in mA
    int64_t timestamp_ms; // timestamp in milli seconds since the start of the program
} measurement_t;

static measurement_t latest_measurement = {
    .current = 100.0f,
    .timestamp_ms = 0};

static uint32_t session_start_ms = 0; // this variable will hold the timestamp of when the current session started

// mutex to protect access to latest_measurement
static SemaphoreHandle_t measurement_mutex;

// pointer to queue used by scheduler to hold the fake measurements (this is equivalent to static struct QueueDefinition* measurement_queue)
static QueueHandle_t measurement_queue;

// array to hold the last GRAPH_W measurements for graphing
static measurement_t samples[GRAPH_BUFFER_SIZE];

// counter to keep track of how many samples are in the samples array
static int sample_count = 0;

// flag set by debounced button press, handled by graph task
static volatile bool reset_requested = false;

// for logging
static const char *LCD_TAG = "LCD";
static const char *WIFI_TAG = "wifi softAP";
static const char *SD_TAG = "SD";

/*  GPIO ISR, need to keep this short
    Place function in RAM (IRAM_ATTR)   */
static void IRAM_ATTR reset_button_isr(void *arg)
{
    (void)arg; // avoid compiler warning about unused parameter

    BaseType_t higher_priority_task_woken = pdFALSE;

    // send notification to the reset_button_task with it's handle, this function is safe to use in ISR
    vTaskNotifyGiveFromISR(reset_button_task_handle, &higher_priority_task_woken);

    // if button task is more important than the current task, yield to it immediately
    if (higher_priority_task_woken == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }
}

// task to handle debouncing of button press, it waits for a notification from the GPIO ISR
static void reset_button_task(void *pvParameters)
{
    (void)pvParameters; // avoid compiler warning about unused parameter

    while (1)
    {
        // sleep until GPIO ISR sends a notification to this task
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // debounce the button press
        vTaskDelay(pdMS_TO_TICKS(30)); // wait 30 ms for button press to settle

        if (gpio_get_level(PIN_NUM_RESET_BUTTON) == 0) // check if button is still pressed (active low)
        {
            ESP_LOGI(LCD_TAG, "Reset button pressed");
            reset_requested = true;

            // wait until user releases button
            while (gpio_get_level(PIN_NUM_RESET_BUTTON) == 0)
            {
                vTaskDelay(pdMS_TO_TICKS(10)); // check every 10 ms
            }

            // debounce te release
            vTaskDelay(pdMS_TO_TICKS(30));
        }

        // clear extra notifications produnced by contact bouncing of the button
        while (ulTaskNotifyTake(pdTRUE, 0) > 0)
        {
            // do nothing
        }
    }
}

static float threshold_ma_at(uint32_t elapsed_seconds)
{
    float threshold = limit_stages[0].limit_ma;

    // dynamic way of calculating how many current limit stages there are
    size_t count = sizeof(limit_stages) / sizeof(limit_stages[0]);

    // calculate threshold for elapsed seconds
    for (size_t i = 1; i < count; i++)
    {
        if (elapsed_seconds < limit_stages[i].after_seconds)
        {
            break;
        }
        threshold = limit_stages[i].limit_ma;
    }

    return threshold;
}

// this array stores the image that is sent to the LCD. Each pixel is represented by a 16-bit value in RGB565 format.
static uint16_t frame_buffer[LCD_H_RES * LCD_V_RES];

// lcd uses 16-bit in RGB565 format colour, however, the ESP32-S3 is little-endian, so we need to swap the byte order of each pixel before sending it to the LCD
static uint16_t rgb565_be(uint16_t colour)
{
    // from GCC built-in function to swap the byte order of a 16-bit value
    return __builtin_bswap16(colour);
}

// freeRTOS task to simulate changing measurements
static void fake_measurement_task(void *pvParameters)
{
    (void)pvParameters;

    while (1)
    {
        // Generate one measurement per second
        vTaskDelay(pdMS_TO_TICKS(1000));

        if (xSemaphoreTake(measurement_mutex, portMAX_DELAY) == pdTRUE)
        {
            // Generate a random movement between -2.5 mA and +1.0 mA
            float random_change = ((int)(esp_random() % 36) - 25) / 10.0f;

            latest_measurement.current += random_change;

            // Keep maximum at 100 mA
            if (latest_measurement.current > 100.0f)
            {
                latest_measurement.current = 100.0f;
            }

            // Restart simulation after reaching zero
            if (latest_measurement.current < 0.0f)
            {
                latest_measurement.current = 100.0f;
            }

            latest_measurement.timestamp_ms = esp_timer_get_time() / 1000;
            xQueueOverwrite(measurement_queue, &latest_measurement);

            xSemaphoreGive(measurement_mutex);
        }
    }
}

// add a new measurement to the samples array, shifting the existing measurements to the left if the array is full
static void add_sample(const measurement_t *value)
{
    if (sample_count < GRAPH_BUFFER_SIZE)
    {
        // graph is not full yet
        samples[sample_count] = *value;
        sample_count++;
    }
    else
    {
        // graph is full, so shift every measurement one position to the left
        for (int i = 0; i < GRAPH_BUFFER_SIZE - 1; i++)
        {
            samples[i] = samples[i + 1];
        }
        samples[GRAPH_BUFFER_SIZE - 1] = *value;
    }
}

// SD initialisation function
static esp_err_t init_sd_card(void)
{
    ESP_LOGI(SD_TAG, "Mounting SD card...");

    // SD card host configuration
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    // using SPI2 bus
    host.slot = SPI_HOST;
    host.max_freq_khz = 4000;

    // configure SD card as a device on the SPI2 bus which is shared
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_SD_CS; // chip select pin for SD card
    slot_config.host_id = SPI_HOST;      // use SPI2 bus

    // FAT filesystem mount configuration
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,  // do not format the card if mounting fails
        .max_files = 5,                   // maximum number of files that can be open at the same time
        .allocation_unit_size = 16 * 1024 // allocation unit size in bytes (16 KB)
    };

    esp_err_t ret = esp_vfs_fat_sdspi_mount(
        SD_MOUNT_POINT, // mount point for the SD card
        &host,          // SD card host configuration
        &slot_config,   // SD card device configuration
        &mount_config,  // FAT filesystem mount configuration
        &sd_card        // pointer to the SD card object (sdmmc_card_t)
    );

    if (ret != ESP_OK)
    {
        ESP_LOGE(SD_TAG, "Failed to mount SD card: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(SD_TAG, "SD card successfully mounted at %s", SD_MOUNT_POINT);

    sdmmc_card_print_info(stdout, sd_card); // print SD card information to console

    return ESP_OK;
}

// create SD card log file
static esp_err_t create_sd_log_file(void)
{
    sd_log_file = fopen(SD_MOUNT_POINT "/V001_KOL.TXT", "w"); // w means write mode, will create the file if it does not exist, and overwrite it if it does exist

    if (sd_log_file == NULL)
    {
        ESP_LOGE(SD_TAG, "Failed to create log file on SD card");
        return ESP_FAIL;
    }

    // write file header
    fprintf(sd_log_file, "KOL V1.1\n");
    fprintf(sd_log_file, "Sample counter 1 (1.0 sec)\n");
    fprintf(sd_log_file, "\n");

    fprintf(sd_log_file,
            "%-5s %-12s %-10s %10s %12s %12s %12s %12s %6s\n",
            "LN",
            "YYYYMMDD",
            "HHMMSS",
            "V",
            "I",
            "C",
            "C+",
            "C-",
            "Ts");
    fflush(sd_log_file); // flush the file buffer to ensure the header is written to the SD card

    sd_line_number = 1; // reset line number for new log file

    sd_logging_active = true; // enable SD card logging

    ESP_LOGI(SD_TAG, "Log file created on SD card: %s/V001_KOL.TXT", SD_MOUNT_POINT);

    return ESP_OK;
}

// write measurement data (measurement_t) to SD card log file, currently only writing the current value and timestamp
static void write_sd_measurement(const measurement_t *measurement)
{
    // check if SD card logging is active
    if (!sd_logging_active || sd_log_file == NULL)
    {
        return;
    }

    int32_t elapsed_seconds = ((uint32_t)measurement->timestamp_ms - session_start_ms) / 1000; // calculate elapsed time in seconds since the start of the session

    // convert elapsed seconds to hours, minutes, seconds
    int hours = (elapsed_seconds / 3600) % 24;
    int minutes = (elapsed_seconds / 60) % 60;
    int seconds = elapsed_seconds % 60;

    float current_A = measurement->current / 1000.0f; // convert mA to A

    // Write one row
    fprintf(sd_log_file,
            "%-5lu "
            "%-12s "
            "%02lu:%02lu:%02lu "
            "%10s "
            "%12.6f "
            "%12s "
            "%12s "
            "%12s "
            "%6s\n",

            // LN
            (unsigned long)sd_line_number,

            // YYYYMMDD
            "NA",

            // HHMMSS
            (unsigned long)hours,
            (unsigned long)minutes,
            (unsigned long)seconds,

            // V
            "NA",

            // I
            current_A,

            // C
            "NA",

            // C+
            "NA",

            // C-
            "NA",

            // Ts
            "NA");
    fflush(sd_log_file); // flush the file buffer to ensure the data is written to the SD card

    ESP_LOGI(SD_TAG, "SD line %lu written - Current: %.1f mA", (unsigned long)sd_line_number, measurement->current);

    sd_line_number++; // increment line number for next measurement

    // stop after 20 measurements (for testing purposes)
    if (sd_line_number > SD_MAX_LOG_LINES)
    {
        ESP_LOGI(SD_TAG, "20 measurements written");

        // Flush anything remaining
        fflush(sd_log_file);

        // Close file
        fclose(sd_log_file);

        sd_log_file = NULL;

        // Unmount SD card
        esp_vfs_fat_sdcard_unmount(
            SD_MOUNT_POINT,
            sd_card);

        sd_card = NULL;

        sd_logging_active = false;

        ESP_LOGI(SD_TAG, "SD card unmounted");

        ESP_LOGI(SD_TAG, "Safe to remove SD card");
    }
}

// convert measurement value to a y coord, caller decides y_max (maximum current)
static int value_to_y_scaled(float current_ma, float y_max)
{
    float value = fmaxf(0.00f, fminf(current_ma, y_max)); // clamp value to the range [0, y_max]

    float normalized = value / y_max; // normalize value to the range [0, 1]

    return GRAPH_Y + GRAPH_H - 1 - ((int)(normalized * (GRAPH_H - 1))); // convert normalized value to y coordinate in the graph
}

// convert elapsed time in ms to an x coordinate in the graoh, caller decides window_start_ms (start of the revolving 5 minute graph window)
static int time_to_x(uint32_t sample_elapsed_ms, uint32_t window_start_ms)
{
    int x = GRAPH_X + (int)(((int64_t)sample_elapsed_ms - window_start_ms) * (GRAPH_W - 1) / GRAPH_HISTORY);
    return x;
}

// draw one pixel into the frame buffer at the given x,y coordinates with the given colour
static void draw_pixel(int x, int y, uint16_t colour)
{
    // check that the coordinates are inside the LCD bounds
    if (x < 0 || x >= LCD_H_RES || y < 0 || y >= LCD_V_RES)
    {
        return;
    }
    // update the frame buffer with the new pixel colour with converted x, y coordinates
    frame_buffer[y * LCD_H_RES + x] = colour;
}

// draw line between two points using Bresenham's line algorithm
static void draw_line(int x0, int y0, int x1, int y1, uint16_t colour)
{
    int dx = abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    while (1)
    {
        draw_pixel(x0, y0, colour);

        if (x0 == x1 && y0 == y1)
        {
            // reached the end point, so exit the loop
            break;
        }

        int e2 = 2 * err;
        if (e2 >= dy)
        {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx)
        {
            err += dx;
            y0 += sy;
        }
    }
}

static void draw_char(int x, int y, char character, uint16_t colour, int scale)
{
    const uint8_t *columns = NULL;

    // Look up the character in the font table
    for (size_t i = 0;
         i < sizeof(font5x7) / sizeof(font5x7[0]);
         i++)
    {
        if (font5x7[i].character == character)
        {
            columns = font5x7[i].columns;
            break;
        }
    }

    if (columns == NULL || scale <= 0)
        return;

    // draw the five columns and seven rows
    for (int col = 0; col < 5; col++)
    {
        for (int row = 0; row < 7; row++)
        {
            if (columns[col] & (1U << row))
            {
                for (int dy = 0; dy < scale; dy++)
                {
                    for (int dx = 0; dx < scale; dx++)
                    {
                        draw_pixel(
                            x + col * scale + dx,
                            y + row * scale + dy,
                            colour
                        );
                    }
                }
            }
        }
    }
}

static void draw_text(int x, int y, const char *message, uint16_t colour, int scale)
{
    for (const char *p = message; *p != '\0'; p++)
    {
        draw_char(x, y, *p, colour, scale);

        // move to the next character
        x += 6 * scale;
    }
}

static void format_current(char *buffer, size_t size, float current_ma)
{
    if (fabsf(current_ma) >= 1000.0f)
    {
        snprintf(buffer, size, "%.3f A", current_ma / 1000.0f);
    }
    else
    {
        snprintf(buffer, size, "%.1f mA", current_ma);
    }
}

// draw the whole graph for every sample...
static void draw_graph(struct esp_lcd_panel_t *panel_handle, uint32_t elapsed_ms)
{
    // RGB565 colour values for the graph
    uint16_t black = rgb565_be(0x0000);
    uint16_t white = rgb565_be(0xFFFF);
    uint16_t green = rgb565_be(0x07E0);
    uint16_t red = rgb565_be(0xF800);
    uint16_t orange = rgb565_be(0xFD20);

    // clear the entire frame buffer to black
    for (int i = 0; i < LCD_V_RES * LCD_H_RES; i++)
    {
        frame_buffer[i] = black;
    }

    // coordinates of the graph boundries
    int graph_left = GRAPH_X;
    int graph_right = GRAPH_X + GRAPH_W - 1;
    int graph_top = GRAPH_Y;
    int graph_bottom = GRAPH_Y + GRAPH_H - 1;

    uint32_t window_start_ms;
    // determine the start of the 5 minute window for the graph
    if (elapsed_ms > GRAPH_HISTORY)
    {
        window_start_ms = elapsed_ms - GRAPH_HISTORY;
    }
    else
    {
        window_start_ms = 0;
    }

    // draw Y axis
    draw_line(graph_left, graph_top, graph_left, graph_bottom, white);
    // draw X axis
    draw_line(graph_left, graph_bottom, graph_right, graph_bottom, white);

    // draw time ticks for each minute of displayed time (we are displaying 5 minutes)
    for (int i = 0; i <= 5; i++)
    {
        int x = GRAPH_X + i * (GRAPH_W - 1) / 5;
        draw_line(x, graph_bottom, x, graph_bottom - 5, white);
    }

    float max_current = 0.0f; // initialise max_current to zero

    // find the maximum current value in sample buffer to determine appropiate max y axis value
    for (int i = 0; i < sample_count; i++)
    {
        // only consider samples inside visible 5 minute time window
        uint32_t sample_elapsed_ms = (uint32_t)samples[i].timestamp_ms - session_start_ms;
        if ((sample_elapsed_ms >= window_start_ms) && (sample_elapsed_ms <= elapsed_ms))
        {
            if (samples[i].current > max_current)
            {
                max_current = samples[i].current;
            }
        }
    }
    float y_max = fmaxf(max_current * 1.2f, 1.0f); // 20% headroom and avoid zero maximum

    // plot thresholds
    bool previous_valid = false;
    int previous_x = 0;
    int previous_y = 0;
    for (int x = graph_left; x <= graph_right; x++)
    {
        // convert X position into elapsed test time
        uint32_t t_ms = window_start_ms + (uint32_t)((x - GRAPH_X) * GRAPH_HISTORY / (GRAPH_W - 1));

        float threshold = threshold_ma_at(t_ms / 1000U);

        // don't plot future thresholds
        if (t_ms > elapsed_ms)
        {
            break;
        }

        // don't draw thresholds above the current graph scale
        if (threshold > y_max)
        {
            previous_valid = false;
            continue;
        }

        int y = value_to_y_scaled(threshold, y_max);

        if (previous_valid)
        {
            draw_line(previous_x, previous_y, x, y, orange);
        }

        previous_x = x;
        previous_y = y;
        previous_valid = true;
    }

    // plot lines connecting measurements
    if (sample_count >= 2)
    {
        for (int i = 1; i < sample_count; i++)
        {
            // calculated elapsed time between the two points we are connecting
            uint32_t t0 = (uint32_t)samples[i - 1].timestamp_ms - session_start_ms;
            uint32_t t1 = (uint32_t)samples[i].timestamp_ms - session_start_ms;

            // check that both endpoints lie inside the displayed time. If not, skip iteration
            if ((t0 < window_start_ms) || (t1 > window_start_ms + GRAPH_HISTORY))
            {
                continue;
            }

            // previous measuremnt
            int x0 = time_to_x(t0, window_start_ms);
            int y0 = value_to_y_scaled(samples[i - 1].current, y_max);

            // current measurement
            int x1 = time_to_x(t1, window_start_ms);
            int y1 = value_to_y_scaled(samples[i].current, y_max);

            // draw line between the two measurements with colour based on the current value
            uint16_t line_colour;
            float threshold = threshold_ma_at(t1 / 1000U);
            if (samples[i].current > threshold)
            {
                line_colour = red; // red if current is above threshold
            }
            else
            {
                line_colour = green; // green if current is below threshold
            }

            draw_line(x0, y0, x1, y1, line_colour);
        }
    }

    char current_text[24];
    char time_text[24];
    char threshold_text[24];
    char label[48];

    float current_ma;
    if (sample_count > 0)
    {
        current_ma = samples[sample_count - 1].current;
    }
    else
    {
        current_ma = 0.0f;
    }

    float threshold = threshold_ma_at(elapsed_ms / 1000U);

    format_current(current_text, sizeof(current_text), current_ma);
    format_current(threshold_text, sizeof(threshold_text), threshold);

    uint32_t seconds = elapsed_ms / 1000U;

    snprintf(time_text, sizeof(time_text),
             "%02lu:%02lu:%02lu",
             (unsigned long)(seconds / 3600U),
             (unsigned long)((seconds / 60U) % 60U),
             (unsigned long)(seconds % 60U));

    uint16_t cyan = rgb565_be(0x07FF);

    // live current
    draw_text(8, 5, "CURRENT", white, 1);
    draw_text(8, 23, current_text, cyan, 2);

    // elapsed time
    snprintf(label, sizeof(label), "TIME %s", time_text);
    draw_text(195, 9, label, white, 1);

    // current threshold
    snprintf(label, sizeof(label), "LIMIT %s", threshold_text);
    draw_text(195, 29, label, orange, 1);

    // instantaneous status
    draw_text(8, 60,
              current_ma > threshold ? "ABOVE LIMIT" : "BELOW LIMIT",
              current_ma > threshold ? red : green,
              1);

    // send completed frame buffer to the LCD
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(
        panel_handle,
        0,           // starting x coordinate
        0,           // starting y coordinate
        LCD_H_RES,   // ending x coordinate (not included)
        LCD_V_RES,   // ending y coordinate (not included)
        frame_buffer // send the frame buffer to the LCD
        ));
}

static void reset_sd_logging(void)
{
    ESP_LOGI(SD_TAG, "Resetting SD logging");

    // Close the current log file if oppen
    if (sd_log_file != NULL)
    {
        fflush(sd_log_file); // flush the file buffer to ensure all data is written to the SD card
        fclose(sd_log_file); // now close the file

        sd_log_file = NULL; // reset the file pointer to NULL
    }

    // disable SD card logging
    sd_logging_active = false;

    // unmount the SD card if it is mounted
    if (sd_card != NULL)
    {
        esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, sd_card);
        sd_card = NULL; // reset the SD card pointer to NULL}
    }

    // remount SD card
    if (init_sd_card() != ESP_OK)
    {
        ESP_LOGE(SD_TAG, "Failed to re-initialize SD card");
        return;
    }

    // create new log file
    if (create_sd_log_file() != ESP_OK)
    {
        ESP_LOGE(SD_TAG, "Failed to create new log file on SD card");
        esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, sd_card);
        sd_card = NULL;
        return;
    }

    ESP_LOGI(SD_TAG, "SD logging reset successfully");
}

// graph task
static void graph_task(void *pvParameters)
{
    // app main will pas the panel handle to this task via pvParameters
    struct esp_lcd_panel_t *panel_handle = (struct esp_lcd_panel_t *)pvParameters; // typcast

    measurement_t new_measurement;

    while (1)
    {
        // wait for a new measurement to be available in the queue
        if (xQueueReceive(measurement_queue, &new_measurement, portMAX_DELAY) == pdTRUE)
        {
            // check if reset button was pressed
            if (reset_requested)
            {
                ESP_LOGI(SD_TAG, "Reset requested, clearing graph and SD logger");

                // reset flag
                reset_requested = false;

                // reset test session timer
                if (xSemaphoreTake(measurement_mutex, portMAX_DELAY) == pdTRUE)
                {
                    session_start_ms = (uint32_t)new_measurement.timestamp_ms;
                    xSemaphoreGive(measurement_mutex);
                }

                // clear all graph samples
                sample_count = 0;

                // clear old measurements
                memset(samples, 0, sizeof(samples));

                // redraw empty graph
                draw_graph(panel_handle, 0);

                // restart SD logging
                reset_sd_logging();
            }

            // calculate time since the current session began
            uint32_t elapsed_time_ms = (uint32_t)new_measurement.timestamp_ms - session_start_ms;

            // store new measurement in the sample array
            add_sample(&new_measurement);

            ESP_LOGI(LCD_TAG, "Time: %lld ms, Current: %.1f mA", new_measurement.timestamp_ms, new_measurement.current);

            // write same measurement to SD card log file
            write_sd_measurement(&new_measurement);

            // redraw the graph with the new sample
            draw_graph(panel_handle, elapsed_time_ms);
        }
    }
}

/* data_get_handler to return JSON data for current
 runs whenever someone's browser requests /data (current draw) */
static esp_err_t data_get_handler(httpd_req_t *req)
{
    // buffer to hold JSON response
    char response[128];

    measurement_t latest_measurement_copy = {0}; // initialise snapshot of latest measurement to zero
    uint32_t session_start_copy = 0;

    if (xSemaphoreTake(measurement_mutex, portMAX_DELAY) == pdTRUE)
    {
        // copy the latest measurement to a function local variable
        latest_measurement_copy = latest_measurement;
        session_start_copy = session_start_ms;
        xSemaphoreGive(measurement_mutex);
    }

    uint32_t elapsed_ms = 0;

    if (latest_measurement_copy.timestamp_ms != 0)
    {
        elapsed_ms = (uint32_t)latest_measurement_copy.timestamp_ms - session_start_copy;
    }
    float threshold = threshold_ma_at(elapsed_ms/1000U);

    // format response as JSON with current value
    snprintf(response,
             sizeof(response),
             "{\"current\":%.1f,\"timestamp\":%lld,\"threshold\":%.1f}",
             latest_measurement_copy.current,
             latest_measurement_copy.timestamp_ms,
             threshold);

    // set response type to JSON
    httpd_resp_set_type(req, "application/json");
    // send response
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

// runs whenever someone's browser requests / (root) - main page
static esp_err_t root_get_handler(httpd_req_t *req)
{

    // browser will receive this HTML page when it requests /
    const char *html =
        "<!DOCTYPE html>"
        "<html>"

        "<head>"
        "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
        "<title>KOL Logger</title>"
        "<link rel=\"icon\" href=\"data:,\">"

        "<style>"

        "body {"
        "    font-family: Arial, sans-serif;"
        "    text-align: center;"
        "    background-color: #f4f4f4;"
        "    margin: 0;"
        "    padding: 30px;"
        "}"

        ".card {"
        "    background-color: white;"
        "    max-width: 400px;"
        "    margin: auto;"
        "    padding: 25px;"
        "    border-radius: 10px;"
        "}"

        ".status {"
        "    font-size: 40px;"
        "    font-weight: bold;"
        "    margin: 20px;"
        "}"

        ".current {"
        "    font-size: 36px;"
        "    font-weight: bold;"
        "    margin: 10px;"
        "}"

        ".info {"
        "    font-size: 18px;"
        "    margin: 10px;"
        "}"

        "</style>"
        "</head>"

        "<body>"

        "<div class=\"card\">"

        "<h1>KOL Logger</h1>"

        "<h2>Measurement Status</h2>"

        "<div class=\"status\" id=\"status\">--</div>"

        "<h2>Current Draw</h2>"

        "<div class=\"current\" id=\"current\">-- mA</div>"

        "<div class=\"info\" id=\"threshold\">Limit: -- mA</div>"

        "<h2>Elapsed Time</h2>"

        "<div class=\"info\" id=\"elapsed\">--:--:--</div>"

        "<div class=\"info\">Status: Logging</div>"

        "</div>"

        "<script>"

        "async function updateData() {"

        "    const response = await fetch('/data');"
        "    const data = await response.json();"

        // Current value
        "    document.getElementById('current').textContent = "
        "        data.current.toFixed(1) + ' mA';"

        // Threshold value
        "    document.getElementById('threshold').textContent = "
        "        'Threshold: ' + data.threshold.toFixed(1) + ' mA';"

        // PASS / FAIL
        "    if (data.current > data.threshold) {"

        "        document.getElementById('status').textContent = 'FAIL';"
        "        document.getElementById('status').style.color = 'red';"
        "        document.getElementById('current').style.color = 'red';"

        "    } else {"

        "        document.getElementById('status').textContent = 'PASS';"
        "        document.getElementById('status').style.color = 'green';"
        "        document.getElementById('current').style.color = 'green';"

        "    }"

        // Convert milliseconds to HH:MM:SS
        "    const totalSeconds = Math.floor(data.timestamp / 1000);"

        "    const hours = Math.floor(totalSeconds / 3600);"
        "    const minutes = Math.floor((totalSeconds % 3600) / 60);"
        "    const seconds = totalSeconds % 60;"

        "    const formattedTime = "
        "        String(hours).padStart(2, '0') + ':' + "
        "        String(minutes).padStart(2, '0') + ':' + "
        "        String(seconds).padStart(2, '0');"

        "    document.getElementById('elapsed').textContent = formattedTime;"

        "}"

        // Update immediately
        "updateData();"

        // Then update every second
        "setInterval(updateData, 1000);"

        "</script>"

        "</body>"
        "</html>";

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

// when web server requests /, this structure defines how to handle it (use root_get_handler)
static const httpd_uri_t root = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = root_get_handler,
    .user_ctx = NULL};
// when web server requests /data, this structure defines how to handle it (use data_get_handler)
static const httpd_uri_t data_uri = {
    .uri = "/data",
    .method = HTTP_GET,
    .handler = data_get_handler,
    .user_ctx = NULL};

// start_webserver function to start the HTTP server and register URI handlers
static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;

    // Create default configuration
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    ESP_LOGI(WIFI_TAG, "Starting HTTP server");

    if (httpd_start(&server, &config) == ESP_OK)
    {

        // Register / webpage
        httpd_register_uri_handler(server, &root);
        httpd_register_uri_handler(server, &data_uri);

        ESP_LOGI(WIFI_TAG, "HTTP server started");

        return server;
    }

    ESP_LOGE(WIFI_TAG, "Failed to start HTTP server");

    return NULL;
}

/* wifi_event_handler function to handle WiFi events
 Just log Wi-Fi events (station connected/disconnected) to the console */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        // // event_data is a pointer to a wifi_event_ap_staconnected_t structure
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(WIFI_TAG, "station " MACSTR " join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        // event_data is a pointer to a wifi_event_ap_stadisconnected_t structure
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(WIFI_TAG, "station " MACSTR " leave, AID=%d, reason=%d",
                 MAC2STR(event->mac), event->aid, event->reason);
    }
}

// wifi_init_softap function to initialize WiFi in softAP mode
void wifi_init_softap(void)
{
    // Initialize TCP/IP network interface (should be called only once in application)
    ESP_ERROR_CHECK(esp_netif_init());
    // Create the default event loop that running in background
    // The default event loop is used to handle Wi-Fi events and IP events
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    // Create default WIFI AP netif
    esp_netif_create_default_wifi_ap();
    // Initialize Wi-Fi with default configuration
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // call wifi_event_handler() when Wi-Fi events occur
    /*  Wi-Fi event occurs
       ↓
        ESP event loop
       ↓
        wifi_event_handler()*/
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    // configure wifi settings for softAP mode
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = ESP_WIFI_SSID,
            .ssid_len = strlen(ESP_WIFI_SSID),
            .channel = ESP_WIFI_CHANNEL,
            .password = ESP_WIFI_PASS,
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .required = false,
            },
#ifdef CONFIG_ESP_WIFI_BSS_MAX_IDLE_SUPPORT
            .bss_max_idle_cfg = {
                .period = WIFI_AP_DEFAULT_MAX_IDLE_PERIOD,
                .protected_keep_alive = 1,
            },
#endif
            .gtk_rekey_interval = GTK_REKEY_INTERVAL,
        },
    };
    if (strlen(ESP_WIFI_PASS) == 0)
    {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP)); // operate ESP32 as an Access Point (AP)
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start()); // start the Wi-Fi driver (ESP32 is now operating as an Access Point)

    ESP_LOGI(WIFI_TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             ESP_WIFI_SSID, ESP_WIFI_PASS, ESP_WIFI_CHANNEL);
}

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // configure the SPI bus
    ESP_LOGI(LCD_TAG, "Configuring SPI bus");
    // use ESP32 default SPI bus configuration
    spi_bus_config_t bus_config = {
        .mosi_io_num = PIN_NUM_MOSI, // ESP32 sends data to the LCD via this pin from the ESP32's MOSI pin to the LCD's SDI pin
        .miso_io_num = PIN_NUM_MISO, // ESP32 receives data from the LCD via this pin from the LCD's SDO pin to the ESP32's MISO pin (not used in this project)
        .sclk_io_num = PIN_NUM_SCLK, // ESP32 provides clock signal to the LCD, via this pin to the LCD's SCLK pin

        // using normal SPI mode, so no need for WP and HD pins
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,

        .max_transfer_sz = LCD_H_RES * LCD_V_RES * sizeof(uint16_t), // maximum transfer size possible in bytes
    };
    /*initialize the ESP32-S3 SPI bus (SPI2), using above configuration, and let the driver choose a DMA channel automatically
    DMA is a hardwired feature that the ESP32 has that allows it to transfer data from RAM to a peripheral, in this case, the SPI bus, without constantinvolvement of the CPU
    CPU only needs to be involved when initiating a transfer and when the transfer is complete.
    This is much faster than the CPU doing it all, and allows the CPU to do other things while the transfer is happening. */
    ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST, &bus_config, SPI_DMA_CH_AUTO));

    if (init_sd_card() == ESP_OK)
    {
        if (create_sd_log_file() != ESP_OK)
        {
            ESP_LOGE(SD_TAG, "Failed to create log file on SD card");

            esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, sd_card);
            sd_card = NULL;
        }
    }
    else
    {
        ESP_LOGE(SD_TAG, "Failed to initialize SD card");
    }

    // create the LCD panel SPI communication interface
    ESP_LOGI(LCD_TAG, "Creating LCD SPI Interface");
    /* define a pointer to a LCD panel IO object,
    initially we point it to NULL, but it will be redefined later with esp_lcd_new_panel_io_spi() to create the LCD panel SPI interface
    The I/O object reporesnts the communication interface between the ESP32 and the LCD */
    struct esp_lcd_panel_io_t *io_handle = NULL;
    // use ESP32 LCD SPI IO configuration structure to define how the LCD communicates with the SPI(2) bus
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = PIN_NUM_DC,     // ESP32 sends data/command signal to the LCD via this pin from the ESP32's GPIO pin to the LCD's D/C pin
        .cs_gpio_num = PIN_NUM_LCD_CS, // ESP32 selects the LCD via this pin from the ESP32's GPIO pin to the LCD's CS pin
        .pclk_hz = LCD_PIXEL_CLOCK_HZ, // SPI clock frequency used when communicating with the LCD
        .lcd_cmd_bits = 8,             // number of bits in a command
        .lcd_param_bits = 8,           // number of bits in a parameter
        .spi_mode = 0,                 // SPI mode 0
        .trans_queue_depth = 10,       // transaction, allows up to 10 transactions to be queued at once to wait
    };
    // finally create the LCD panel SPI interface using the defined configuration io_config, and provide a handle io_handle, and check for errors
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(
        (esp_lcd_spi_bus_handle_t)SPI_HOST,
        &io_config,
        &io_handle // This handle now contains a pointer to the LCD panel I/O object
        ));

    // create ILI9341 LCD paneL driver
    ESP_LOGI(LCD_TAG, "Creating ILI9341 LCD panel driver");
    /* define a pointer to a LCD panel object,
    initially we point it to NULL, but it will be redefined later with esp_lcd_new_panel_ili9341() to create tje ILI9341 LCD panel driver
    The panel object represents the actual LCD controller/driver */
    struct esp_lcd_panel_t *panel_handle = NULL;
    // use ESP32 LCD panel device configuration structure to define characteristics of the ILI9341 LCD panel
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_NUM_RST,              // ESP32 resets the LCD via this pin from the ESP32's GPIO pin to the LCD's RESET pin
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR, // colour component order
        .bits_per_pixel = 16,                       // number of bits per pixel used by the LCD
    };
    // finally create the ILI9341 LCD panel driver using the defined LCD panel I/O SPI interface, configuration panel_config, and provide a handle panel_handle, and check for errors
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(
        io_handle,
        &panel_config,
        &panel_handle // This handle now contains a pointer to the ILI9341 LCD panel object
        ));

    // reset the LCD
    ESP_LOGI(LCD_TAG, "Resetting LCD");
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));

    // initialize ILI9341
    ESP_LOGI(LCD_TAG, "Initializing LCD");
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true));
    // ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));

    // turn the display on
    ESP_LOGI(LCD_TAG, "Turning on LCD");
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    // create a queue to hold the latest measurement, with a length of 1 (only the latest measurement is needed)
    measurement_queue = xQueueCreate(1, sizeof(measurement_t));
    if (measurement_queue == NULL)
    {
        ESP_LOGE(LCD_TAG, "Failed to create measurement queue");
        return;
    }

    // create a mutex to protect access to latest_measurement
    measurement_mutex = xSemaphoreCreateMutex();
    if (measurement_mutex == NULL)
    {
        ESP_LOGE(LCD_TAG, "Failed to create measurement mutex");
        return;
    }

    session_start_ms = (uint32_t)(esp_timer_get_time() / 1000); // record when session started in milliseconds

    // reset button task and GPIO interrupt
    BaseType_t button_task_result = xTaskCreate(
        reset_button_task,        // task function
        "reset_button",           // task name
        2048,                     // stack size
        NULL,                     // nothing is passed to the task
        4,                        // task priority
        &reset_button_task_handle // task handle
    );
    if (button_task_result != pdPASS)
    {
        ESP_LOGE(LCD_TAG, "Failed to create reset button task");
        return;
    }

    gpio_config_t button_config = {
        .pin_bit_mask = (1ULL << PIN_NUM_RESET_BUTTON), // select GPIO4
        .mode = GPIO_MODE_INPUT,                        // set GPIO4 as input
        .pull_up_en = GPIO_PULLUP_ENABLE,               // enable pull-up resistor (internal)
        .pull_down_en = GPIO_PULLDOWN_DISABLE,          // disable pull-down resistor (internal)
        .intr_type = GPIO_INTR_NEGEDGE                  // trigger interrupt on falling edge (button press)
    };

    ESP_ERROR_CHECK(gpio_config(&button_config)); // apply the button configuration to the GPIO driver and check for errors

    esp_err_t isr_service_result = gpio_install_isr_service(0); // install the GPIO ISR service with default configuration (no flags)
    if ((isr_service_result != ESP_OK) && (isr_service_result != ESP_ERR_INVALID_STATE))
    {
        ESP_ERROR_CHECK(isr_service_result); // check for errors, but ignore ESP_ERR_INVALID_STATE (ISR service already installed)
    }
    ESP_ERROR_CHECK(gpio_isr_handler_add( // add the ISR handler for GPIO4 (reset button) and check for errors
        PIN_NUM_RESET_BUTTON,             // GPIO
        reset_button_isr,                 // pointer to ISR function for GPIO4 (reset button)
        NULL));

    // create empty graph with no samples
    draw_graph(panel_handle, 0);

    ESP_LOGI(WIFI_TAG, "ESP_WIFI_MODE_AP");
    wifi_init_softap();

    start_webserver();

    // graph task
    xTaskCreate(
        graph_task,   // task function
        "graph",      // task name
        4096,         // stack size
        panel_handle, // parameters passed to task (pass the LCD panel handle so the graph task can draw to the LCD)
        5,            // task priority
        NULL          // task handle
    );

    xTaskCreate(
        fake_measurement_task, // task function
        "fake_measurement",    // task name
        2048,                  // stack size
        NULL,                  // parameters passed to task
        5,                     // task priority
        NULL                   // task handle
    );

    ESP_LOGI(LCD_TAG, "Done!");
}

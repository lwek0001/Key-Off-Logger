#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include <string.h>

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
#define PIN_NUM_MOSI    GPIO_NUM_11
#define PIN_NUM_MISO    GPIO_NUM_13
#define PIN_NUM_SCLK    GPIO_NUM_12
#define PIN_NUM_LCD_CS  GPIO_NUM_10
#define PIN_NUM_SD_CS   GPIO_NUM_15
#define PIN_NUM_DC      GPIO_NUM_9
#define PIN_NUM_RST     GPIO_NUM_14

// native resolution of the ILI9341 LCD
#define LCD_H_RES 240
#define LCD_V_RES 320

// graph dimensions
#define GRAPH_X 20
#define GRAPH_Y 20
#define GRAPH_W 200
#define GRAPH_H 250

// configuration macros for WiFi settings, defined in project configuration
#define ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID // preprocessor macro for WiFi SSID from project configuration, associate ESP_WIFI_SSID with CONFIG_ESP_WIFI_SSID
#define ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define ESP_WIFI_CHANNEL   CONFIG_ESP_WIFI_CHANNEL
#define MAX_STA_CONN       CONFIG_ESP_MAX_STA_CONN

#if CONFIG_ESP_GTK_REKEYING_ENABLE
#define GTK_REKEY_INTERVAL CONFIG_ESP_GTK_REKEY_INTERVAL
#else
#define GTK_REKEY_INTERVAL 0
#endif

#define SD_MOUNT_POINT "/sdcard"
#define SD_MAX_LOG_LINES 20

static sdmmc_card_t *sd_card = NULL;
static FILE *sd_log_file = NULL;

static uint32_t sd_line_number = 1;

static bool sd_logging_active = false;

static const char *TAG_SD = "SD";

// initialise fake measurements for testing
typedef struct {
    float current; // current draw in mA
    int64_t timestamp_ms; // timestamp in milli seconds since the start of the program
} measurement_t;

static measurement_t latest_measurement = {
    .current = 100.0f,
    .timestamp_ms = 0
};

static SemaphoreHandle_t measurement_mutex; // mutex to protect access to latest_measurement

// pointer to queue used by scheduler to hold the fake measurements (this is equivalent to static struct QueueDefinition* measurement_queue)
static QueueHandle_t measurement_queue;

static measurement_t samples[GRAPH_W];
static int sample_count = 0;


// for logging
static const char *TAG1 = "LCD";
static const char *TAG = "wifi softAP";

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
    // avoid compiler warning about unused parameter
    (void)pvParameters;
    while (1)
    {
        // generate one measurement per second
        vTaskDelay(pdMS_TO_TICKS(1000));

        // guarentee queue gets consistent current value and timestamp
        if (xSemaphoreTake(measurement_mutex, portMAX_DELAY)==pdTRUE)
        {
            latest_measurement.current -= 1.0f;
            if (latest_measurement.current < 0.0f) {
                latest_measurement.current = 100.0f;
            }
            latest_measurement.timestamp_ms = esp_timer_get_time() / 1000; // time in milliseconds since the start of the program

            // send the current measurement to the graph task via the queue (will only contain the latest measurement)
            xQueueOverwrite(measurement_queue, &latest_measurement);

            xSemaphoreGive(measurement_mutex);
        }
    }
}

// add a new measurement to the samples array, shifting the existing measurements to the left if the array is full
static void add_sample(const measurement_t *value)
{
    if (sample_count < GRAPH_W) {
        // graph is not full yet
        samples[sample_count] = *value;
        sample_count++;
    } else {
        // graph is full, so shift every measurement one position to the left
        for (int i=0; i < GRAPH_W-1; i++) {
            samples[i] = samples[i+1];
        }
        samples[GRAPH_W-1] = *value;
    }
}

// SD initialisation function
static esp_err_t init_sd_card(void)
{
    ESP_LOGI(TAG_SD, "Mounting SD card...");

    // SD card host configuration
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    // using SPI2 bus
    host.slot = SPI_HOST;
    host.max_freq_khz = 4000;

    // configure SD card as a device on the SPI2 bus which is shared
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_SD_CS; // chip select pin for SD card
    slot_config.host_id = SPI_HOST; // use SPI2 bus

    // FAT filesystem mount configuration
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false, // do not format the card if mounting fails
        .max_files = 5, // maximum number of files that can be open at the same time
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
        ESP_LOGE(TAG_SD, "Failed to mount SD card: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG_SD, "SD card successfully mounted at %s", SD_MOUNT_POINT);

    sdmmc_card_print_info(stdout, sd_card); // print SD card information to console

    return ESP_OK;
}

// create SD card log file
static esp_err_t create_sd_log_file(void)
{
    sd_log_file = fopen(SD_MOUNT_POINT"/V001_KOL.TXT", "w"); // w means write mode, will create the file if it does not exist, and overwrite it if it does exist

    if (sd_log_file == NULL) 
    {
        ESP_LOGE(TAG_SD, "Failed to create log file on SD card");
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

    ESP_LOGI(TAG_SD, "Log file created on SD card: %s/V001_KOL.TXT", SD_MOUNT_POINT);

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

    int64_t elapsed_seconds = measurement->timestamp_ms / 1000; // convert timestamp from ms to s

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
            "NA"
    );
    fflush(sd_log_file); // flush the file buffer to ensure the data is written to the SD card

    ESP_LOGI(TAG_SD, "SD line %lu written - Current: %.1f mA", (unsigned long)sd_line_number, measurement->current);

    sd_line_number++; // increment line number for next measurement

    // stop after 20 measurements (for testing purposes)
    if (sd_line_number > SD_MAX_LOG_LINES)
    {
        ESP_LOGI(TAG_SD,
                 "20 measurements written");


        // Flush anything remaining
        fflush(sd_log_file);


        // Close file
        fclose(sd_log_file);

        sd_log_file = NULL;


        // Unmount SD card
        esp_vfs_fat_sdcard_unmount(
            SD_MOUNT_POINT,
            sd_card
        );

        sd_card = NULL;

        sd_logging_active = false;


        ESP_LOGI(TAG_SD,
                 "SD card unmounted");

        ESP_LOGI(TAG_SD,
                 "Safe to remove SD card");
    }
}

// convert measurement value (0-100) to a y coord
static int value_to_y(float value)
{
    // limit the value to the range 0-100
    if (value < 0.0f) {
        value = 0.0f;
    } else if (value > 100.0f) {
        value = 100.0f;
    }
    // convert 100 -> top of graph, 0 -> bottom of graph
    float normalized = value / 100.0f;
    int y = GRAPH_Y + (GRAPH_H - 1) - (int)(normalized * (GRAPH_H - 1));
    return y;
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

    while (1) {
        draw_pixel(x0, y0, colour);

        if (x0 == x1 && y0 == y1) 
        {
            // reached the end point, so exit the loop
            break;
        }

        int e2 = 2 * err;
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

// draw the whole graph for every sample...
static void draw_graph(struct esp_lcd_panel_t *panel_handle)
{
  // RGB565 colour values for the graph
  uint16_t black = rgb565_be(0x0000); // black
  uint16_t white = rgb565_be(0xFFFF); // white  
  uint16_t green = rgb565_be(0x07E0); // green

  // clear the entire frame buffer to black
  for (int i=0; i < LCD_V_RES*LCD_H_RES; i++)
  {
    frame_buffer[i] = black;
  }

  // coordinates of the graph boundries
  int graph_left = GRAPH_X;
  int graph_right = GRAPH_X + GRAPH_W - 1;
  int graph_top = GRAPH_Y;
  int graph_bottom = GRAPH_Y + GRAPH_H - 1;

  // draw Y axis
  draw_line(graph_left, graph_top, graph_left, graph_bottom, white);
  // draw X axis
  draw_line(graph_left, graph_bottom, graph_right, graph_bottom, white);

  // draw time ticks every 10 pixels (1 pixel = 1 second)
  for (int x=graph_left; x <= graph_right; x += 10) 
  {
      draw_line(x, graph_bottom, x, graph_bottom - 5, white);
  }

  if (sample_count >= 2) 
  {
    int64_t start_time = samples[0].timestamp_ms;

    for (int i=1; i < sample_count; i++)
    {
        // previous measuremnt
        int x0 = GRAPH_X + (samples[i - 1].timestamp_ms - start_time) / 1000; // convert milliseconds to seconds
        int y0 = value_to_y(samples[i - 1].current);

        // current measurement
        int x1 = GRAPH_X + (samples[i].timestamp_ms - start_time) / 1000; // convert milliseconds to seconds
        int y1 = value_to_y(samples[i].current);

        // draw line between the two measurements
        draw_line(x0, y0, x1, y1, green);   
    }
  }

  // send completed frame buffer to the LCD
  ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(
      panel_handle,
      0,              // starting x coordinate
      0,              // starting y coordinate
      LCD_H_RES,      // ending x coordinate (not included)
      LCD_V_RES,      // ending y coordinate (not included)
      frame_buffer // send the frame buffer to the LCD
  ));
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
            // store new measurement in the sample array
            add_sample(&new_measurement);
            ESP_LOGI(TAG1, "Time: %lld ms, Current: %.1f mA", new_measurement.timestamp_ms, new_measurement.current);

            // write same measurement to SD card log file
            write_sd_measurement(&new_measurement);

            // redraw the graph with the new sample
            draw_graph(panel_handle);
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

    if (xSemaphoreTake(measurement_mutex, portMAX_DELAY)==pdTRUE)
    {
        // copy the latest measurement to a function local variable
        latest_measurement_copy = latest_measurement;
        xSemaphoreGive(measurement_mutex);
    }

    // format response as JSON with current value
    snprintf(response,
             sizeof(response),
             "{\"current\":%.1f,\"timestamp\":%lld}",
             latest_measurement_copy.current, latest_measurement_copy.timestamp_ms);
    
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

        ".value {"
        "    font-size: 32px;"
        "    font-weight: bold;"
        "    margin: 10px;"
        "}"
        "</style>"

        "</head>"

        "<body>"

        "<div class=\"card\">"

        "<h1>KOL Logger</h1>"
        "<p>Status: Running</p>"

        "<h2>Current Draw</h2>"
        "<div class=\"value\" id=\"current\">-- mA</div>"
        "<div class=\"value\" id=\"timestamp\">Time: -- ms</div>"

        "</div>"

        "<script>"

        "async function updateData() {"

        "    const response = await fetch('/data');"
        "    const data = await response.json();"

        "    document.getElementById('current').textContent = "
        "        data.current.toFixed(1) + ' mA';"

        "    document.getElementById('timestamp').textContent = "
        "        'Time: ' + data.timestamp + ' ms';"

        "}"

        "updateData();"

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
    .user_ctx = NULL
};
// when web server requests /data, this structure defines how to handle it (use data_get_handler)
static const httpd_uri_t data_uri = {
    .uri = "/data",
    .method = HTTP_GET,
    .handler = data_get_handler,
    .user_ctx = NULL
};

// start_webserver function to start the HTTP server and register URI handlers
static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;

    // Create default configuration 
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    ESP_LOGI(TAG, "Starting HTTP server");

    if (httpd_start(&server, &config) == ESP_OK) {

        // Register / webpage
        httpd_register_uri_handler(server, &root);
        httpd_register_uri_handler(server, &data_uri);

        ESP_LOGI(TAG, "HTTP server started");

        return server;
    }

    ESP_LOGE(TAG, "Failed to start HTTP server");

    return NULL;
}

/* wifi_event_handler function to handle WiFi events
 Just log Wi-Fi events (station connected/disconnected) to the console */
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        // // event_data is a pointer to a wifi_event_ap_staconnected_t structure
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        // event_data is a pointer to a wifi_event_ap_stadisconnected_t structure
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d, reason=%d",
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
#ifdef CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT
            .authmode = WIFI_AUTH_WPA3_PSK,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
#else /* CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT */
            .authmode = WIFI_AUTH_WPA2_PSK,
#endif
            .pmf_cfg = {
                    .required = true,
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
    if (strlen(ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP)); // operate ESP32 as an Access Point (AP)
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());  // start the Wi-Fi driver (ESP32 is now operating as an Access Point)

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             ESP_WIFI_SSID, ESP_WIFI_PASS, ESP_WIFI_CHANNEL);
}

void app_main(void)
{
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // configure the SPI bus
    ESP_LOGI(TAG1, "Configuring SPI bus");
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
    DMA is a hardwired feature of the ESP32 that allows it to transfer data from RAM to a peripheral, in this case this SPI bus, without constant involvement of the CPU
    CPU only needs to be involved when initiating a transfer and when the transfer is complete.
    This is much faster than the CPU doing it, and allows the CPU to do other things while the transfer is happening. */
    ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST, &bus_config, SPI_DMA_CH_AUTO));
    
    if (init_sd_card() == ESP_OK)
    {
        if (create_sd_log_file() != ESP_OK)
        {
            ESP_LOGE(TAG_SD, "Failed to create log file on SD card");

            esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, sd_card);
            sd_card = NULL;
        }
    }
    else
    {
        ESP_LOGE(TAG_SD, "Failed to initialize SD card");
    }

    // create the LCD panel SPI communication interface
    ESP_LOGI(TAG1, "Creating LCD SPI Interface");
    /* define a pointer to a LCD panel IO object, 
    initially we point it to NULL, but it will be redefined later with esp_lcd_new_panel_io_spi() to create the LCD panel SPI interface
    The I/O object reporesnts the communication interface between the ESP32 and the LCD */
    struct esp_lcd_panel_io_t *io_handle = NULL; 
    // use ESP32 LCD SPI IO configuration structure to define how the LCD communicates with the SPI(2) bus
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = PIN_NUM_DC, // ESP32 sends data/command signal to the LCD via this pin from the ESP32's GPIO pin to the LCD's D/C pin
        .cs_gpio_num = PIN_NUM_LCD_CS, // ESP32 selects the LCD via this pin from the ESP32's GPIO pin to the LCD's CS pin
        .pclk_hz = LCD_PIXEL_CLOCK_HZ, // SPI clock frequency used when communicating with the LCD
        .lcd_cmd_bits = 8, // number of bits in a command
        .lcd_param_bits = 8, // number of bits in a parameter
        .spi_mode = 0, // SPI mode 0
        .trans_queue_depth = 10, // transaction, allows up to 10 transactions to be queued at once to wait
    };
    // finally create the LCD panel SPI interface using the defined configuration io_config, and provide a handle io_handle, and check for errors
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(
        (esp_lcd_spi_bus_handle_t)SPI_HOST, 
        &io_config, 
        &io_handle // This handle now contains a pointer to the LCD panel I/O object
    ));


    // create ILI9341 LCD paneL driver
    ESP_LOGI(TAG1, "Creating ILI9341 LCD panel driver");
    /* define a pointer to a LCD panel object, 
    initially we point it to NULL, but it will be redefined later with esp_lcd_new_panel_ili9341() to create tje ILI9341 LCD panel driver
    The panel object represents the actual LCD controller/driver */
    struct esp_lcd_panel_t *panel_handle = NULL; 
    // use ESP32 LCD panel device configuration structure to define characteristics of the ILI9341 LCD panel
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_NUM_RST, // ESP32 resets the LCD via this pin from the ESP32's GPIO pin to the LCD's RESET pin
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB, // colour component order, we are using R - G- B order
        .bits_per_pixel = 16, // number of bits per pixel used by the LCD
    };
    // finally create the ILI9341 LCD panel driver using the defined LCD panel I/O SPI interface, configuration panel_config, and provide a handle panel_handle, and check for errors
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(
        io_handle, 
        &panel_config, 
        &panel_handle // This handle now contains a pointer to the ILI9341 LCD panel object
    ));


    // reset the LCD
    ESP_LOGI(TAG1, "Resetting LCD");
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));

    // initialize ILI9341
    ESP_LOGI(TAG1, "Initializing LCD");
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));

    // turn the display on
    ESP_LOGI(TAG1, "Turning on LCD");
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    // create a queue to hold the latest measurement, with a length of 1 (only the latest measurement is needed)
    measurement_queue = xQueueCreate(1, sizeof(measurement_t));
    if (measurement_queue == NULL) {
        ESP_LOGE(TAG1, "Failed to create measurement queue");
        return;
    }

    // create a mutex to protect access to latest_measurement
    measurement_mutex = xSemaphoreCreateMutex();
    if (measurement_mutex == NULL) {
        ESP_LOGE(TAG1, "Failed to create measurement mutex");
        return;
    }

    // create empty graph with no samples
    draw_graph(panel_handle);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
    wifi_init_softap();

    start_webserver();

    // graph task
    xTaskCreate(
    graph_task,             // task function
    "graph",                // task name
    4096,                   // stack size
    panel_handle,           // parameters passed to task (pass the LCD panel handle so the graph task can draw to the LCD)
    5,                      // task priority
    NULL                    // task handle
    );

    xTaskCreate(
    fake_measurement_task,  // task function
    "fake_measurement",     // task name
    2048,                   // stack size  
    NULL,                   // parameters passed to task 
    5,                      // task priority    
    NULL                    // task handle
    );

    ESP_LOGI(TAG1, "Done!");
}

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "driver/rc522_spi.h"
#include "rc522.h"
#include "esp_log.h"


#define LED_ENTRY_GREEN 32
#define LED_ENTRY_RED   33
#define LED_EXIT_GREEN 27
#define LED_EXIT_RED   14
#define PIN_MOSI   23
#define PIN_MISO   19
#define PIN_SCLK   18
#define ENTRY_CS   5
#define SERVO_ENTRY_PIN 25
#define SERVO_EXIT_PIN 26
#define SERVO_CLOSED_US  500    // door closed, arm at about 0 degrees
#define SERVO_OPEN_US   1470    // door open, arm at about 90 degrees
#define RST 4
#define EXIT_CS 17
#define SD_CS 16

typedef enum { READER_ENTRY, READER_EXIT } reader_t;

typedef struct {char uid[RC522_PICC_UID_STR_BUFFER_SIZE_MAX]; bool valid; reader_t reader;}scan_event_t;

typedef struct {int green_pin; int red_pin; int servo_pin; ledc_channel_t channel;} door_t;

static const door_t doors[2] = {{32, 33, SERVO_ENTRY_PIN, LEDC_CHANNEL_0}, 
{27, 14, SERVO_EXIT_PIN, LEDC_CHANNEL_1}
};

static QueueHandle_t scan_queue;

#define SPI_HOST   SPI3_HOST   // TODO 0: your choice, SPI2_HOST or SPI3_HOST. Be ready to say why.

static const char *TAG = "ENTRANCE";

// TODO 1: write a function  esp_err_t spi_bus_setup(void)
esp_err_t spi_bus_setup(void) {
    spi_bus_config_t  config = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    return spi_bus_initialize(SPI_HOST, &config, SPI_DMA_CH_AUTO);
}
//   - declare an spi_bus_config_t variable
//   - set .mosi_io_num, .miso_io_num, .sclk_io_num
//   - set .quadwp_io_num and .quadhd_io_num to -1 (we're not using quad SPI)
//   - set .max_transfer_sz (hint: the SD card and RC522 both move small
//     chunks, so 4096 is plenty)
//   - call spi_bus_initialize(SPI_HOST, &cfg, SPI_DMA_CH_AUTO)
//   - return the result
static rc522_spi_config_t entry_config = {
    .host_id = SPI_HOST,
    .bus_config = NULL,
    .dev_config = {
    .spics_io_num = ENTRY_CS,
    .clock_speed_hz = 1000000,
},
    .rst_io_num = RST
};

static rc522_spi_config_t exit_config = {
    .host_id = SPI_HOST,
    .bus_config = NULL,
    .dev_config = {
    . spics_io_num = EXIT_CS,
    .clock_speed_hz = 1000000,
    },
    .rst_io_num = -1
};

// TODO 1: a static const array of valid tag strings, e.g.
//         static const char *valid_tags[] = { "AA BB CC DD" };
static const char *valid_tags[] = {"AA BB CC DD"};
//         (I expect the helper to format as "01 02 03 04" with spaces,
//          but check what your own log prints)
// TODO 2: write bool tag_is_valid(const char *uid_str)
// loop over valid_tags with strcmp, return true on a match
bool tag_is_valid(const char *uid_str)
{
    for (size_t i = 0; i < sizeof(valid_tags) / sizeof(valid_tags[0]); i++) {
        if (strcmp(uid_str, valid_tags[i]) == 0) {
            return true;
        }
    }
    return false;
}

void leds_init(void) {
//Loop over both doors using the doors[] table, and set the green and red pin of each as an output, low.
    for (size_t i = 0; i < sizeof(doors) / sizeof(doors[0]); ++i) {
        gpio_reset_pin(doors[i].green_pin);
        gpio_reset_pin(doors[i].red_pin);
        gpio_set_direction(doors[i].green_pin, GPIO_MODE_OUTPUT);
        gpio_set_direction(doors[i].red_pin, GPIO_MODE_OUTPUT);
        gpio_set_level(doors[i].green_pin, 0);
        gpio_set_level(doors[i].red_pin, 0);
    }
}

// EDIT: servo_init. Keep the timer config exactly as it is, and do it ONCE.
//       Then configure one channel per door in a loop, each with its own
//       gpio_num and channel from doors[], on the same LEDC_TIMER_0.
void servo_init(void) {
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_14_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    for (size_t i = 0; i < sizeof(doors) / sizeof(doors[0]); ++i) {
        ledc_channel_config_t channel_config = {
            .gpio_num = doors[i].servo_pin,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .channel = doors[i].channel,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&channel_config));
    }
}

// EDIT: servo_set_pulse_us. Add a parameter saying which door (reader_t),
//       and use doors[door].channel instead of the hard-coded LEDC_CHANNEL_0
//       in both ledc_set_duty and ledc_update_duty.
void servo_set_pulse_us(uint32_t pulse_us, reader_t door) {
    uint32_t duty = (pulse_us * (1 << 14)) / 20000;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, doors[door].channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, doors[door].channel));
}

// EDIT: door_open and door_close. Add a reader_t parameter and pass it on
//       to servo_set_pulse_us.
void door_open(reader_t door) {
    servo_set_pulse_us(SERVO_OPEN_US, door);
}

void door_close(reader_t door) {
    servo_set_pulse_us(SERVO_CLOSED_US, door);
}

// EDIT: door_task. Use event.reader to pick the door:
//       - the green and red LED pins come from doors[event.reader]
//       - door_open / door_close get event.reader
//       Replace the two LED_ENTRY_* names with the table lookups.
void door_task(void *arg) {
    scan_event_t event;
    while (1) {
        if (xQueueReceive(scan_queue, &event, portMAX_DELAY)) {
            if (event.valid) {
                ESP_LOGI(TAG, "Door %d opened for tag: %s", event.reader, event.uid);
                gpio_set_level(doors[event.reader].green_pin, 1);
                door_open(event.reader);
                vTaskDelay(pdMS_TO_TICKS(3000));
                door_close(event.reader);
                gpio_set_level(doors[event.reader].green_pin,0);
            } else {
                ESP_LOGW(TAG, "Access denied for tag: %s", event.uid);
                gpio_set_level(doors[event.reader].red_pin, 1);
                vTaskDelay(pdMS_TO_TICKS(1000));
                gpio_set_level(doors[event.reader].red_pin, 0);
            }
        }
    }
}

// EDIT: on_picc_state_changed.
//   1. At the top, read which reader fired. The last argument you gave to
//      rc522_register_events comes back here as "arg". Cast it back:
//      reader_t reader = (reader_t)(intptr_t)arg;
//   2. Before xQueueSend, set ev.reader = reader;
//   3. Include the reader in your log line so you can see which one fired.
static void on_picc_state_changed(void *arg, esp_event_base_t base, int32_t event_id, void *data)
{
    rc522_picc_state_changed_event_t *event = (rc522_picc_state_changed_event_t *)data;
    rc522_picc_t *picc = event->picc;
    if (picc->state != RC522_PICC_STATE_ACTIVE) {
        return;
    }
    reader_t reader = (reader_t)(intptr_t)arg;

    char uid_str[RC522_PICC_UID_STR_BUFFER_SIZE_MAX];
    if (rc522_picc_uid_to_str(&picc->uid, uid_str, sizeof(uid_str)) != ESP_OK) {
        ESP_LOGE(TAG, "Reader %d: Failed to convert UID to string", reader);
        return;
    }
    ESP_LOGI(TAG, "Tag scanned (reader %d): %s -> %s", reader, uid_str,
        tag_is_valid(uid_str) ? "VALID" : "DENIED");

    scan_event_t ev;
    strncpy(ev.uid, uid_str, sizeof(ev.uid));
    ev.valid = tag_is_valid(uid_str);
    ev.reader = reader;
    xQueueSend(scan_queue, &ev, 0);
}

static rc522_driver_handle_t entry_driver;
static rc522_handle_t entry_scanner;
static rc522_driver_handle_t exit_driver;
static rc522_handle_t exit_scanner;

// EDIT: entry_reader_start. Turn it into one function that works for both readers.
//   Give it parameters: a pointer to the spi config, a pointer to the driver
//   handle, a pointer to the scanner handle, and a reader_t id.
//   Use those instead of the entry_* globals inside.
//   In rc522_register_events, replace the last argument (NULL) with
//   (void *)(intptr_t)id, which is what the callback reads back as "arg".
//   Rename it (for example reader_start) and call it twice from app_main.
static void reader_start(rc522_spi_config_t *spi_config,
                         rc522_driver_handle_t *driver,
                         rc522_handle_t *scanner,
                         reader_t id)
{
    ESP_ERROR_CHECK(rc522_spi_create(spi_config, driver));
    ESP_ERROR_CHECK(rc522_driver_install(*driver));
    rc522_config_t scanner_config = {
        .driver = *driver,
    };
    ESP_ERROR_CHECK(rc522_create(&scanner_config, scanner));
    ESP_ERROR_CHECK(rc522_register_events(*scanner, RC522_EVENT_PICC_STATE_CHANGED,
        on_picc_state_changed, (void *)(intptr_t)id));
    ESP_ERROR_CHECK(rc522_start(*scanner));
}

void deselect_other_devices(void){
    gpio_reset_pin(SD_CS);
    gpio_set_direction(SD_CS, GPIO_MODE_OUTPUT);
    gpio_set_level(SD_CS, 1);
}

void app_main(void)
{
    esp_err_t value = spi_bus_setup();
    if (value == ESP_OK) {
        ESP_LOGI(TAG, "SPI bus ready");
        esp_log_level_set("rc522", ESP_LOG_NONE);
        deselect_other_devices();   // EDIT: SD card only now
        leds_init();                // now sets up both doors
        servo_init();               // now sets up both doors
        door_close(READER_ENTRY);
        door_close(READER_EXIT);
        scan_queue = xQueueCreate(5, sizeof(scan_event_t));
        xTaskCreate(door_task, "door_task", 4096, NULL, 5, NULL);
        reader_start(&entry_config, &entry_driver, &entry_scanner, READER_ENTRY);
        reader_start(&exit_config, &exit_driver, &exit_scanner, READER_EXIT);
    } else {
        ESP_LOGE(TAG, "Failed to setup SPI bus: %s", esp_err_to_name(value));
    }
}
#if __has_include(<Arduino.h>)
#include <Arduino.h>
#define I2C_INTERNAL Wire1
#else
//#define USE_SINGLE_BUFFER
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#define I2C_INTERNAL I2C_NUM_1
#endif
#include "i2s_sampler.hpp"
#include "processor.hpp"
#include "ui.hpp"
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_lcd_panel_ili9342.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <ft6336.hpp>
#include <m5core2_power.hpp>
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
#include <driver/i2s_pdm.h>
#else
#include <driver/i2s.h>
#endif
#ifdef ARDUINO
using namespace arduino;
#else
using namespace esp_idf;
static uint32_t millis() {
    return ((uint32_t)pdTICKS_TO_MS(xTaskGetTickCount()));
}
#endif
static ft6336<320, 280> touch(I2C_INTERNAL);
static m5core2_power power;
// approx 30ms of audio @ 16KHz
#define WINDOW_SIZE 512
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
i2s_pdm_rx_config_t pdm_rx_cfg = {
    .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(64000),
    /* The default mono slot is the left slot (whose 'select pin' of the PDM microphone is pulled down) */
    .slot_cfg = { 
    .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
    .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
    .slot_mode = I2S_SLOT_MODE_MONO,
    .slot_mask = I2S_PDM_SLOT_RIGHT,
    },
    .gpio_cfg = {
        .clk = GPIO_NUM_0,
        .din = GPIO_NUM_34,
        .invert_flags = {
            .clk_inv = false,
        },
    },
};
#else
// i2s config for reading from both m5stack mic
static i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
    .sample_rate = 64000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 2,
    .dma_buf_len = 1024,
};

// i2s pins
static i2s_pin_config_t i2s_pins = {
                            .mck_io_num = I2S_PIN_NO_CHANGE,
                            .bck_io_num = GPIO_NUM_12,
                            .ws_io_num = GPIO_NUM_0,
                            .data_out_num = I2S_PIN_NO_CHANGE,
                            .data_in_num = GPIO_NUM_34
};
#endif
static esp_lcd_panel_handle_t lcd_handle;
#ifndef USE_SINGLE_BUFFER
// use two 32KB buffers (DMA)
static uint8_t lcd_transfer_buffer1[32 * 1024];
static uint8_t lcd_transfer_buffer2[32 * 1024];
#else
static uint8_t lcd_transfer_buffer1[64 * 1024];
static uint8_t* const lcd_transfer_buffer2 = nullptr;
#endif
static i2s_sampler<WINDOW_SIZE> main_sampler;
static processor<WINDOW_SIZE> main_processor;
static TaskHandle_t processing_task_handle;
static TaskHandle_t drawing_task_handle;

// the screen/control definitions
screen_t main_screen({320, 240},
                     sizeof(lcd_transfer_buffer1),
                     lcd_transfer_buffer1,
                     lcd_transfer_buffer2);
analyzer_box_t main_analyzer(main_screen);

// tell UIX the DMA transfer is complete
static bool lcd_flush_ready(esp_lcd_panel_io_handle_t panel_io,
                            esp_lcd_panel_io_event_data_t *edata,
                            void *user_ctx) {
    main_screen.flush_complete();
    return true;
}

// tell the lcd panel api to transfer data via DMA
static void lcd_on_flush(const gfx::rect16 &bounds, const void *bmp, void *state) {
    int x1 = bounds.x1, y1 = bounds.y1, x2 = bounds.x2 + 1, y2 = bounds.y2 + 1;
    esp_lcd_panel_draw_bitmap(lcd_handle, x1, y1, x2, y2, (void *)bmp);
}

// for the touch panel
static void lcd_on_touch(gfx::point16 *out_locations, size_t *in_out_locations_size, void *state) {
    static uint32_t touch_ts = 0;
    if (millis() > touch_ts + 50) {
        touch_ts = millis();
        touch.update();
    }
    // UIX supports multiple touch points. so does the FT6336 so we potentially have
    // two values
    *in_out_locations_size = 0;
    uint16_t x, y;
    if (touch.xy(&x, &y)) {
        // printf("xy: (%d,%d)\n",x,y);
        out_locations[0] = gfx::point16(x, y);
        ++*in_out_locations_size;
        if (touch.xy2(&x, &y)) {
            // printf("xy2: (%d,%d)\n",x,y);
            out_locations[1] = gfx::point16(x, y);
            ++*in_out_locations_size;
        }
    }
}
// initialize the screen using the esp panel API
static void lcd_panel_init() {
    spi_bus_config_t buscfg;
    memset(&buscfg, 0, sizeof(buscfg));
    buscfg.sclk_io_num = 18;
    buscfg.mosi_io_num = 23;
    buscfg.miso_io_num = -1;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = sizeof(lcd_transfer_buffer1) + 8;

    // Initialize the SPI bus on VSPI (SPI3)
    spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);

    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config;
    memset(&io_config, 0, sizeof(io_config));
    io_config.dc_gpio_num = 15, io_config.cs_gpio_num = 5, io_config.pclk_hz = 40 * 1000 * 1000,
    io_config.lcd_cmd_bits = 8, io_config.lcd_param_bits = 8, io_config.spi_mode = 0,
    io_config.trans_queue_depth = 10, io_config.on_color_trans_done = lcd_flush_ready;
    // Attach the LCD to the SPI bus
    esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI3_HOST, &io_config, &io_handle);

    lcd_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config;
    memset(&panel_config, 0, sizeof(panel_config));
    panel_config.reset_gpio_num = -1;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    panel_config.rgb_endian = LCD_RGB_ENDIAN_BGR;
#else
    panel_config.color_space = ESP_LCD_COLOR_SPACE_BGR;
#endif
    panel_config.bits_per_pixel = 16;

    // Initialize the LCD configuration
    if(ESP_OK!=esp_lcd_new_panel_ili9342(io_handle, &panel_config, &lcd_handle)) {
        printf("Error initializing LCD panel.\n");
        while(1);
    }

    // Reset the display
    esp_lcd_panel_reset(lcd_handle);

    // Initialize LCD panel
    esp_lcd_panel_init(lcd_handle);
    // esp_lcd_panel_io_tx_param(io_handle, LCD_CMD_SLPOUT, NULL, 0);
    //  Swap x and y axis (Different LCD screens may need different options)
    esp_lcd_panel_swap_xy(lcd_handle, false);
    esp_lcd_panel_set_gap(lcd_handle, 0, 0);
    esp_lcd_panel_mirror(lcd_handle, false, false);
    esp_lcd_panel_invert_color(lcd_handle, true);
    // Turn on the screen
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    esp_lcd_panel_disp_on_off(lcd_handle, false);
#else
    esp_lcd_panel_disp_off(lcd_handle, false);
#endif
}
static void processing_task(void *param);

static void processing_task(void *param) {
    const TickType_t xMaxBlockTime = pdMS_TO_TICKS(100);
    while (true) {
        // wait for some samples to process
        uint32_t ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
        if (ulNotificationValue > 0) {
            const int16_t *input = main_sampler.buffer();
            main_processor.update(input);
            main_analyzer.samples(main_processor.samples());
            main_analyzer.fft(main_processor.fft());
            xTaskNotify(processing_task_handle, 0, eSetValueWithOverwrite);
            xTaskNotify(drawing_task_handle, 1, eSetValueWithOverwrite);
        }
    }
}
static void drawing_task(void *param) {
    const TickType_t xMaxBlockTime = pdMS_TO_TICKS(1000);
    int frames = 0;
    uint32_t fps_ts = millis();
    unsigned long ms = 0;
    while (true) {

        // wait to be told to redraw
        uint32_t ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
        if (ulNotificationValue != 0) {
            uint32_t start_ts = millis();
            main_analyzer.invalidate();
            main_screen.update();
            uint32_t end_ts = millis();
            ms+=(end_ts-start_ts);
            ++frames;
            if(millis()>=fps_ts+1000) {
                fps_ts = millis();
                printf("FPS: %d / Avg: %lums\n",frames,frames>0?ms/frames:-1);
                frames = 0;
                ms = 0;
            }

            xTaskNotify(drawing_task_handle, 0, eSetValueWithOverwrite);
        }
    }
    
}
#ifdef ARDUINO
void setup() {
    setCpuFrequencyMhz(240);
    Serial.begin(115200);
#else
extern "C" void app_main() {
#endif
    power.initialize();
    power.lcd_voltage(3);
    lcd_panel_init();
    touch.initialize();
    main_screen.background_color(screen_t::pixel_type(0,0,6));
    main_screen.on_flush_callback(lcd_on_flush);
    main_screen.on_touch_callback(lcd_on_touch);
    main_analyzer.bounds(main_screen.bounds());
    main_screen.register_control(main_analyzer);
    // create a processing task to update the sample stream/fft
    xTaskCreatePinnedToCore(
        processing_task, "Processing Task", 4096, nullptr, 2, &processing_task_handle, 0);
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0) 
    main_sampler.initialize(I2S_NUM_0, chan_cfg, pdm_rx_cfg, processing_task_handle);
#else
    main_sampler.initialize(I2S_NUM_0, i2s_pins, i2s_config, processing_task_handle);
#endif
    // create a drawing task to update our UI
    xTaskCreatePinnedToCore(
        drawing_task, "Drawing Task", 4096, nullptr, 1, &drawing_task_handle, 1);
#ifndef ARDUINO
    int count=0;
    while(1) {
        if(count++==10) {
            vTaskDelay(5);
            count = 0;
        }
    }
#endif
}

void loop() {
    // do nothing
}

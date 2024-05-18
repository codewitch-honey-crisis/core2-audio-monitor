#if __has_include(<Arduino.h>)
#    include <Arduino.h>
#    define I2C_INTERNAL Wire1
#else
#    include <freertos/FreeRTOS.h>
#    include <freertos/task.h>
#    define I2C_INTERNAL I2C_NUM_1
#endif
#include "i2s_sampler.hpp"
#include "lcd_panel.hpp"
#include "processor.hpp"
#include "ui.hpp"
#include <ft6336.hpp>
#include <m5core2_power.hpp>
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
#    include <driver/i2s_pdm.h>
#else
#    include <driver/i2s.h>
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
    /* The default mono slot is the left slot (whose 'select pin' of the PDM microphone is pulled
       down) */
    .slot_cfg =
        {
            .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
            .slot_mode = I2S_SLOT_MODE_MONO,
            .slot_mask = I2S_PDM_SLOT_RIGHT,
        },
    .gpio_cfg =
        {
            .clk = GPIO_NUM_0,
            .din = GPIO_NUM_34,
            .invert_flags =
                {
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
static i2s_pin_config_t i2s_pins = {.mck_io_num = I2S_PIN_NO_CHANGE,
                                    .bck_io_num = GPIO_NUM_12,
                                    .ws_io_num = GPIO_NUM_0,
                                    .data_out_num = I2S_PIN_NO_CHANGE,
                                    .data_in_num = GPIO_NUM_34};
#endif
static i2s_sampler<WINDOW_SIZE> main_sampler;
static processor<WINDOW_SIZE> main_processor;
static TaskHandle_t processing_task_handle;
static TaskHandle_t drawing_task_handle;

// the screen/control definitions
screen_t main_screen({320, 240}, 32 * 1024, lcd_transfer_buffer1, lcd_transfer_buffer2);
analyzer_box_t main_analyzer(main_screen);

// for the touch panel
static void lcd_on_touch(gfx::point16 *out_locations, size_t *in_out_locations_size, void *state) {
    static uint32_t touch_ts = 0;
    if (millis() > touch_ts + 13) {
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
            ms += (end_ts - start_ts);
            ++frames;
            if (millis() >= fps_ts + 1000) {
                fps_ts = millis();
                int fps = frames==0?-1:roundf(1000.0f/((float)(ms/(float)frames)));
                int ms_avg = frames==0?-1:ms/frames;
                printf("FPS: %d / Avg: %lums\n",fps,ms_avg);
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
    main_screen.background_color(gfx::color<typename screen_t::pixel_type>::black);
    main_screen.on_touch_callback(lcd_on_touch);
    main_analyzer.bounds(main_screen.bounds());
    main_screen.register_control(main_analyzer);
    lcd_active_screen(&main_screen);
    // create a processing task to update the sample stream/fft
    xTaskCreatePinnedToCore(
        processing_task, "Processing Task", 1024, nullptr, 2, &processing_task_handle, 0);
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    main_sampler.initialize(I2S_NUM_0, chan_cfg, pdm_rx_cfg, processing_task_handle);
#else
    main_sampler.initialize(I2S_NUM_0, i2s_pins, i2s_config, processing_task_handle);
#endif
    // create a drawing task to update our UI
    // need 4096 words for printf
    xTaskCreatePinnedToCore(
        drawing_task, "Drawing Task", 4096, nullptr, 1, &drawing_task_handle, 1);
#ifndef ARDUINO
    // don't need this thread.
    vTaskDelete(NULL);
#endif
}

void loop() {
    // getting rid of this clears up CPU
    vTaskDelete(NULL);
}

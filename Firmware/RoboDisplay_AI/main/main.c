#include <stdio.h>
#include <inttypes.h>
#include "esp_log.h"
#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_ili9341.h"
#include "esp_lcd_touch_xpt2046.h"
#include "lvgl.h"
#include "esp_lvgl_port.h"

#include "driver/ledc.h"

static const char *TAG = "CYD_HELLO";

// LCD Pins
#define LCD_HOST       SPI2_HOST
#define PIN_NUM_LCD_SCLK   14
#define PIN_NUM_LCD_MOSI   13
#define PIN_NUM_LCD_MISO   12
#define PIN_NUM_LCD_CS     15
#define PIN_NUM_LCD_DC     2
#define PIN_NUM_LCD_PCLK   20 * 1000 * 1000

#define PIN_NUM_BK_LIGHT   21
#define LCD_BK_LIGHT_ON_LEVEL 1

#define LCD_H_RES 240
#define LCD_V_RES 320

// Touch Pins
#define TOUCH_HOST          SPI3_HOST
#define PIN_NUM_TOUCH_SCLK  25
#define PIN_NUM_TOUCH_MOSI  32
#define PIN_NUM_TOUCH_MISO  39
#define PIN_NUM_TOUCH_CS    33
#define PIN_NUM_TOUCH_INT   36

// RGB LED Pins
#define LEDC_LS_TIMER          LEDC_TIMER_0
#define LEDC_LS_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_LS_CH0_GPIO       (4)
#define LEDC_LS_CH0_CHANNEL    LEDC_CHANNEL_0
#define LEDC_LS_CH1_GPIO       (16)
#define LEDC_LS_CH1_CHANNEL    LEDC_CHANNEL_1
#define LEDC_LS_CH2_GPIO       (17)
#define LEDC_LS_CH2_CHANNEL    LEDC_CHANNEL_2

#define LEDC_DUTY_RES          LEDC_TIMER_13_BIT
#define LEDC_FREQUENCY         (5000)

static int click_count = 0;
static lv_obj_t *count_label;

static void ledc_init(void)
{
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY,
        .speed_mode = LEDC_LS_MODE,
        .timer_num = LEDC_LS_TIMER,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel[3] = {
        {
            .channel    = LEDC_LS_CH0_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_LS_CH0_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER,
            .flags.output_invert = 1 // Active low
        },
        {
            .channel    = LEDC_LS_CH1_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_LS_CH1_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER,
            .flags.output_invert = 1 // Active low
        },
        {
            .channel    = LEDC_LS_CH2_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_LS_CH2_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER,
            .flags.output_invert = 1 // Active low
        },
    };

    for (int i = 0; i < 3; i++) {
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel[i]));
    }
}

static void rainbow_task(void *pvParameters)
{
    uint32_t hue = 0;
    uint32_t r, g, b;
    const uint32_t max_duty = (1 << 13) - 1;

    while (1) {
        // Very simple HSV to RGB (simplified for hue only)
        uint32_t region = hue / 43;
        uint32_t remainder = (hue % 43) * 6;

        uint32_t p = 0;
        uint32_t q = 255 - remainder;
        uint32_t t = remainder;

        switch (region) {
            case 0: r = 255; g = t;   b = p;   break;
            case 1: r = q;   g = 255; b = p;   break;
            case 2: r = p;   g = 255; b = t;   break;
            case 3: r = p;   g = q;   b = 255; break;
            case 4: r = t;   g = p;   b = 255; break;
            default: r = 255; g = p;   b = q;   break;
        }

        ledc_set_duty(LEDC_LS_MODE, LEDC_LS_CH0_CHANNEL, (r * max_duty) / 255);
        ledc_update_duty(LEDC_LS_MODE, LEDC_LS_CH0_CHANNEL);
        ledc_set_duty(LEDC_LS_MODE, LEDC_LS_CH1_CHANNEL, (g * max_duty) / 255);
        ledc_update_duty(LEDC_LS_MODE, LEDC_LS_CH1_CHANNEL);
        ledc_set_duty(LEDC_LS_MODE, LEDC_LS_CH2_CHANNEL, (b * max_duty) / 255);
        ledc_update_duty(LEDC_LS_MODE, LEDC_LS_CH2_CHANNEL);

        hue++;
        if (hue >= 256) hue = 0;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void btn_event_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        click_count++;
        lv_label_set_text_fmt(count_label, "Clicks: %d", click_count);
        
        lv_indev_t * indev = lv_indev_active();
        lv_point_t point;
        lv_indev_get_point(indev, &point);
        
        printf("Button pushed! Total clicks: %d | Point: x=%d, y=%d\n", click_count, (int)point.x, (int)point.y);
        ESP_LOGI(TAG, "Button pushed! Total clicks: %d | x=%d, y=%d", click_count, (int)point.x, (int)point.y);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing LCD SPI bus...");
    spi_bus_config_t lcd_buscfg = {
        .sclk_io_num = PIN_NUM_LCD_SCLK,
        .mosi_io_num = PIN_NUM_LCD_MOSI,
        .miso_io_num = PIN_NUM_LCD_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * 80 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &lcd_buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Initializing Touch SPI bus...");
    spi_bus_config_t touch_buscfg = {
        .sclk_io_num = PIN_NUM_TOUCH_SCLK,
        .mosi_io_num = PIN_NUM_TOUCH_MOSI,
        .miso_io_num = PIN_NUM_TOUCH_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(TOUCH_HOST, &touch_buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = PIN_NUM_LCD_DC,
        .cs_gpio_num = PIN_NUM_LCD_CS,
        .pclk_hz = PIN_NUM_LCD_PCLK,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    ESP_LOGI(TAG, "Install ILI9341 panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = -1,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    ESP_LOGI(TAG, "Initialize touch controller");
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_spi_config_t tp_io_config = ESP_LCD_TOUCH_IO_SPI_XPT2046_CONFIG(PIN_NUM_TOUCH_CS);
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)TOUCH_HOST, &tp_io_config, &tp_io_handle));

    esp_lcd_touch_config_t tp_cfg = {
        .x_max = LCD_H_RES,
        .y_max = LCD_V_RES,
        .rst_gpio_num = -1,
        .int_gpio_num = PIN_NUM_TOUCH_INT,
        .flags = {
            .swap_xy = 1,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };
    esp_lcd_touch_handle_t tp_handle = NULL;
    ESP_ERROR_CHECK(esp_lcd_touch_new_spi_xpt2046(tp_io_handle, &tp_cfg, &tp_handle));

    ESP_LOGI(TAG, "Turn on backlight");
    gpio_set_direction(PIN_NUM_BK_LIGHT, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL);

    ESP_LOGI(TAG, "Initialize LEDC for RGB LED");
    ledc_init();
    xTaskCreate(rainbow_task, "rainbow_task", 2048, NULL, 5, NULL);

    ESP_LOGI(TAG, "Initialize LVGL port");
    lvgl_port_cfg_t port_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    port_cfg.task_priority = 10;
    port_cfg.timer_period_ms = 5;
    ESP_ERROR_CHECK(lvgl_port_init(&port_cfg));

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = LCD_H_RES * 40,
        .double_buffer = true,
        .hres = LCD_H_RES,
        .vres = LCD_V_RES,
        .monochrome = false,
        .rotation = {
            .swap_xy = false,
            .mirror_x = true,
            .mirror_y = false,
        },
        .flags = {
            .buff_dma = true,
        }
    };
    lv_display_t *disp = lvgl_port_add_disp(&disp_cfg);

    const lvgl_port_touch_cfg_t touch_cfg = {
        .disp = disp,
        .handle = tp_handle,
    };
    lvgl_port_add_touch(&touch_cfg);

    ESP_LOGI(TAG, "Display UI");
    lv_obj_t *screen = lv_display_get_screen_active(disp);
    
    lv_obj_t *title = lv_label_create(screen);
    lv_label_set_text(title, "CYD Click Counter");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);

    count_label = lv_label_create(screen);
    lv_label_set_text(count_label, "Clicks: 0");
    lv_obj_align(count_label, LV_ALIGN_TOP_MID, 0, 50);

    lv_obj_t *btn = lv_button_create(screen);
    lv_obj_set_size(btn, 120, 50); 
    lv_obj_center(btn);
    lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_CLICKED, NULL);
    
    // Change color on press
    lv_obj_set_style_bg_color(btn, lv_palette_main(LV_PALETTE_RED), LV_STATE_PRESSED);

    lv_obj_t *btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Push me");
    lv_obj_center(btn_label);

    ESP_LOGI(TAG, "Done");

    while (1) {
        printf("Hello World\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

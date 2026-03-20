#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_ili9341.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_touch_xpt2046.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"

static const char *TAG = "CYD";

// Pin definitions for LCD (SPI2)
#define LCD_MOSI_PIN  13
#define LCD_SCK_PIN   14
#define LCD_MISO_PIN  12
#define LCD_CS_PIN    15
#define LCD_DC_PIN    2
#define LCD_BACKLIGHT_PIN 21

// Pin definitions for Touch (SPI3)
#define TOUCH_MOSI_PIN  32
#define TOUCH_MISO_PIN 39
#define TOUCH_SCK_PIN   25
#define TOUCH_CS_PIN    33

// Display dimensions
#define LCD_WIDTH  320
#define LCD_HEIGHT 240

// Global variables
static uint32_t click_count = 0;
static lv_obj_t *click_count_label = NULL;

// Button callback
static void button_event_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        click_count++;
        
        // Update the label
        char buf[32];
        snprintf(buf, sizeof(buf), "Clicks: %lu", (unsigned long)click_count);
        lv_label_set_text(click_count_label, buf);
        
        // Print to serial
        printf("I'm pushed! Total clicks: %lu\n", (unsigned long)click_count);
    }
}

// Periodic Hello World task
static void hello_world_task(void *pvParameter)
{
    while(1) {
        printf("Hello World\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    // Initialize SPI bus for LCD (SPI2)
    spi_bus_config_t lcd_spi_bus_config = {
        .mosi_io_num = LCD_MOSI_PIN,
        .miso_io_num = LCD_MISO_PIN,
        .sclk_io_num = LCD_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_WIDTH * LCD_HEIGHT * 2,
    };
    
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &lcd_spi_bus_config, SPI_DMA_CH_AUTO));
    
    // Configure LCD panel IO
    esp_lcd_panel_io_handle_t lcd_io = NULL;
    esp_lcd_panel_io_spi_config_t lcd_io_config = {
        .dc_gpio_num = LCD_DC_PIN,
        .cs_gpio_num = LCD_CS_PIN,
        .pclk_hz = 40 * 1000 * 1000,
        .trans_queue_depth = 10,
        .on_color_trans_done = NULL,
        .user_ctx = NULL,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &lcd_io_config, &lcd_io));
    
    // Configure ILI9341 panel
    esp_lcd_panel_handle_t lcd_panel = NULL;
    esp_lcd_panel_dev_config_t lcd_panel_config = {
        .reset_gpio_num = -1,
        .color_space = ESP_LCD_COLOR_SPACE_RGB,
        .bits_per_pixel = 16,
    };
    
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(lcd_io, &lcd_panel_config, &lcd_panel));
    
    // Initialize LCD panel
    ESP_ERROR_CHECK(esp_lcd_panel_reset(lcd_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(lcd_panel));
    
    // Configure mirroring for correct orientation (mirror_x = true for horizontal mirroring)
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(lcd_panel, true, false));
    
    // Turn on backlight
    gpio_set_direction(LCD_BACKLIGHT_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LCD_BACKLIGHT_PIN, 1);
    
    // Initialize SPI bus for Touch (SPI3)
    spi_bus_config_t touch_spi_bus_config = {
        .mosi_io_num = TOUCH_MOSI_PIN,
        .miso_io_num = TOUCH_MISO_PIN,
        .sclk_io_num = TOUCH_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &touch_spi_bus_config, SPI_DMA_CH_AUTO));
    
    // Configure XPT2046 touch panel IO
    esp_lcd_panel_io_handle_t touch_io = NULL;
    esp_lcd_panel_io_spi_config_t touch_io_config = {
        .cs_gpio_num = TOUCH_CS_PIN,
        .dc_gpio_num = GPIO_NUM_NC,
        .pclk_hz = 1 * 1000 * 1000,
        .trans_queue_depth = 3,
        .on_color_trans_done = NULL,
        .user_ctx = NULL,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI3_HOST, &touch_io_config, &touch_io));
    
    // Configure XPT2046 touch
    esp_lcd_touch_handle_t touch_handle = NULL;
    esp_lcd_touch_config_t touch_config = {
        .x_max = LCD_WIDTH,
        .y_max = LCD_HEIGHT,
        .rst_gpio_num = -1,
        .int_gpio_num = -1,
        .levels.reset = 0,
        .flags.swap_xy = 1,  // Enable XY swapping for correct coordinate mapping
        .flags.mirror_x = 0,
        .flags.mirror_y = 0,
    };
    
    // Initialize XPT2046 touch controller
    ESP_ERROR_CHECK(esp_lcd_touch_new_spi_xpt2046(touch_io, &touch_config, &touch_handle));
    
    // Initialize LVGL port
    lvgl_port_cfg_t lvgl_cfg = {
        .task_priority = 4,
        .task_stack = 4096,
        .task_affinity = -1,
        .task_max_sleep_ms = 500,
        .timer_period_ms = 5,
    };
    ESP_ERROR_CHECK(lvgl_port_init(&lvgl_cfg));
    
    // Add LCD to LVGL port
    lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = lcd_io,
        .panel_handle = lcd_panel,
        .buffer_size = LCD_WIDTH * LCD_HEIGHT * 2,
        .double_buffer = false,
        .hres = LCD_WIDTH,
        .vres = LCD_HEIGHT,
        .monochrome = false,
        .rotation = {
            .swap_xy = false,
            .mirror_x = true,  // Mirror X for correct orientation
            .mirror_y = false,
        },
        .flags = {
            .buff_dma = 1,
            .buff_spiram = 0,
        },
    };
    lv_disp_t * display = lvgl_port_add_disp(&disp_cfg);
    
    // Configure display rotation
    lv_disp_set_rotation(display, LV_DISP_ROT_NONE);
    
    // Add touch to LVGL
    lvgl_port_touch_cfg_t touch_cfg = {
        .disp = display,
        .handle = touch_handle,
    };
    lvgl_port_add_touch(&touch_cfg);
    
    // Wait for display to be ready
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Create UI
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_make(0, 0, 0), LV_PART_MAIN);
    
    // Title label
    lv_obj_t *title_label = lv_label_create(scr);
    lv_label_set_text(title_label, "CYD Click Counter");
    lv_obj_set_style_text_color(title_label, lv_color_make(255, 255, 255), LV_PART_MAIN);
    lv_obj_align(title_label, LV_ALIGN_TOP_MID, 0, 10);
    
    // Click counter label
    click_count_label = lv_label_create(scr);
    char buf[32];
    snprintf(buf, sizeof(buf), "Clicks: %lu", (unsigned long)click_count);
    lv_label_set_text(click_count_label, buf);
    lv_obj_set_style_text_color(click_count_label, lv_color_make(255, 255, 0), LV_PART_MAIN);
    lv_obj_align(click_count_label, LV_ALIGN_TOP_MID, 0, 50);
    
    // Button
    lv_obj_t *btn = lv_btn_create(scr);
    lv_obj_set_size(btn, 120, 50);
    lv_obj_align(btn, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_event_cb(btn, button_event_handler, LV_EVENT_CLICKED, NULL);
    
    // Button label
    lv_obj_t *btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Push me");
    lv_obj_set_style_text_color(btn_label, lv_color_make(0, 0, 0), LV_PART_MAIN);
    lv_obj_center(btn_label);
    
    // Button styles
    lv_obj_set_style_bg_color(btn, lv_color_make(0, 255, 0), LV_PART_MAIN);
    lv_obj_set_style_bg_color(btn, lv_color_make(0, 200, 0), LV_STATE_PRESSED);
    
    // Start Hello World task
    xTaskCreate(&hello_world_task, "hello_world_task", 2048, NULL, 3, NULL);
    
    ESP_LOGI(TAG, "CYD Hello World initialized!");
    printf("Hello World\n");
    
    // Main loop - LVGL tasks
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10));
        lvgl_port_lock(0);
        lv_task_handler();
        lvgl_port_unlock();
    }
}

idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES esp_lcd_ili9341 esp_lvgl_port esp_lcd_touch esp_lcd_touch_xpt2046 lvgl
)

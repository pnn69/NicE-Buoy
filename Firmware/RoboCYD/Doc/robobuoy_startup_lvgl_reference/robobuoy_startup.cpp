#include "robobuoy_startup.h"

static lv_obj_t *spinner;
static lv_obj_t *wifiIcon;
static lv_obj_t *loraIcon;

void robobuoy_boot_screen()
{
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), 0);

    spinner = lv_spinner_create(lv_scr_act(), 1200, 60);
    lv_obj_set_size(spinner, 140, 140);
    lv_obj_align(spinner, LV_ALIGN_TOP_MID, 0, 20);

    wifiIcon = lv_label_create(lv_scr_act());
    lv_label_set_text(wifiIcon, LV_SYMBOL_WIFI);
    lv_obj_align(wifiIcon, LV_ALIGN_BOTTOM_LEFT, 40, -20);

    loraIcon = lv_label_create(lv_scr_act());
    lv_label_set_text(loraIcon, LV_SYMBOL_UPLOAD);
    lv_obj_align(loraIcon, LV_ALIGN_BOTTOM_RIGHT, -40, -20);
}

void wifi_state_disconnected(){ lv_obj_set_style_text_color(wifiIcon, lv_color_hex(0x606060),0);}
void wifi_state_connected(){ lv_obj_set_style_text_color(wifiIcon, lv_color_hex(0x00FF40),0);}
void wifi_state_ap(){ lv_obj_set_style_text_color(wifiIcon, lv_color_hex(0x0088FF),0);}
void lora_rx(){ lv_obj_set_style_text_color(loraIcon, lv_color_hex(0x00FF40),0);}
void lora_tx(){ lv_obj_set_style_text_color(loraIcon, lv_color_hex(0xFF0000),0);}

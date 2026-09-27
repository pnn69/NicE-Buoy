idf_component_register(
    SRCS
        "main.cpp"
        "oled.cpp"
        "lora.cpp"
        "packet_queue.cpp"
        "wifi.cpp"
        "udp.cpp"

    INCLUDE_DIRS
        "."
        "../../RoboDependency"

    REQUIRES
        driver
        esp_wifi
        esp_netif
        esp_event
        nvs_flash
        lwip
)
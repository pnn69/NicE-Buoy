idf_component_register(
    SRCS
        "main.cpp"
        "oled.cpp"
        "lora.cpp"
        "packet_queue.cpp"
        "wifi.cpp"
        "udp.cpp"
        "router.cpp"

    INCLUDE_DIRS
        "."
        "../../RoboDependency"
        "../../RoboDependency/RoboCompute/src"

    REQUIRES
        driver
        esp_wifi
        esp_netif
        esp_event
        nvs_flash
        lwip
        espressif__iot_bridge
        espressif__mesh_lite
)
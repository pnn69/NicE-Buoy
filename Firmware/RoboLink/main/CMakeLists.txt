idf_component_register(
    SRCS
        "main.cpp"
        "oled.cpp"
        "lora.cpp"
        "packet_queue.cpp"

    INCLUDE_DIRS
        "."

    REQUIRES
        driver
)
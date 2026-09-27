idf_component_register(
    SRCS
        "main.cpp"
        "oled.cpp"
        "lora.cpp"

    INCLUDE_DIRS
        "."

    REQUIRES
        driver
)
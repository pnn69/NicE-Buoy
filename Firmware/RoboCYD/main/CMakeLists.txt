idf_component_register(
    SRCS
        "migration_main.cpp"
        "../src/main.cpp"
        "../src/cyd_display.cpp"
        "../src/cyd_touch.cpp"
        "../src/cyd_wifi.cpp"
        "../src/cyd_lora.cpp"
        "../src/boot_screen.cpp"
        "../src/buoy_data.cpp"
        "../src/RGBledDriver.cpp"
        "../src/robo_mesh.cpp"

    INCLUDE_DIRS
        "."
        "../src"

    REQUIRES
        arduino-esp32
        TFT_eSPI
        XPT2046_Bitbang_Slim
        arduinoWebSockets
        arduino-LoRa
)
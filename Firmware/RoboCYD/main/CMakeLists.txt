idf_component_register(
    SRCS
        "migration_main.cpp"
        "../src/cyd_display.cpp"
        "../src/cyd_touch.cpp"

    INCLUDE_DIRS
        "."
        "../src"

    REQUIRES
        arduino-esp32
        TFT_eSPI
        XPT2046_Bitbang_Slim
)
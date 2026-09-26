idf_component_register(
    SRCS
        "src/WebSockets.cpp"
        "src/WebSocketsServer.cpp"
        "src/WebSocketsClient.cpp"
        "src/SocketIOclient.cpp"
        "src/libb64/cdecode.c"
        "src/libb64/cencode.c"
        "src/libsha1/libsha1.c"

    INCLUDE_DIRS
        "src"

    REQUIRES
        arduino-esp32
)
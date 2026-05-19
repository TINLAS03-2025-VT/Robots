cmake_minimum_required(VERSION 3.12)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_PLATFORM rp2350)
set(PICO_BOARD pico2_w)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(main C CXX ASM)

pico_sdk_init()

link_directories(libmicroros)
add_executable(main
    main.c
    interface/common/transport_common.c
    interface/wifi/pico_wifi_transport.c
    interface/uart/pico_uart_transport.c
)
target_link_libraries(main
    pico_stdlib
    microros
    pico_cyw43_arch_lwip_threadsafe_background
)

target_include_directories(main PUBLIC
    libmicroros/include
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/..
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    WIFI_SSID="${WIFI_SSID}"
    WIFI_PASSWORD="${WIFI_PASSWORD}"
    AGENT_IP="${AGENT_IP}"
)

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Configure Pico
pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate UF2
pico_add_extra_outputs(main)

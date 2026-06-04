cmake_minimum_required(VERSION 3.12)
set(PICO_BOARD pico2_w)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
project(pico_micro_ros_example C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()
link_directories(libmicroros)
add_executable(pico_micro_ros_example
    pico_micro_ros_example.c
    picow_udp_transports.c
	robot/movement.c
)
target_link_libraries(pico_micro_ros_example
    pico_stdlib
    microros
    pico_cyw43_arch_lwip_threadsafe_background
	hardware_pwm
	m
)
target_include_directories(pico_micro_ros_example
    PRIVATE
	  ${CMAKE_CURRENT_LIST_DIR}
	  ${CMAKE_CURRENT_LIST_DIR}/robot
    PUBLIC
      libmicroros/include
)

target_compile_definitions(pico_micro_ros_example PRIVATE
    WIFI_SSID="${WIFI_SSID}"
    WIFI_PASSWORD="${WIFI_PASSWORD}"
    AGENT_IP="${AGENT_IP}"
    AGENT_PORT=${AGENT_PORT}
    MICRO_ROS_CLIENT_KEY=${MICRO_ROS_CLIENT_KEY}
)

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")
# Configure Pico
pico_enable_stdio_usb(pico_micro_ros_example 1)
pico_enable_stdio_uart(pico_micro_ros_example 1)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)
# Generate UF2
pico_add_extra_outputs(pico_micro_ros_example)

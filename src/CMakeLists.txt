cmake_minimum_required(VERSION 3.12)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(TARGET_NAME pico_micro_ros_example)

add_executable(${TARGET_NAME}
    pico_micro_ros_example.c
    picow_udp_transports.c
    robot/movement.c
    robot/rff-algorithm.c
)

target_link_libraries(${TARGET_NAME}
    hardware_pwm
    m
    microros
    pico_cyw43_arch_lwip_threadsafe_background
    pico_float
    pico_stdio
    pico_stdlib
)

target_include_directories(${TARGET_NAME}
    PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/robot
)

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -Wall -Wextra -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra -ffunction-sections -fdata-sections")

# Configure Pico
pico_enable_stdio_usb(${TARGET_NAME} 1)
pico_enable_stdio_uart(${TARGET_NAME} 0)

# Generate UF2
pico_add_extra_outputs(${TARGET_NAME})

cmake_minimum_required(VERSION 3.12)

# 1. Global Setup Configuration
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT DEFINED PICO_BOARD)
    set(PICO_BOARD pico2_w)
endif()

# Use local SDK downloaded by setup.cmake
set(LOCAL_EXT_PATH "${CMAKE_CURRENT_LIST_DIR}/external")
set(PICO_SDK_PATH "${LOCAL_EXT_PATH}/pico-sdk")
set(LIBMICROROS_PATH "${LOCAL_EXT_PATH}/libmicroros/libmicroros")
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

set(picotool_DIR "${CMAKE_CURRENT_LIST_DIR}/external/picotool/build/install")

project(pico_micro_ros_workspace C CXX ASM)

# 2. Global Pico Core Init
pico_sdk_init()

# 3. Global Compile Macro Overrides (applies to all targets automatically)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# 4. Global Local Micro-ROS Directory Hook
link_directories(${LIBMICROROS_PATH})

include_directories(
    ${LIBMICROROS_PATH}/include
    ${CMAKE_CURRENT_LIST_DIR}
)

# 5. Load Targets
add_subdirectory(src)

# 6. Target to add compile_commands.json to root dir
add_custom_target(copy_compile_commands ALL
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_BINARY_DIR}/compile_commands.json"
        "${CMAKE_SOURCE_DIR}/compile_commands.json"
    COMMENT "Syncing compile_commands.json to root for clangd LSP support"
)
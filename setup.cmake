# ==============================================================================
# Self-Contained Cross-Platform Setup Script for micro-ROS Pico SDK
# Run via: cmake -P setup.cmake
# ==============================================================================

message("--- Starting Self-Contained Repository Setup ---")

# 1. Verify secrets.h exists before trying to build
if(NOT EXISTS "${CMAKE_CURRENT_LIST_DIR}/secrets.h")
    message(FATAL_ERROR "Missing secrets.h! Please copy secrets.h.template to secrets.h and fill in your credentials.")
endif()

# 2. Verify ARM Toolchain / Compiler
message("Checking for arm-none-eabi-gcc toolchain...")
find_program(ARM_GCC arm-none-eabi-gcc)
if(NOT ARM_GCC AND NOT DEFINED ENV{PICO_TOOLCHAIN_PATH})
    message(FATAL_ERROR "The 'arm-none-eabi-gcc' compiler is required but not found in system PATH. Please install it or set the PICO_TOOLCHAIN_PATH environment variable.")
endif()

# 3. Define Local Paths
set(LOCAL_EXT_DIR "${CMAKE_CURRENT_LIST_DIR}/external")
set(PICO_SDK_PATH "${LOCAL_EXT_DIR}/pico-sdk")
set(MICRO_ROS_PATH "${LOCAL_EXT_DIR}/libmicroros")
set(PICOTOOL_SRC_DIR "${LOCAL_EXT_DIR}/picotool")
set(PICOTOOL_BUILD_DIR "${PICOTOOL_SRC_DIR}/build")
set(BUILD_DIR "${CMAKE_CURRENT_LIST_DIR}/build")

# 4a. Fetch Pico SDK Dependency
if(NOT EXISTS "${PICO_SDK_PATH}/CMakeLists.txt")
    message("Pico SDK not found locally. Fetching submodules into external/...")
    file(MAKE_DIRECTORY "${LOCAL_EXT_DIR}")
    execute_process(
        COMMAND git clone --recurse-submodules https://github.com/raspberrypi/pico-sdk.git ${PICO_SDK_PATH}
        RESULT_VARIABLE git_result
    )
    if(NOT git_result EQUAL 0)
        message(FATAL_ERROR "Failed to clone Pico SDK.")
    endif()
endif()

# ------------------------------------------------------------------------------
# 4b. FETCH PRECOMPILED MICRO-ROS LIBRARY
# ------------------------------------------------------------------------------

if(NOT EXISTS "${MICRO_ROS_PATH}/libmicroros/include")
    message("micro-ROS library not found locally. Fetching precompiled binaries...")
    
    # We clone the official micro-ROS Pico precompiled repository directly into external/
    execute_process(
        COMMAND git clone https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk.git ${MICRO_ROS_PATH}
        RESULT_VARIABLE uros_result
    )
    if(NOT uros_result EQUAL 0)
        message(FATAL_ERROR "Failed to clone micro-ROS library assets.")
    endif()
else()
    message("micro-ROS library already found locally at ${MICRO_ROS_PATH}")
endif()

# ------------------------------------------------------------------------------
# 4c. FETCH AND BUILD NATIVE PICOTOOL WITH USB SUPPORT
# ------------------------------------------------------------------------------
if(NOT EXISTS "${PICOTOOL_SRC_DIR}/CMakeLists.txt")
    message("--- Fetching Picotool Repository ---")
    execute_process(
        COMMAND git clone https://github.com/raspberrypi/picotool.git ${PICOTOOL_SRC_DIR}
        RESULT_VARIABLE picotool_clone_res
    )
    if(NOT picotool_clone_res EQUAL 0)
        message(FATAL_ERROR "Failed to clone picotool asset tree.")
    endif()
endif()

# ------------------------------------------------------------------------------
# 4c. FETCH AND BUILD NATIVE PICOTOOL WITH USB SUPPORT
# ------------------------------------------------------------------------------
if(NOT EXISTS "${PICOTOOL_SRC_DIR}/CMakeLists.txt")
    message("--- Fetching Picotool Repository ---")
    execute_process(
        COMMAND git clone https://github.com/raspberrypi/picotool.git ${PICOTOOL_SRC_DIR}
        RESULT_VARIABLE picotool_clone_res
    )
    if(NOT picotool_clone_res EQUAL 0)
        message(FATAL_ERROR "Failed to clone picotool asset tree.")
    endif()
endif()

# Explicit Step to Compile the Full System Standalone version of Picotool
if(NOT EXISTS "${PICOTOOL_BUILD_DIR}/picotool")
    file(MAKE_DIRECTORY "${PICOTOOL_BUILD_DIR}")
    message("--- Pre-building Independent Full Picotool with USB support ---")

    # CROSS-PLATFORM SYSTEM FIX:
    # Forces C++ standard headers to evaluate before picoboot.h pollutes the '__unused' macro.
    set(PICOTOOL_CXX_FLAGS "-include iostream")

    execute_process(
        COMMAND ${CMAKE_COMMAND}
            -DPICO_SDK_PATH=${PICO_SDK_PATH}
            -DCMAKE_CXX_FLAGS=${PICOTOOL_CXX_FLAGS}
            ..
        WORKING_DIRECTORY "${PICOTOOL_BUILD_DIR}"
        RESULT_VARIABLE picotool_config_res
    )

    if(NOT picotool_config_res EQUAL 0)
        message(FATAL_ERROR "Failed to configure standalone full-feature picotool application.")
    endif()

    execute_process(
        COMMAND ${CMAKE_COMMAND} --build .
        WORKING_DIRECTORY "${PICOTOOL_BUILD_DIR}"
        RESULT_VARIABLE picotool_build_res
    )
    if(NOT picotool_build_res EQUAL 0)
        message(FATAL_ERROR "Failed to compile standalone full-feature picotool application.")
    endif()
endif()

# Point to this specific binary so the core build process doesn't rebuild a limited version
set(picotool_DIR "${PICOTOOL_BUILD_DIR}" CACHE PATH "Path to full feature picotool" FORCE)

# 5. Configure CMake Build Cache
if(NOT EXISTS "${BUILD_DIR}")
    file(MAKE_DIRECTORY "${BUILD_DIR}")
endif()

message("--- Configuring Build Cache ---")
execute_process(
	COMMAND ${CMAKE_COMMAND} -DPICO_SDK_PATH=${PICO_SDK_PATH} ${CMAKE_CURRENT_LIST_DIR}
    WORKING_DIRECTORY "${BUILD_DIR}"
    RESULT_VARIABLE cmake_result
)
if(NOT cmake_result EQUAL 0)
    message(FATAL_ERROR "CMake configuration failed.")
endif()

# 6. Compile Agnostically
message("--- Compiling Project ---")
execute_process(
    COMMAND ${CMAKE_COMMAND} --build .
    WORKING_DIRECTORY "${BUILD_DIR}"
    RESULT_VARIABLE build_result
)

if(build_result EQUAL 0)
    message("--- Setup and Build Completed Successfully! ---")
else()
    message(FATAL_ERROR "Compilation failed.")
endif()

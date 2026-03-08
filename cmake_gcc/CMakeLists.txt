# Define minimal required version of CMake.
cmake_minimum_required(VERSION "3.20")

# Include toolchain before project() call
include(toolchain.cmake)

# Project definition
project(
	wifi_gspi_merged
	VERSION 1.0
	LANGUAGES C CXX ASM
)

# Include the definition of the slc_wifi_gspi_merged target,
# which contains the content of the SLC project
include(wifi_gspi_merged.cmake)

#=============================================================================
# ELRS C++ Platform Abstraction Layer for SiW917
#=============================================================================

# SiW917 platform abstraction files (Arduino compatibility layer)
# Now using elrs_cpp folder (the clean C++ implementation)
set(ELRS_PLATFORM_SOURCES
    "../src/elrs_cpp/hal/Arduino.cpp"
    "../src/elrs_cpp/hal/SPIEx.cpp"
    "../src/elrs_cpp/hal/hwTimer.cpp"
    "../src/elrs_cpp/hal/LR1121_hal.cpp"
    "../src/elrs_cpp/elrs_main.cpp"
    "../src/elrs_cpp/common.cpp"
    # CT-based hardware timer (C implementation wrapped by hwTimer.cpp)
    "../src/hw_timer.c"
    # Hardware timer test suite
    "../src/hw_timer_test.c"
)

# ELRS C++ library sources from our elrs_cpp folder
# These are SiW917-adapted versions of the upstream ELRS code
set(ELRS_CPP_LIB_DIR "../src/elrs_cpp/lib")

set(ELRS_CPP_SOURCES
    # CRC
    "${ELRS_CPP_LIB_DIR}/CRC/crc.cpp"
    # FHSS
    "${ELRS_CPP_LIB_DIR}/FHSS/FHSS.cpp"
    "${ELRS_CPP_LIB_DIR}/FHSS/random.cpp"
    # OTA
    "${ELRS_CPP_LIB_DIR}/OTA/OTA.cpp"
    # LR1121 Driver (our SiW917 version, NOT the upstream ESP32 version)
    "${ELRS_CPP_LIB_DIR}/LR1121Driver/LR1121.cpp"
    # OPTIONS
    "${ELRS_CPP_LIB_DIR}/OPTIONS/options.cpp"
    # FEC - Forward Error Correction (Hamming codes)
    "${ELRS_CPP_LIB_DIR}/FEC/FEC.cpp"
    "${ELRS_CPP_LIB_DIR}/FEC/hamming.cpp"
    # Device stubs
    "../src/elrs_cpp/hal/device.cpp"
    # StubbornSender (for telemetry TX)
    "${ELRS_CPP_LIB_DIR}/StubbornSender/stubborn_sender.cpp"
)

#=============================================================================
# Main executable
#=============================================================================

add_executable(wifi_gspi_merged
    # SiW917 platform layer
    ${ELRS_PLATFORM_SOURCES}
    # ELRS C++ libraries (upstream)
    ${ELRS_CPP_SOURCES}
)

#=============================================================================
# Include directories for ELRS
#=============================================================================

# Get the include directories from slc and apply them to our executable
# This is needed because C++ source files need SDK headers during compilation
get_target_property(SLC_INCLUDE_DIRS slc INTERFACE_INCLUDE_DIRECTORIES)
if(SLC_INCLUDE_DIRS)
    target_include_directories(wifi_gspi_merged PUBLIC ${SLC_INCLUDE_DIRS})
endif()

# Upstream ELRS source base path
set(ELRS_UPSTREAM_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../ExpressLRS/src")

target_include_directories(wifi_gspi_merged PUBLIC
    # Our SiW917 platform headers (must come FIRST to override upstream)
    "../src/elrs_cpp/hal"
    "../src/elrs_cpp/include"
    "../src/elrs_cpp/lib/CRC"
    "../src/elrs_cpp/lib/FHSS"
    "../src/elrs_cpp/lib/OTA"
    "../src/elrs_cpp/lib/LR1121Driver"
    "../src/elrs_cpp/lib/SX12xxDriverCommon"
    "../src/elrs_cpp/lib/CrsfProtocol"
    "../src/elrs_cpp/lib/TelemetryProtocol"
    "../src/elrs_cpp/lib/LQCALC"
    "../src/elrs_cpp/lib/PFD"
    "../src/elrs_cpp/lib/FIFO"
    "../src/elrs_cpp/lib/LowPassFilter"
    "../src/elrs_cpp/lib/OPTIONS"
    "../src/elrs_cpp/lib/FEC"
    "../src/elrs_cpp/lib/StubbornSender"
    
    # Upstream ELRS include directories (for missing headers)
    "${ELRS_UPSTREAM_DIR}/include"
    "${ELRS_UPSTREAM_DIR}/lib/DEVICE"
    "${ELRS_UPSTREAM_DIR}/lib/FEC"
    "${ELRS_UPSTREAM_DIR}/lib/CrsfProtocol"
    "${ELRS_UPSTREAM_DIR}/lib/TelemetryProtocol"
    "${ELRS_UPSTREAM_DIR}/lib/LQCALC"
    "${ELRS_UPSTREAM_DIR}/lib/PFD"
    "${ELRS_UPSTREAM_DIR}/lib/FIFO"
    "${ELRS_UPSTREAM_DIR}/lib/LR1121Driver"
    "${ELRS_UPSTREAM_DIR}/lib/SX12xxDriverCommon"
    "${ELRS_UPSTREAM_DIR}/lib/OPTIONS"
    "${ELRS_UPSTREAM_DIR}/lib/OTA"
    "${ELRS_UPSTREAM_DIR}/lib/CRC"
    "${ELRS_UPSTREAM_DIR}/lib/FHSS"
    "${ELRS_UPSTREAM_DIR}/lib/logging"
    "${ELRS_UPSTREAM_DIR}/lib/MeanAccumulator"
    "${ELRS_UPSTREAM_DIR}/lib/rx-crsf"
    "${ELRS_UPSTREAM_DIR}/lib/CONFIG"
    "${ELRS_UPSTREAM_DIR}/lib/elrs_eeprom"
    "${ELRS_UPSTREAM_DIR}/lib/SPIEx"
    "${ELRS_UPSTREAM_DIR}/lib/HWTIMER"
    "${ELRS_UPSTREAM_DIR}/lib/MSP"
    "${ELRS_UPSTREAM_DIR}/lib/CRSF2MSP"
    "${ELRS_UPSTREAM_DIR}/lib/StubbornReceiver"
    "${ELRS_UPSTREAM_DIR}/lib/StubbornSender"
)

#=============================================================================
# Compile definitions
#=============================================================================

target_compile_definitions(wifi_gspi_merged PUBLIC
    # Platform identification - don't redefine if already set
    # PLATFORM_SIW917=1  # Set in targets.h with guards
    # TARGET_RX=1        # Set in targets.h with guards
    
    # Radio type - LR1121 dual-band
    # RADIO_LR1121=1     # Set in hardware.h with guards
    
    # Regulatory domain (can be overridden)
    # Regulatory_Domain_FCC_915=1  # Set in hardware.h with guards
    
    # Debug logging
    # DEBUG_LOG=1        # Set in hardware.h with guards
)

#=============================================================================
# Compile options
#=============================================================================

target_compile_options(wifi_gspi_merged PUBLIC
    # C++ specific flags (already defined in slc but ensure they apply)
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
    # Suppress some warnings from upstream ELRS code
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-unused-variable>
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-unused-parameter>
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-ignored-qualifiers>
    # Don't treat warnings as errors for ELRS code
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-error>
)

target_link_options(wifi_gspi_merged PUBLIC
    # No additional link options needed
)

# Link with the content defined in the SLC project
# Using PUBLIC so our C++ files get the include directories from slc
target_link_libraries(wifi_gspi_merged PUBLIC
    slc
)

# Include managed project content if available
include(wifi_gspi_merged_project.cmake OPTIONAL RESULT_VARIABLE managed_project)
if(managed_project)
    message(STATUS "Using managed project content from ${managed_project}")
endif()

# Force the gcc linker command
set_target_properties(wifi_gspi_merged PROPERTIES LINKER_LANGUAGE C)

# Create .bin, .hex and .s37 artifacts after building the project
add_custom_command(TARGET wifi_gspi_merged
    POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} ${OBJCOPY_SREC_CMD} "$<TARGET_FILE:wifi_gspi_merged>" "$<TARGET_FILE_DIR:wifi_gspi_merged>/$<TARGET_FILE_BASE_NAME:wifi_gspi_merged>.s37"
    COMMAND ${CMAKE_OBJCOPY} ${OBJCOPY_IHEX_CMD} "$<TARGET_FILE:wifi_gspi_merged>" "$<TARGET_FILE_DIR:wifi_gspi_merged>/$<TARGET_FILE_BASE_NAME:wifi_gspi_merged>.hex"
    COMMAND ${CMAKE_OBJCOPY} ${OBJCOPY_BIN_CMD}  "$<TARGET_FILE:wifi_gspi_merged>" "$<TARGET_FILE_DIR:wifi_gspi_merged>/$<TARGET_FILE_BASE_NAME:wifi_gspi_merged>.bin" 
)

# Run post-build pipeline to perform additional post-processing
if(post_build_command)
add_custom_command(TARGET wifi_gspi_merged
    POST_BUILD
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/..
    COMMAND ${post_build_command}
)
endif()
